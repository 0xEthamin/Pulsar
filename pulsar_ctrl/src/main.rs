//! Bluetooth receiver and user interface firmware for the ESP32.
//!
//! This board owns the radio link, the SBC decode and everything the user
//! touches. It forwards the decoded samples to the processing board untouched,
//! with no filtering, no gain and no limiting.
//!
//! The audio path outranks every other task here.
//!
//! The Bluedroid A2DP sink hands decoded PCM to `pulsar_lib::bridge` from the
//! Bluetooth callback task. A dedicated thread pumps the bridge into the I2S
//! transmit channel, which runs as target on the clocks of the processing
//! board. The two share the bridge through a mutex that each side holds for one
//! copy: the push of one decoded chunk, or the drain of one block, which also
//! runs the drift correction. On ESP-IDF
//! `std::sync::Mutex` is a pthread mutex, which ESP-IDF builds on a `FreeRTOS`
//! mutex with priority inheritance. The pump thread logs the statistics of the
//! bridge every `STATS_PERIOD_BLOCKS` blocks, and the callback task logs none.
//!
//! Nothing here writes non-volatile storage. Bluedroid starts without NVS and
//! keeps no bond across a reset, so a phone pairs again after each boot.

use std::convert::Infallible;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;

use esp_idf_svc::bt::a2dp::{A2dpEvent, AudioStatus, Codec, ConnectionStatus, EspA2dp};
use esp_idf_svc::bt::gap::{DiscoveryMode, EspGap, GapEvent};
use esp_idf_svc::bt::{BtClassic, BtDriver};
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::i2s::config::{
    Config,
    DataBitWidth,
    Role,
    SlotMode,
    StdClkConfig,
    StdConfig,
    StdGpioConfig,
    StdSlotConfig,
};
use esp_idf_svc::hal::i2s::{I2sDriver, I2sTx};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::sys::{ESP_ERR_TIMEOUT, EspError, link_patches, uxTaskGetStackHighWaterMark};
use pulsar_lib::bridge::{
    self,
    Bridge,
    BridgeStats,
    FrameSource,
    SlotWriter,
    StreamCodec,
    StreamVerdict,
};
use pulsar_lib::constants::SAMPLE_RATE_HZ;

/// Name the board advertises to phones.
const DEVICE_NAME: &str = "Pulsar";

/// Stack of the pump thread, which holds one PCM block and one slot block.
const PUMP_STACK_BYTES: usize = 8192;

/// `FreeRTOS` priority of the pump thread, above the default pthread priority of 5.
const PUMP_PRIORITY: u8 = 15;

/// Time one channel write waits for room in the DMA queue.
const WRITE_TIMEOUT: TickType = TickType::new_millis(100);

/// Pause after a failed pump, so a channel that keeps failing does not spin.
const PUMP_RETRY: Duration = Duration::from_millis(100);

/// Pumped blocks between two statistics lines, 5.0 s of blocks of
/// `bridge::BLOCK_FRAMES` frames at 44.1 kHz.
const STATS_PERIOD_BLOCKS: u32 = 919;

/// Bridge between the A2DP callback and the pump thread.
static BRIDGE: Mutex<Bridge<{ bridge::RING_FRAMES }>> =
    Mutex::new(Bridge::new(bridge::PREFILL_FRAMES));

/// Failure while bringing the board up.
#[derive(Debug)]
enum StartError
{
    /// An ESP-IDF call failed.
    Esp(EspError),
    /// The pump thread did not start.
    Spawn(std::io::Error),
}

impl From<EspError> for StartError
{
    fn from(error: EspError) -> Self
    {
        Self::Esp(error)
    }
}

/// The shared bridge, seen from the pump thread.
struct SharedBridge;

impl FrameSource for SharedBridge
{
    /// Drains one block under the lock. A poisoned lock gives out nothing,
    /// which the pump turns into silence.
    fn take_frames(&mut self, pcm: &mut [u8]) -> usize
    {
        BRIDGE.lock().map_or(0, |mut shared| shared.drain_into(pcm))
    }
}

/// The I2S transmit channel.
struct Link(I2sDriver<'static, I2sTx>);

impl SlotWriter for Link
{
    type Error = EspError;

    /// Writes a prefix of `bytes`. A timeout with nothing written returns 0.
    fn write(&mut self, bytes: &[u8]) -> Result<usize, EspError>
    {
        match self.0.write(bytes, WRITE_TIMEOUT.ticks())
        {
            Err(error) if error.code() == ESP_ERR_TIMEOUT => Ok(0),
            other => other,
        }
    }
}

/// Runs `action` on the shared bridge. A poisoned lock skips it.
fn with_bridge(action: impl FnOnce(&mut Bridge<{ bridge::RING_FRAMES }>))
{
    if let Ok(mut shared) = BRIDGE.lock()
    {
        action(&mut shared);
    }
}

/// Starts a stream on the codec the phone negotiated.
///
/// `pulsar_lib::bridge::stream_verdict` decides. A refused stream leaves the
/// bridge closed, so the link carries silence, and logs one line.
fn start_stream(codec: &Codec)
{
    let stream = match codec
    {
        Codec::Sbc([octet, ..]) => StreamCodec::Sbc(*octet),
        _ => StreamCodec::Other,
    };

    let mut verdict = None;
    with_bridge(|shared| verdict = Some(shared.open(stream)));

    match verdict
    {
        Some(StreamVerdict::Forward(_)) => {}
        Some(refused) => println!
        (
            "A2DP stream refused ({refused:?}), the chain takes SBC at {SAMPLE_RATE_HZ} Hz: forwarding silence"
        ),
        None => println!("A2DP stream not started, the bridge lock is poisoned"),
    }
}

/// Handles one A2DP event from the Bluetooth callback task.
fn on_a2dp(event: A2dpEvent<'_>) -> usize
{
    match event
    {
        A2dpEvent::SinkData(pcm) =>
        {
            with_bridge(|shared|
            {
                shared.push(pcm);
            });
        }
        A2dpEvent::AudioCodecConfigured { codec, .. } => start_stream(&codec),
        A2dpEvent::AudioState { status, .. } if status != AudioStatus::Started =>
        {
            with_bridge(Bridge::flush);
        }
        A2dpEvent::ConnectionState { status: ConnectionStatus::Disconnected, .. } =>
        {
            with_bridge(Bridge::close);
        }
        _ => {}
    }

    0
}

/// Logs the outcome of a pairing.
fn on_gap(event: &GapEvent<'_>)
{
    if let GapEvent::AuthenticationCompleted { bd_addr, status, .. } = event
    {
        println!("pairing with {bd_addr}: {status:?}");
    }
}

/// Returns the least free stack, in bytes, the calling task has had since it
/// started.
#[allow(unsafe_code)]
fn stack_headroom() -> u32
{
    // SAFETY: ESP-IDF FreeRTOS `uxTaskGetStackHighWaterMark` reads the stack of
    // the task it names and, given a null handle, of the calling task, which
    // is alive for the whole call. It writes nothing and returns bytes.
    unsafe { uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

/// Logs one line of `stats` with the stack headroom of the pump thread, unless
/// the period saw no stream at all.
fn log_stats(stats: &BridgeStats)
{
    if stats.samples == 0 && stats.dropped == 0 && stats.underruns == 0 && stats.trimmed == 0
    {
        return;
    }

    println!
    (
        "bridge: fill {}..{} mean {} over {} drains, corrections +{} -{}, dropped {}, trimmed {}, underruns {}, pump stack free {} of {} bytes",
        stats.fill_min,
        stats.fill_max,
        stats.fill_mean,
        stats.samples,
        stats.inserted,
        stats.removed,
        stats.dropped,
        stats.trimmed,
        stats.underruns,
        stack_headroom(),
        PUMP_STACK_BYTES
    );
}

/// Pumps the bridge into the link, one block at a time, for ever.
///
/// Every `STATS_PERIOD_BLOCKS` pumps it takes the statistics of the bridge
/// under the lock and logs them outside it.
fn run_pump(mut link: Link) -> !
{
    let mut pcm = [0; bridge::BLOCK_FRAMES * bridge::PCM_FRAME_BYTES];
    let mut slots = [0; bridge::BLOCK_FRAMES * bridge::SLOT_FRAME_BYTES];
    let mut failing = false;
    let mut pumped: u32 = 0;

    loop
    {
        pumped = pumped.saturating_add(1);

        if pumped >= STATS_PERIOD_BLOCKS
        {
            pumped = 0;
            let mut stats = None;
            with_bridge(|shared| stats = Some(shared.take_stats()));

            if let Some(stats) = stats
            {
                log_stats(&stats);
            }
        }

        match bridge::pump(&mut SharedBridge, &mut pcm, &mut slots, &mut link)
        {
            Ok(_) => failing = false,
            Err(error) =>
            {
                if !failing
                {
                    println!("I2S pump failed: {error:?}");
                }

                failing = true;
                thread::sleep(PUMP_RETRY);
            }
        }
    }
}

/// Starts the pump thread on `link` at `PUMP_PRIORITY`.
///
/// # Errors
///
/// `Esp` when the thread configuration is refused, `Spawn` when the thread
/// does not start.
fn spawn_pump(link: Link) -> Result<(), StartError>
{
    ThreadSpawnConfiguration
    {
        name: Some(c"pulsar_pump"),
        stack_size: PUMP_STACK_BYTES,
        priority: PUMP_PRIORITY,
        ..Default::default()
    }
    .set()?;

    let spawned = thread::Builder::new()
        .stack_size(PUMP_STACK_BYTES)
        .spawn(move || run_pump(link));

    ThreadSpawnConfiguration::default().set()?;
    spawned.map(drop).map_err(StartError::Spawn)
}

fn main()
{
    // The ESP-IDF link step drops these symbols without an explicit reference.
    link_patches();
    EspLogger::initialize_default();

    match run()
    {
        Err(StartError::Esp(error)) => println!("start failed: {error}"),
        Err(StartError::Spawn(error)) => println!("pump thread failed to start: {error}"),
    }
}

/// Brings up the link, the pump and the A2DP sink, then parks for ever.
///
/// # Errors
///
/// `StartError` when a step of the bring up fails. A pump thread already
/// started keeps running and writes silence.
fn run() -> Result<Infallible, StartError>
{
    let peripherals = Peripherals::take()?;

    let config = StdConfig::new
    (
        Config::default().role(Role::Target).auto_clear(true),
        StdClkConfig::from_sample_rate_hz(SAMPLE_RATE_HZ),
        StdSlotConfig::philips_slot_default(DataBitWidth::Bits32, SlotMode::Stereo),
        StdGpioConfig::default(),
    );

    let mut channel = I2sDriver::new_std_tx
    (
        peripherals.i2s0,
        &config,
        peripherals.pins.gpio26,
        peripherals.pins.gpio22,
        None::<AnyIOPin>,
        peripherals.pins.gpio25,
    )?;
    channel.tx_enable()?;
    spawn_pump(Link(channel))?;

    let driver = BtDriver::<BtClassic>::new(peripherals.modem, None)?;
    let gap = EspGap::new(&driver)?;
    gap.subscribe(|event| on_gap(&event))?;
    let a2dp = EspA2dp::new_sink(&driver)?;
    a2dp.subscribe(on_a2dp)?;
    gap.set_device_name(DEVICE_NAME)?;
    gap.set_scan_mode(true, DiscoveryMode::Discoverable)?;

    loop
    {
        thread::park();
    }
}
