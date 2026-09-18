//! The interface task: the encoder of the cabinet and the control link.
//!
//! One thread owns both. It reads the pulse counter the encoder feeds, takes
//! the fine control the phone set, and hands the pair to
//! `pulsar_lib::panel::Uplink`, which decides what leaves on the link. The
//! thread sits below the pump and blocks on the scheduler between two polls,
//! so it holds the processor only for the work of one poll and the audio path
//! never waits on it. It also prints each state the phone passes through and
//! each step the encoder lands on, which keeps the console off the radio task
//! the bridge is fed from.
//!
//! The encoder costs the processor nothing between two readings: the pulse
//! counter counts the edges in hardware and its glitch filter drops the
//! shortest noise. Its interrupt fires once per `COUNT_LIMIT` counts, which is
//! thousands of turns apart.
//!
//! The link is a UART at `pulsar_lib::link::LINK_BAUD`, eight data bits, no
//! parity, one stop bit, on the pins of the cabinet harness.

use std::thread;
use std::time::{Duration, Instant};

use esp_idf_svc::hal::delay::TICK_RATE_HZ;
use esp_idf_svc::hal::gpio::{AnyIOPin, InputPin, OutputPin};
use esp_idf_svc::hal::pcnt::PcntUnitDriver;
use esp_idf_svc::hal::pcnt::config::
{
    ChannelConfig,
    ChannelEdgeAction,
    ChannelLevelAction,
    GlitchFilterConfig,
    UnitConfig,
};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::hal::uart::config::{Config, DataBits, StopBits};
use esp_idf_svc::hal::uart::{Uart, UartDriver};
use esp_idf_svc::hal::units::Hertz;
use esp_idf_svc::sys::EspError;
use pulsar_lib::link::LINK_BAUD;
use pulsar_lib::panel::{CoarseControl, Uplink, volume_of};
use pulsar_lib::protocol::{MAX_FRAME_LEN, Message, ToDsp};

use crate::StartError;
use crate::avrcp;

/// Stack of the interface thread, which holds one frame and two drivers.
const UI_STACK_BYTES: usize = 4096;

/// `FreeRTOS` priority of the interface thread, below the pump so that the
/// audio path never waits on a detent or on a frame.
const UI_PRIORITY: u8 = 5;

/// Milliseconds between two readings of the encoder, and between two polls of
/// the schedule. A hand on the knob produces a detent every few tens of
/// milliseconds at most.
///
/// It holds at least one scheduler tick, which the assertion below pins to the
/// tick rate of the build.
const POLL_PERIOD_MS: u64 = 10;

/// Time one turn of the loop waits before it reads the controls again.
const POLL_PERIOD: Duration = Duration::from_millis(POLL_PERIOD_MS);

const _: () = assert!
(
    POLL_PERIOD_MS * TICK_RATE_HZ as u64 >= 1_000,
    "a delay shorter than one scheduler tick busy waits on the processor \
     instead of blocking, so the interface task never yields to the idle task \
     and the task watchdog fires"
);

/// Counts the hardware counter runs to before it wraps into the accumulator.
const COUNT_LIMIT: i32 = 32_767;

/// Pulses shorter than this are noise and do not reach the counter.
const GLITCH_FILTER: Duration = Duration::from_micros(1);

/// What a write on the control link fails with.
#[derive(Debug)]
enum SendError
{
    /// The driver refused the write.
    Esp(EspError),
    /// The message did not fit the frame buffer.
    Frame,
}

/// Brings up the link and the encoder, and starts the interface thread.
///
/// `pin_a` and `pin_b` are the two channels of the encoder, held high by
/// external resistors. `tx` and `rx` are the control link.
///
/// # Errors
///
/// `Esp` when a driver refuses its configuration, `Spawn` when the thread does
/// not start.
pub(crate) fn start<U, T, R, A, B>
(
    uart: U,
    tx: T,
    rx: R,
    pin_a: A,
    pin_b: B,
) -> Result<(), StartError>
where
    U: Uart + 'static,
    T: OutputPin + 'static,
    R: InputPin + 'static,
    A: InputPin + 'static,
    B: InputPin + 'static,
{
    let config = Config::new()
        .baudrate(Hertz(LINK_BAUD))
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::STOP1)
        .queue_size(0);
    let link = UartDriver::new(uart, tx, rx, None::<AnyIOPin>, None::<AnyIOPin>, &config)?;
    let encoder = build_encoder(pin_a, pin_b)?;

    ThreadSpawnConfiguration
    {
        name: Some(c"pulsar_ui"),
        stack_size: UI_STACK_BYTES,
        priority: UI_PRIORITY,
        ..Default::default()
    }
    .set()?;

    let spawned = thread::Builder::new()
        .stack_size(UI_STACK_BYTES)
        .spawn(move || run(link, encoder));

    ThreadSpawnConfiguration::default().set()?;
    spawned.map(drop).map_err(StartError::Spawn)
}

/// Builds the pulse counter that decodes the encoder.
///
/// One channel counts both edges of A and takes its direction from the level
/// of B, so one detent of the cabinet encoder moves the count by
/// `pulsar_lib::panel::COUNTS_PER_DETENT`. On the harness of the cabinet, which
/// wires CLK to A and DT to B, the count rises when the knob turns clockwise
/// and the level rises with it. Swapping the two edge actions below, or the two
/// wires, reverses that.
///
/// The accumulator carries what the hardware counter drops when it reaches
/// `COUNT_LIMIT`, so the reading grows monotonically with the knob.
fn build_encoder<A, B>(pin_a: A, pin_b: B) -> Result<PcntUnitDriver<'static>, EspError>
where
    A: InputPin + 'static,
    B: InputPin + 'static,
{
    let mut encoder = PcntUnitDriver::new
    (
        &UnitConfig
        {
            low_limit: -COUNT_LIMIT,
            high_limit: COUNT_LIMIT,
            accum_count: true,
            ..Default::default()
        }
    )?;

    encoder.set_glitch_filter
    (
        Some
        (
            &GlitchFilterConfig
            {
                max_glitch: GLITCH_FILTER,
                ..Default::default()
            }
        )
    )?;

    encoder
        .add_channel(Some(pin_a), Some(pin_b), &ChannelConfig::default())?
        .set_edge_action(ChannelEdgeAction::Decrease, ChannelEdgeAction::Increase)?
        .set_level_action(ChannelLevelAction::Keep, ChannelLevelAction::Inverse)?;

    encoder.enable()?;
    encoder.add_watch_points_and_clear([-COUNT_LIMIT, COUNT_LIMIT])?;
    encoder.start()?;
    Ok(encoder)
}

/// Reads the controls and feeds the link, for ever.
///
/// A failing counter leaves the step where it was, and a failing link leaves
/// the schedule running: the processing board takes the silence for a lost
/// link and ramps the gain down on its own.
#[expect
(
    clippy::needless_pass_by_value,
    reason = "the thread owns both drivers for the life of the program, and \
              dropping either one deletes the peripheral it drives"
)]
fn run(link: UartDriver<'static>, encoder: PcntUnitDriver<'static>) -> !
{
    let mut coarse = CoarseControl::at_power_up(encoder.get_count().unwrap_or(0));
    let mut uplink = Uplink::at_power_up();
    let mut frame = [0_u8; MAX_FRAME_LEN];
    let mut failing = false;
    let mut last = Instant::now();
    let mut reported = avrcp::phone_volume();
    let mut reported_coarse = coarse.coarse();

    loop
    {
        thread::sleep(POLL_PERIOD);

        // The whole milliseconds are consumed and the rest is left on the
        // instant, so the schedule keeps the time the poll really took.
        let elapsed_ms = u32::try_from(last.elapsed().as_millis()).unwrap_or(u32::MAX);
        last += Duration::from_millis(u64::from(elapsed_ms));

        // The reading of the counter travels with the step it lands on, so a
        // known number of detents turned between two lines gives the counts one
        // detent of the knob really produces.
        if let Ok(count) = encoder.get_count()
        {
            let step = coarse.read(count);
            if step != reported_coarse
            {
                reported_coarse = step;
                println!("coarse step {step}, counter {count}");
            }
        }

        // The radio task folds the AVRCP events into one word and writes no
        // line of its own, so the console never paces the task the bridge is
        // fed from. This is where a phone state reaches the log.
        let phone = avrcp::phone_volume();
        if phone != reported
        {
            reported = phone;
            println!("phone volume {phone:?}, fine {}", phone.fine());
        }

        let volume = volume_of(coarse.coarse(), phone);
        let Some(message) = uplink.poll(elapsed_ms, volume)
        else
        {
            continue;
        };

        match send(&link, &mut frame, message)
        {
            Ok(()) => failing = false,
            Err(error) =>
            {
                if !failing
                {
                    match error
                    {
                        SendError::Esp(error) => println!("control link write failed: {error}"),
                        SendError::Frame => println!("a control message outgrew its frame buffer"),
                    }
                }
                failing = true;
            }
        }
    }
}

/// Writes `message` on the link as one frame.
///
/// # Errors
///
/// `Frame` when the message does not fit the buffer, which the protocol sizes
/// from the widest message, and `Esp` when the driver refuses the write.
fn send
(
    link: &UartDriver<'static>,
    frame: &mut [u8; MAX_FRAME_LEN],
    message: ToDsp,
) -> Result<(), SendError>
{
    let used = message.encode_into(frame).map_err(|_| SendError::Frame)?;
    let bytes = frame.get(..used).ok_or(SendError::Frame)?;
    link.write(bytes).map_err(SendError::Esp)?;
    Ok(())
}
