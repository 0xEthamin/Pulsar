//! The fine volume control, as the phone drives it over AVRCP.
//!
//! The cabinet is the target of the remote control connection, so a phone that
//! carries absolute volume sets the fine control by sending it, and this board
//! answers a registration for the volume change notification. The rules live in
//! `pulsar_lib::panel::PhoneVolume`, which decides what each event means and
//! what the fine control becomes. This module is the glue: it turns Bluedroid
//! events into `PhoneEvent`, keeps the state in one word, and answers the
//! notification.
//!
//! `esp-idf-svc` 0.52.1 exposes the controller role of AVRCP and not the
//! target role, so the five calls below go to ESP-IDF directly.
//!
//! The cabinet never completes a volume change notification, since the only
//! control it owns is the coarse step of the encoder and that step is not the
//! absolute volume of the profile. A phone therefore keeps the volume it
//! arrived with, and the notification response carries the value the phone
//! itself last set.
//!
//! Bluedroid couples AVRCP to A2DP: the target has to be initialised before
//! the sink, or the initialisation fails.

use core::sync::atomic::{AtomicU16, Ordering};

use esp_idf_svc::sys::
{
    EspError,
    esp,
    esp_avrc_bit_mask_op_t_ESP_AVRC_BIT_MASK_OP_SET,
    esp_avrc_feature_flag_t_ESP_AVRC_FEAT_FLAG_CAT2,
    esp_avrc_init_state_t_ESP_AVRC_INIT_SUCCESS,
    esp_avrc_rn_event_ids_t_ESP_AVRC_RN_VOLUME_CHANGE,
    esp_avrc_rn_evt_bit_mask_operation,
    esp_avrc_rn_evt_cap_mask_t,
    esp_avrc_rn_param_t,
    esp_avrc_rn_rsp_t_ESP_AVRC_RN_RSP_INTERIM,
    esp_avrc_tg_cb_event_t,
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_CONNECTION_STATE_EVT,
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_PROF_STATE_EVT,
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT,
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_REMOTE_FEATURES_EVT,
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT,
    esp_avrc_tg_cb_param_t,
    esp_avrc_tg_init,
    esp_avrc_tg_register_callback,
    esp_avrc_tg_send_rn_rsp,
    esp_avrc_tg_set_rn_evt_cap,
};
use pulsar_lib::panel::{PhoneEvent, PhoneVolume};

/// The remote control connection came up or went away.
const CONNECTION_STATE: esp_avrc_tg_cb_event_t = esp_avrc_tg_cb_event_t_ESP_AVRC_TG_CONNECTION_STATE_EVT;

/// The service record of the phone as a controller was read.
const REMOTE_FEATURES: esp_avrc_tg_cb_event_t = esp_avrc_tg_cb_event_t_ESP_AVRC_TG_REMOTE_FEATURES_EVT;

/// The phone set an absolute volume.
const SET_ABSOLUTE_VOLUME: esp_avrc_tg_cb_event_t =
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT;

/// The phone registered for a notification.
const REGISTER_NOTIFICATION: esp_avrc_tg_cb_event_t =
    esp_avrc_tg_cb_event_t_ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT;

/// The target finished coming up or going down.
const PROFILE_STATE: esp_avrc_tg_cb_event_t = esp_avrc_tg_cb_event_t_ESP_AVRC_TG_PROF_STATE_EVT;

/// What the phone carries, as one word the radio task writes and the interface
/// task reads. It starts on silence.
static PHONE: AtomicU16 = AtomicU16::new(PhoneVolume::SILENT_BITS);

/// Returns what the phone currently carries.
pub(crate) fn phone_volume() -> PhoneVolume
{
    PhoneVolume::from_bits(PHONE.load(Ordering::Acquire))
}

/// Drops the fine control back to silence, for a phone that leaves without
/// closing its remote control connection first.
pub(crate) fn forget_phone()
{
    record(PhoneEvent::Disconnected);
}

/// Notes that an audio connection came up.
///
/// A remote control connection outlives an audio one and the stack reports it
/// only when it opens, so without this an audio connection cycling under a
/// surviving control connection would leave the fine control at silence until
/// the next power up. It raises no level of its own: the state it reaches has
/// heard nothing from the phone, so the fine control stays at 0.
pub(crate) fn note_media_connection()
{
    record(PhoneEvent::MediaConnected);
}

/// Brings up the AVRCP target and announces the volume change notification.
///
/// Call this after the Bluetooth stack is enabled and before the A2DP sink
/// starts. Without the announced capability a phone gets its registration
/// refused and falls back to scaling its own stream.
///
/// `esp_avrc_tg_init` returns as soon as its message is queued, so the verdict
/// of the target coming up is not in the return code: it arrives later as a
/// profile state event, which `on_target_event` reports.
///
/// # Errors
///
/// `EspError` when a call of the stack fails.
#[allow(unsafe_code)]
pub(crate) fn start_target() -> Result<(), EspError>
{
    // SAFETY: the callback is a plain function with the signature ESP-IDF
    // declares, with no state of its own besides one atomic. The two calls
    // take no pointer and report through their return code.
    esp!(unsafe { esp_avrc_tg_register_callback(Some(on_target_event)) })?;
    esp!(unsafe { esp_avrc_tg_init() })?;

    let mut events = esp_avrc_rn_evt_cap_mask_t { bits: 0 };
    // SAFETY: both calls read and write through a pointer to a local that
    // lives across them, and neither keeps it. `esp_avrc_tg_set_rn_evt_cap`
    // copies the mask into the message it posts.
    unsafe
    {
        esp_avrc_rn_evt_bit_mask_operation
        (
            esp_avrc_bit_mask_op_t_ESP_AVRC_BIT_MASK_OP_SET,
            &raw mut events,
            esp_avrc_rn_event_ids_t_ESP_AVRC_RN_VOLUME_CHANGE,
        );
        esp!(esp_avrc_tg_set_rn_evt_cap(&raw const events))
    }
}

/// Folds `event` into the shared state and returns what it became.
fn record(event: PhoneEvent) -> PhoneVolume
{
    let mut state = PhoneVolume::Absent;
    let _ = PHONE.fetch_update
    (
        Ordering::AcqRel,
        Ordering::Acquire,
        |bits|
        {
            state = PhoneVolume::from_bits(bits).apply(event);
            Some(state.to_bits())
        },
    );
    state
}

/// Answers a volume change registration with the value the phone last set.
///
/// The stack copies the whole parameter union out of the pointer it is given,
/// so the union is written whole and the widest member is what zeroes it.
///
/// A refusal is not reported from here. A phone registers as often as it likes
/// and the console of this task writes by polling, so a line on that path would
/// put the radio task on the console at a rate the phone picks. A registration
/// that never turns into a value is what a refusal looks like, and the
/// interface task prints every state the phone passes through.
#[allow(unsafe_code)]
fn answer_registration(volume: u8)
{
    let mut reported = esp_avrc_rn_param_t { elm_id: [0; 8] };
    reported.volume = volume;

    // SAFETY: the pointer names a local that lives across the call, every byte
    // of it is initialised, and the stack copies the union into the message it
    // posts rather than keeping the pointer.
    let _ = unsafe
    {
        esp_avrc_tg_send_rn_rsp
        (
            esp_avrc_rn_event_ids_t_ESP_AVRC_RN_VOLUME_CHANGE,
            esp_avrc_rn_rsp_t_ESP_AVRC_RN_RSP_INTERIM,
            &raw mut reported,
        )
    };
}

/// Handles one AVRCP target event from the Bluedroid task.
///
/// It reads one field of the parameter union, folds the event into the shared
/// state and returns. Nothing here waits on the interface task or the audio
/// path, and nothing a phone can repeat writes to the console: the interface
/// task is what reports a state change. The profile state is the exception,
/// since the stack raises it once when the target comes up and once when it
/// goes down, neither of which a phone can drive.
#[allow(unsafe_code)]
unsafe extern "C" fn on_target_event(event: esp_avrc_tg_cb_event_t, param: *mut esp_avrc_tg_cb_param_t)
{
    // SAFETY: Bluedroid passes a pointer to the parameter union of the event,
    // valid for the call, and fills the member that event names.
    let Some(param) = (unsafe { param.as_ref() })
    else
    {
        return;
    };

    match event
    {
        CONNECTION_STATE =>
        {
            // SAFETY: the connection state event fills `conn_stat`.
            let connected = unsafe { param.conn_stat }.connected;
            record(if connected
            {
                PhoneEvent::Connected
            }
            else
            {
                PhoneEvent::Disconnected
            });
        }
        REMOTE_FEATURES =>
        {
            // SAFETY: the remote features event fills `rmt_feats`.
            let flags = unsafe { param.rmt_feats }.ct_feat_flag;
            let category_two = u32::from(flags) & esp_avrc_feature_flag_t_ESP_AVRC_FEAT_FLAG_CAT2 != 0;
            record(PhoneEvent::Controller { category_two });
        }
        SET_ABSOLUTE_VOLUME =>
        {
            // SAFETY: the absolute volume event fills `set_abs_vol`.
            let octet = unsafe { param.set_abs_vol }.volume;
            record(PhoneEvent::AbsoluteVolume(octet));
        }
        REGISTER_NOTIFICATION =>
        {
            // SAFETY: the registration event fills `reg_ntf`.
            let identifier = unsafe { param.reg_ntf }.event_id;
            if u32::from(identifier) == esp_avrc_rn_event_ids_t_ESP_AVRC_RN_VOLUME_CHANGE
            {
                let state = record(PhoneEvent::VolumeNotificationRegistered);
                answer_registration(state.reported());
            }
        }
        PROFILE_STATE =>
        {
            // SAFETY: the profile state event fills `avrc_tg_init_stat`.
            let state = unsafe { param.avrc_tg_init_stat }.state;
            if state == esp_avrc_init_state_t_ESP_AVRC_INIT_SUCCESS
            {
                println!("AVRCP target up");
            }
            else
            {
                // A target that did not come up raises no event at all after
                // this one, so the fine volume stays at 0 and the cabinet is
                // silent. This line is what tells that apart from a phone
                // without absolute volume.
                println!("AVRCP target not up, init state {state}: the cabinet stays silent");
            }
        }
        _ => {}
    }
}
