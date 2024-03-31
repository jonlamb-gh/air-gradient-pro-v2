use crate::{
    common::{Measurement, MeasurementReceiver},
    config,
    display::SystemStatus,
    tasks::display::{Message as DisplayMessage, MessageSender as DisplayMessageSender},
    util,
};
use defmt::{debug, warn};
use embassy_futures::select::{select, Either};
use embassy_net::{udp::UdpSocket, Ipv4Address};
use embassy_time::{Duration, Ticker};
use wire_protocols::{
    broadcast::{Message as WireMessage, Repr as Message, MESSAGE_LEN},
    DateTime, DeviceSerialNumber, ProtocolVersion, StatusFlags,
};

const LOCAL_EPHEMERAL_PORT: u16 = 16000;

pub struct DataManagerTaskState {
    measurement_recvr: MeasurementReceiver,
    display_sender: DisplayMessageSender,
    bcast_socket: UdpSocket<'static>,
    msg_buffer: [u8; MESSAGE_LEN],
}

impl DataManagerTaskState {
    pub fn new(
        measurement_recvr: MeasurementReceiver,
        display_sender: DisplayMessageSender,
        bcast_socket: UdpSocket<'static>,
    ) -> Self {
        Self {
            measurement_recvr,
            display_sender,
            bcast_socket,
            msg_buffer: [0; MESSAGE_LEN],
        }
    }
}

#[embassy_executor::task]
pub async fn data_manager_task(state: DataManagerTaskState) -> ! {
    let DataManagerTaskState {
        measurement_recvr,
        display_sender,
        mut bcast_socket,
        mut msg_buffer,
    } = state;
    let mut msg = default_bcast_message();
    let mut cycles_till_warmed_up = config::DATA_MANAGER_WARM_UP_PERIOD_CYCLES;
    let mut bcast_ticker = Ticker::every(Duration::from_secs(config::BCAST_INTERVAL_SEC));

    debug!("DM: initializing data manager state");
    msg.device_serial_number = util::read_device_serial_number();
    msg.status_flags.set_initialized(true);

    bcast_ticker.reset();
    loop {
        let mut send_msg = false;

        // TODO timeout gets reset every time we recv a Measurement ??
        // maybe use a ticker for this instead?
        match select(measurement_recvr.receive(), bcast_ticker.next()).await {
            Either::First(measurement) => match measurement {
                Measurement::Sht31(m) => {
                    msg.temperature = m.temperature;
                    msg.humidity = m.humidity;
                    msg.status_flags.set_temperature_valid(true);
                    msg.status_flags.set_humidity_valid(true);
                }
                Measurement::Sgp41(m) => {
                    msg.voc_ticks = m.voc_ticks;
                    msg.nox_ticks = m.nox_ticks;
                    msg.status_flags.set_voc_ticks_valid(true);
                    msg.status_flags.set_nox_ticks_valid(true);
                }
                Measurement::GasIndices(m) => {
                    // The gas indices are valid once they are non-zero
                    if let Some(i) = m.voc_index {
                        msg.voc_index = i.get();
                        msg.status_flags.set_voc_index_valid(true);
                    }
                    if let Some(i) = m.nox_index {
                        msg.nox_index = i.get();
                        msg.status_flags.set_nox_index_valid(true);
                    }
                }
                Measurement::Pms5003(m) => {
                    msg.pm2_5_atm = m.pm2_5_atm;
                    msg.status_flags.set_pm2_5_valid(true);
                }
                Measurement::S8Lp(m) => {
                    msg.co2 = m.co2;
                    msg.status_flags.set_co2_valid(true);
                }
            },
            Either::Second(_) => {
                // TODO invalidate stale fields on timer or keep valid?

                if cycles_till_warmed_up != 0 {
                    cycles_till_warmed_up = cycles_till_warmed_up.saturating_sub(1);

                    if cycles_till_warmed_up == 0 {
                        debug!("DM: warm up period complete");
                    }
                } else {
                    send_msg = true;
                }

                msg.uptime_seconds += config::BCAST_INTERVAL_SEC as u32;
            }
        }

        if send_msg {
            if !bcast_socket.is_open() {
                bcast_socket.bind(LOCAL_EPHEMERAL_PORT).unwrap();
            }
            if bcast_socket.may_send() {
                let endpoint = (
                    Ipv4Address(config::BROADCAST_ADDRESS),
                    config::BROADCAST_PORT,
                );
                let mut wire = WireMessage::new_unchecked(&mut msg_buffer);
                msg.emit(&mut wire);

                match bcast_socket
                    .send_to(&msg_buffer[..MESSAGE_LEN], endpoint)
                    .await
                {
                    Err(e) => warn!("Failed to send broadcast message. {:?}", e),
                    Ok(()) => {
                        debug!("DM: Sent message sn {}", msg.sequence_number);
                        msg.sequence_number = msg.sequence_number.wrapping_add(1);
                    }
                }
            } else {
                warn!("Socket cannot send");
                bcast_socket.close();
            }

            let sys_status = SystemStatus {
                pm2_5: msg.status_flags.pm2_5_valid().then_some(msg.pm2_5_atm),
                temp: msg
                    .status_flags
                    .temperature_valid()
                    .then_some(msg.temperature),
                humidity: msg.status_flags.humidity_valid().then_some(msg.humidity),
                co2: msg.status_flags.co2_valid().then_some(msg.co2),
                voc_index: msg.status_flags.voc_index_valid().then_some(msg.voc_index),
                nox_index: msg.status_flags.nox_index_valid().then_some(msg.nox_index),
                msg_seqnum: msg.sequence_number,
            };

            display_sender
                .send(DisplayMessage::SystemStatus(sys_status))
                .await;
        }
    }
}

const fn default_bcast_message() -> Message {
    Message {
        protocol_version: ProtocolVersion::v1(),
        firmware_version: config::FIRMWARE_VERSION,
        device_id: config::DEVICE_ID,
        device_serial_number: DeviceSerialNumber::zero(),
        sequence_number: 0,
        uptime_seconds: 0,
        status_flags: StatusFlags::empty(),
        datetime: DateTime::zero(),
        temperature: 0,
        humidity: 0,
        voc_ticks: 0,
        nox_ticks: 0,
        voc_index: 0,
        nox_index: 0,
        pm2_5_atm: 0,
        co2: 0,
    }
}
