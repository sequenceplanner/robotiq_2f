use futures::stream::Stream;
use futures::stream::StreamExt;
use r2r::robotiq_2f_msgs::msg::CommandState as CommandMsg;
use r2r::robotiq_2f_msgs::msg::MeasuredState as MeasuredMsg;
use r2r::QosProfile;
use std::cmp::*;
use std::sync::{Arc, Mutex};
use std::{thread, time};
use tokio_modbus::client::Context;
use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

#[derive(Debug, Default, Clone, Copy)]
struct MeasuredState {
    pub g_act: u8, // activation status
    pub g_gto: u8, // action status
    pub g_sta: u8, // gripper status
    pub g_obj: u8, // object detection status
    pub g_flt: u8, // fault status
    pub g_pr: u8,  // position request echo
    pub g_po: u8,  // position
    pub g_cu: u8,  // current
}

#[derive(Debug, Default, Clone, Copy)]
struct CommandState {
    pub r_act: u8, // action request
    pub r_gto: u8, // go to request
    pub r_atr: u8, // automatic release routine
    pub r_ard: u8, // automatic release direction
    pub r_pr: u8,  // position request
    pub r_sp: u8,  // speed
    pub r_fr: u8,  // force
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "robotiq_2f_driver", "")?;
    let subscriber = node.subscribe::<CommandMsg>("/robotiq_2f_command", QosProfile::default())?;
    let pub_timer = node.create_wall_timer(std::time::Duration::from_millis(10))?;
    let publisher =
        node.create_publisher::<MeasuredMsg>("/robotiq_2f_measured", QosProfile::default())?;

    let tty_path = "/dev/ttyUSB0";

    let msr_state = Arc::new(Mutex::new(MeasuredState::default()));
    let msr_state_clone_1 = msr_state.clone();
    let msr_state_clone_2 = msr_state.clone();

    let cmd_state = Arc::new(Mutex::new(CommandState::default()));
    let cmd_state_clone_1 = cmd_state.clone();
    let cmd_state_clone_2 = cmd_state.clone();
    let cmd_state_clone_3 = cmd_state.clone();
    let cmd_state_clone_4 = cmd_state.clone();

    tokio::task::spawn(async move {
        modbus_client(tty_path, msr_state_clone_1, cmd_state_clone_3).await;
    });

    reset_on_startup(cmd_state_clone_1).await;
    activate_on_startup(cmd_state_clone_2).await;

    tokio::task::spawn(async move {
        match publisher_callback(publisher, pub_timer, msr_state_clone_2).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!("robotiq_2f_driver", "Publisher failed with {}.", e),
        };
    });

    tokio::task::spawn(async move {
        match subscriber_callback(subscriber, cmd_state_clone_4).await {
            Ok(()) => (),
            Err(e) => r2r::log_error!("robotiq_2f_driver", "Publisher failed with {}.", e),
        };
    });

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.join().unwrap();

    Ok(())
}

async fn modbus_client(
    tty_path: &str,
    msr: Arc<Mutex<MeasuredState>>,
    cmd: Arc<Mutex<CommandState>>,
) -> () {
    //Option<tokio_modbus::client::Context> {
    let builder = tokio_serial::new(tty_path, 115200);
    let port = SerialStream::open(&builder);
    match port {
        Ok(stream) => match rtu::connect_slave(stream, Slave(0x0009)).await {
            Ok(mut ctx) => {
                loop {
                    read_state(&mut ctx, &msr).await;
                    // println!("State to be written: {:?}", cmd);
                    write_state(&mut ctx, &cmd).await;
                }
            }
            Err(e) => {
                r2r::log_error!(
                    "robotiq_2f_driver",
                    "RTU connecting to slave failed with {}.",
                    e
                );
                ()
            }
        },
        Err(e) => {
            r2r::log_error!(
                "robotiq_2f_driver",
                "Serial port builder failed with {}.",
                e
            );
            ()
        }
    }
}

async fn read_state(ctx: &mut Context, msr: &Arc<Mutex<MeasuredState>>) -> () {
    match ctx.read_holding_registers(0x07D0, 3).await {
        Ok(response) => {
            let mut state = vec![];
            response.iter().for_each(|x| {
                state.push((x & 0xFF00).rotate_right(8));
                state.push(x & 0x00FF)
            });
            *msr.lock().unwrap() = MeasuredState {
                g_act: (state[0] as u8 >> 0) & 0x01,
                g_gto: (state[0] as u8 >> 3) & 0x01,
                g_sta: (state[0] as u8 >> 4) & 0x03,
                g_obj: (state[0] as u8 >> 6) & 0x03,
                g_flt: state[2] as u8,
                g_pr: state[3] as u8,
                g_po: state[4] as u8,
                g_cu: state[5] as u8,
            };
        }
        Err(e) => {
            r2r::log_warn!(
                "robotiq_2f_driver",
                "Reading the holding registers failed with {}.",
                e
            );
        }
    };
}

async fn build_measured_msg(msr: &Arc<Mutex<MeasuredState>>) -> MeasuredMsg {
    let state = *msr.lock().unwrap();
    let measured = match state.g_gto {
        0 => "unknown",
        1 => match state.g_flt {
            0 => match state.g_obj {
                0 | 3 => match state.g_po {
                    x if x < 10 => "opened",
                    x if x > 220 => "closed",
                    _ => "unknown",
                },
                1 | 2 => "gripping",
                _ => "unknown",
            },
            _ => "fault",
        },
        _ => "unknown",
    };

    MeasuredMsg {
        measured: measured.to_string(),
        g_act: state.g_act,
        g_gto: state.g_gto,
        g_sta: state.g_sta,
        g_obj: state.g_obj,
        g_flt: state.g_flt,
        g_pr: state.g_pr,
        g_po: state.g_po,
        g_cu: state.g_cu,
    }
}

async fn normalize_command_state(cmd: &Arc<Mutex<CommandState>>) -> CommandState {
    let command = *cmd.lock().unwrap();
    CommandState {
        r_act: min(max(command.r_act, 0), 1),
        r_gto: min(max(command.r_gto, 0), 1),
        r_atr: min(max(command.r_atr, 0), 1),
        r_ard: min(max(command.r_ard, 0), 1),
        r_pr: min(max(command.r_pr, 0), 255),
        r_sp: min(max(command.r_sp, 0), 255),
        r_fr: min(max(command.r_fr, 0), 255),
    }
}

async fn write_state(ctx: &mut Context, cmd: &Arc<Mutex<CommandState>>) -> () {
    match build_command_state(&cmd).await {
        Some(state) => match ctx.write_multiple_registers(0x03E8, &state).await {
            Ok(()) => {
                // r2r::log_info!("robotiq_2f_driver", "Command state written: {:?}.", command);
                ()
            }
            Err(e) => {
                r2r::log_warn!(
                    "robotiq_2f_driver",
                    "Writing multiple registers failed with {}.",
                    e
                );
                ()
            }
        },
        None => {
            r2r::log_warn!("robotiq_2f_driver", "Building command state failed.");
            ()
        }
    }
}

async fn command_msg_to_state(msg: CommandMsg, state: &Arc<Mutex<CommandState>>) -> () {
    let r_pr = match msg.use_high_level {
        true => match msg.command.as_str() {
            "open" => 0,
            "close" => 255,
            _ => {
                r2r::log_warn!("robotiq_2f_driver", "Command: {} is invalid.", msg.command);
                0
            }
        },
        false => 0,
    };

    *state.lock().unwrap() = CommandState {
        r_act: if msg.use_high_level { 1 } else { msg.r_act },
        r_gto: if msg.use_high_level { 1 } else { 1 }, //{ msg.r_gto },
        r_atr: if msg.use_high_level { 0 } else { msg.r_atr },
        r_ard: if msg.use_high_level { 0 } else { msg.r_ard },
        r_pr: if msg.use_high_level { r_pr } else { msg.r_pr },
        r_sp: if msg.use_high_level { 255 } else { msg.r_sp },
        r_fr: if msg.use_high_level { 150 } else { msg.r_fr },
    }
}

async fn build_command_state(cmd: &Arc<Mutex<CommandState>>) -> Option<[u16; 3]> {
    let norm = normalize_command_state(&cmd).await;
    let bytes_msg = match norm.r_act.checked_add(norm.r_gto << 3) {
        Some(pre_x) => match pre_x.checked_add(norm.r_atr << 4) {
            Some(x) => Some([x, 0, 0, norm.r_pr, norm.r_sp, norm.r_fr]),
            None => {
                r2r::log_warn!(
                    "robotiq_2f_driver",
                    "Checked integer addition failed, overflow occurred."
                );
                None
            }
        },
        None => {
            r2r::log_warn!(
                "robotiq_2f_driver",
                "Checked integer addition failed, overflow occurred."
            );
            None
        }
    };

    match bytes_msg {
        Some([a, b, c, d, e, f]) => match (a as u16).rotate_right(8).checked_add(b as u16) {
            Some(reg_1) => match (c as u16).rotate_right(8).checked_add(d as u16) {
                Some(reg_2) => match (e as u16).rotate_right(8).checked_add(f as u16) {
                    Some(reg_3) => {
                        // println!("witing command state {:?}", *cmd.lock().unwrap());
                        Some([reg_1, reg_2, reg_3])
                    }
                    None => {
                        r2r::log_warn!(
                            "robotiq_2f_driver",
                            "Checked integer addition failed, overflow occurred."
                        );
                        None
                    }
                },
                None => {
                    r2r::log_warn!(
                        "robotiq_2f_driver",
                        "Checked integer addition failed, overflow occurred."
                    );
                    None
                }
            },
            None => {
                r2r::log_warn!(
                    "robotiq_2f_driver",
                    "Checked integer addition failed, overflow occurred."
                );
                None
            }
        },
        None => {
            r2r::log_warn!(
                "robotiq_2f_driver",
                "Checked integer addition failed, overflow occurred."
            );
            None
        }
    }
}

async fn reset_on_startup(cmd: Arc<Mutex<CommandState>>) -> () {
    let reset_state = CommandState {
        r_act: 0,
        ..Default::default()
    };

    *cmd.lock().unwrap() = reset_state;
    thread::sleep(time::Duration::from_millis(500));
}

async fn activate_on_startup(cmd: Arc<Mutex<CommandState>>) -> () {
    let active_state = CommandState {
        r_act: 1,
        r_gto: 1,
        r_sp: 255,
        r_fr: 150,
        ..Default::default()
    };

    *cmd.lock().unwrap() = active_state;

    thread::sleep(time::Duration::from_millis(500));
}

async fn publisher_callback(
    publisher: r2r::Publisher<MeasuredMsg>,
    mut timer: r2r::Timer,
    msr: Arc<Mutex<MeasuredState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let msg = build_measured_msg(&msr).await;
        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "robotiq_2f_driver",
                    "Publisher failed to send a message with: {}",
                    e
                );
            }
        };
        timer.tick().await?;
        // timer.tick().await?;
    }
}

async fn subscriber_callback(
    mut subscriber: impl Stream<Item = CommandMsg> + Unpin,
    cmd: Arc<Mutex<CommandState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(msg) => {
                command_msg_to_state(msg, &cmd).await;
            }
            None => {
                r2r::log_error!("robotiq_2f_driver", "Subscriber did not get the message?",);
                ()
            }
        }
    }
}
