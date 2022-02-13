use std::cmp::*;
use std::{thread, time};
use tokio_modbus::client::Context;
use tokio_modbus::prelude::*;
use tokio_serial::SerialStream;

#[derive(Debug, Default)]
struct MeasuredState {
    pub valid: bool, // some unwrap failed, disregard state
    pub g_act: u8,   // activation status
    pub g_gto: u8,   // action status
    pub g_sta: u8,   // gripper status
    pub g_obj: u8,   // object detection status
    pub g_flt: u8,   // fault status
    pub g_pr: u8,    // position request echo
    pub g_po: u8,    // position
    pub g_cu: u8,    // current
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

    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    let tty_path = "/dev/ttyUSB0";

    match modbus_client(tty_path).await {
        Some(mut client) => {
            tokio::task::spawn(async move {
                initialize_on_startup(&mut client).await;
                loop {
                    println!("{:?}", read_state(&mut client).await);
                    thread::sleep(time::Duration::from_millis(10));
                    // write_state(&mut client, active_state).await;
                }
            });
        }
        None => {
            r2r::log_error!(
                "robotiq_2f_driver",
                "Gripper couldn't connect to path: {}",
                tty_path,
            );
        }
    }

    handle.join().unwrap();

    Ok(())
}

async fn modbus_client(tty_path: &str) -> Option<tokio_modbus::client::Context> {
    let tty_path = tty_path.to_string();
    let builder = tokio_serial::new(tty_path, 115200);
    let port = SerialStream::open(&builder);
    match port {
        Ok(stream) => match rtu::connect_slave(stream, Slave(0x0009)).await {
            Ok(ctx) => Some(ctx),
            Err(e) => {
                r2r::log_error!(
                    "robotiq_2f_driver",
                    "RTU connecting to slave failed with {}.",
                    e
                );
                None
            }
        },
        Err(e) => {
            r2r::log_error!(
                "robotiq_2f_driver",
                "Serial port builder failed with {}.",
                e
            );
            None
        }
    }
}

async fn read_state(ctx: &mut Context) -> MeasuredState {
    let mut state = vec![];
    let valid = match ctx.read_holding_registers(0x07D0, 3).await {
        Ok(response) => {
            response.iter().for_each(|x| {
                state.push((x & 0xFF00).rotate_right(8));
                state.push(x & 0x00FF)
            });
            true
        }
        Err(e) => {
            r2r::log_warn!(
                "robotiq_2f_driver",
                "Reading the holding registers failed with {}.",
                e
            );
            false
        }
    };

    MeasuredState {
        valid,
        g_act: (state[0] as u8 >> 0) & 0x01,
        g_gto: (state[0] as u8 >> 3) & 0x01,
        g_sta: (state[0] as u8 >> 4) & 0x03,
        g_obj: (state[0] as u8 >> 6) & 0x03,
        g_flt: state[2] as u8,
        g_pr: state[3] as u8,
        g_po: state[4] as u8,
        g_cu: state[5] as u8,
    }
}

async fn normalize_command_state(command: CommandState) -> CommandState {
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

async fn write_state(ctx: &mut Context, command: CommandState) -> () {
    match build_command_state(command).await {
        Some(state) => match ctx.write_multiple_registers(0x03E8, &state).await {
            Ok(()) => (),
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

async fn build_command_state(command: CommandState) -> Option<[u16; 3]> {
    let norm = normalize_command_state(command).await;
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
                    Some(reg_3) => Some([reg_1, reg_2, reg_3]),
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

async fn initialize_on_startup(ctx: &mut Context) -> () {
    let reset_state = CommandState {
        r_act: 0,
        ..Default::default()
    };

    write_state(ctx, reset_state).await;

    let active_state = CommandState {
        r_act: 1,
        r_gto: 1,
        r_sp: 255,
        r_fr: 150,
        ..Default::default()
    };

    write_state(ctx, active_state).await;
}
