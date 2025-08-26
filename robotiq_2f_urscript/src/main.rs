use micro_sp::*;
use serde::{Deserialize, Serialize};
use std::error::Error;

use std::io;
use std::net::TcpStream;
use tokio::time::{Duration, interval};

#[derive(Serialize, Deserialize, Clone)]
pub struct GripperCommand {
    // open, close, move_to, set force, activate, etc...
    pub command_type: String,
    pub velocity: f64,
    pub force: f64,
    pub ref_pos_percentage: i64, // fully closed: 100, fully open 0, or anything inbetween
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    initialize_env_logger();
    let mut interval = interval(Duration::from_millis(250));
    let templates_dir = std::env::var("TEMPLATES_DIR").expect("TEMPLATES_DIR is not set");
    let ur_address = std::env::var("UR_ADDRESS").expect("UR_ADDRESS is not set");
    let ur_port = std::env::var("UR_PORT")
        .expect("UR_PORT is not set")
        .parse()
        .unwrap();
    let gripper_id = std::env::var("GRIPPER_ID").expect("GRIPPER_ID is not set");
    let log_target = format!("{}_rq_2f_script_driver", gripper_id);

    let state = generate_gripper_interface_state(&gripper_id);
    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;

    let templates: tera::Tera = {
        let tera = match tera::Tera::new(&format!("{}/*.script", templates_dir)) {
            Ok(t) => {
                log::warn!(target: &log_target, "Searching for Tera templates, wait...",);
                t
            }
            Err(e) => {
                log::error!(target: &log_target, "UR Script template parsing error(s): {}", e);
                ::std::process::exit(1);
            }
        };
        tera
    };

    let template_names = templates
        .get_template_names()
        .map(|x| x.to_string())
        .collect::<Vec<String>>();
    if template_names.len() == 0 {
        log::error!(target: &log_target, "Couldn't find any Tera templates.");
    } else {
        log::info!(target: &log_target, "Found templates.");
    }

    let keys: Vec<String> = vec![
        format!("{}_request_trigger", gripper_id),
        format!("{}_request_state", gripper_id),
        format!("{}_command_type", gripper_id),
        format!("{}_velocity", gripper_id),
        format!("{}_force", gripper_id),
        format!("{}_ref_pos_percentage", gripper_id),
    ]
    .iter()
    .map(|k| k.to_string())
    .collect();

    let connection_manager = ConnectionManager::new().await;
    let mut con = connection_manager.get_connection().await;
    StateManager::set_state(&mut con, &state).await;
    loop {
        interval.tick().await;
        if let Err(_) = connection_manager.check_redis_health(&log_target).await {
            continue;
        }
        let state = match StateManager::get_state_for_keys(&mut con, &keys).await {
            Some(s) => s,
            None => continue,
        };

        let mut request_trigger = state
            .get_bool_or_default_to_false(&format!("{gripper_id}_request_trigger"), &log_target);

        let mut request_state = state
            .get_string_or_default_to_unknown(&format!("{gripper_id}_request_state"), &log_target);

        if request_trigger {
            request_trigger = false;
            if request_state == ActionRequestState::Initial.to_string() {
                let command_type = state.get_string_or_default_to_unknown(
                    &format!("{gripper_id}_command_type"),
                    &log_target,
                );

                let velocity = state
                    .get_float_or_value(&format!("{gripper_id}_velocity"), 1.0, &log_target);

                let force =
                    state.get_float_or_value(&format!("{gripper_id}_force"), 1.0, &log_target);

                let ref_pos_percentage = state.get_int_or_default_to_zero(
                    &format!("{gripper_id}_ref_pos_percentage"),
                    &log_target,
                );

                let gripper_command = GripperCommand {
                    command_type,
                    velocity,
                    force,
                    ref_pos_percentage,
                };

                let script = match generate_script(gripper_command, &templates, &log_target) {
                    Ok(script) => script,
                    Err(_) => {
                        log::error!(target: &log_target,
                                "Failed to generate UR Script.");
                        continue;
                    }
                };

                match send_gripper_script(&ur_address, ur_port, &script) {
                    Ok(_) => request_state = ActionRequestState::Succeeded.to_string(),
                    Err(_) => request_state = ActionRequestState::Failed.to_string(),
                }
            }
        }

        StateManager::set_sp_value(
            &mut con,
            &format!("{gripper_id}_request_state"),
            &request_state.to_spvalue(),
        )
        .await;
        StateManager::set_sp_value(
            &mut con,
            &format!("{gripper_id}_request_trigger"),
            &request_trigger.to_spvalue(),
        )
        .await;
    }
}

fn generate_script(
    gripper_command: GripperCommand,
    templates: &tera::Tera,
    log_target: &str,
) -> Result<String, Box<dyn std::error::Error>> {
    let empty_context = tera::Context::new();
    match templates.render(
        &format!("{}.script", gripper_command.command_type.to_string()),
        match &tera::Context::from_serialize(gripper_command.clone()) {
            Ok(context) => context,
            Err(e) => {
                log::error!(target: &log_target,
                    "Creating a Tera Context from a serialized interpretation failed with: {e}.");
                log::error!(target: &log_target,
                    "An empty Tera Context will be used instead.");
                &empty_context
            }
        },
    ) {
        Ok(script) => Ok(script),
        Err(e) => {
            log::error!(target: &log_target,
                "Rendering the {}.script Tera Template failed with: {}.",
                gripper_command.command_type,
                e
            );
            return Err(Box::new(e));
        }
    }
}

fn send_gripper_script(host: &str, port: u16, script_content: &str) -> io::Result<u64> {
    let server_address = format!("{}:{}", host, port);
    let mut stream = TcpStream::connect(server_address)?;
    let mut reader = script_content.as_bytes();
    io::copy(&mut reader, &mut stream)
}

pub fn generate_gripper_interface_state(gripper_id: &str) -> State {
    let state = State::new();

    let request_trigger = bv!(&&format!("{}_request_trigger", gripper_id));
    let request_state = v!(&&format!("{}_request_state", gripper_id));
    let command_type = v!(&&format!("{}_command_type", gripper_id));
    let velocity = fv!(&&format!("{}_velocity", gripper_id));
    let force = fv!(&&format!("{}_force", gripper_id));
    let ref_pos_percentage = iv!(&&format!("{}_ref_pos_percentage", gripper_id));

    let state = state.add(assign!(request_trigger, false.to_spvalue()));
    let state = state.add(assign!(request_state, "initial".to_spvalue()));
    let state = state.add(assign!(
        command_type,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ));
    let state = state.add(assign!(velocity, SPValue::Float64(FloatOrUnknown::UNKNOWN)));
    let state = state.add(assign!(force, SPValue::Float64(FloatOrUnknown::UNKNOWN)));
    let state = state.add(assign!(
        ref_pos_percentage,
        SPValue::Int64(IntOrUnknown::UNKNOWN)
    ));

    state
}
