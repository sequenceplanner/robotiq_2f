use r2r::geometry_msgs::msg::{Transform, TransformStamped};
use r2r::std_msgs::msg::Header;
use r2r::ParameterValue;
use r2r::QosProfile;
use r2r::tf2_msgs::msg::TFMessage;
use serde::{Deserialize, Serialize};
use std::fs::{self, File};
use std::io::BufReader;
use std::sync::{Arc, Mutex};

pub static NODE_ID: &'static str = "robotiq_2f_tfbc";
pub static BROADCAST_RATE: u64 = 1000;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct FrameData {
    pub parent_frame_id: String,
    pub child_frame_id: String,
    pub transform: Transform,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // setup the node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, NODE_ID, "")?;

    // handle parameters passed on from the launch files
    let params = node.params.clone();
    let params_things = params.lock().unwrap(); // OK to panic
    let scenario_path = params_things.get("scenario_path");

    let mut scenario = vec![];
    match scenario_path {
        Some(p) => match p {
            ParameterValue::String(dir_res) => match fs::read_dir(dir_res) {
                Ok(dir) => dir.for_each(|file| match file {
                    Ok(entry) => match entry.path().to_str() {
                        Some(valid) => scenario.push(valid.to_string()),
                        None => r2r::log_warn!(NODE_ID, "Path is not valid unicode."),
                    },
                    Err(e) => r2r::log_warn!(NODE_ID, "Reading entry failed with '{}'.", e),
                }),
                Err(e) => {
                    r2r::log_warn!(
                        NODE_ID,
                        "Reading the scenario directory failed with: '{}'. Empty scenario will be launched.",
                        e
                    );
                }
            },
            _ => {
                r2r::log_warn!(
                    NODE_ID,
                    "Parameter 'scenario_path' has to be of type String. Empty scenario will be launched."
                );
            }
        },
        None => {
            r2r::log_warn!(
                NODE_ID,
                "Parameter 'scenario_path' not specified. Empty scenario will be launched."
            );
        }
    };

    println!("{:?}", scenario);

    let mut scenario_frames_init = vec![];
    scenario
        .iter()
        .for_each(|x| match File::open(x) {
            Ok(file) => {
                let reader = BufReader::new(file);
                match serde_json::from_reader(reader) {
                    Ok(json) => scenario_frames_init.push(json),
                    Err(e) => r2r::log_warn!(NODE_ID, "Serde failed with: '{}'.", e)
                }
            },
            Err(e) => r2r::log_warn!(NODE_ID, "Opening json file failed with: '{}'.", e)
        });

    let scenario_frames = Arc::new(Mutex::new(scenario_frames_init));

    // publish the joint state of the robot at the simulation rate
    let pub_timer = node.create_wall_timer(std::time::Duration::from_millis(BROADCAST_RATE))?;
    let static_frame_broadcaster =
        node.create_publisher::<TFMessage>("/tf_static", QosProfile::transient_local(QosProfile::default()))?;

    // spawn a tokio task to handle publishing static frames
    let scenario_frames_clone_1 = scenario_frames.clone();
    tokio::task::spawn(async move {
        match static_frame_broadcaster_callback(
            static_frame_broadcaster,
            pub_timer,
            &scenario_frames_clone_1,
        )
        .await
        {
            Ok(()) => (),
            Err(e) => r2r::log_error!(NODE_ID, "Joint state publisher failed with: '{}'.", e),
        };
    });

    // keep the node alive
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(1000));
    });

    r2r::log_info!(NODE_ID, "Node started.");

    handle.join().unwrap();

    Ok(())
}

// broadcast static frames
async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    transforms: &Arc<Mutex<Vec<FrameData>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = transforms.lock().unwrap().clone();
        let mut updated_transforms = vec!();

        transforms_local.iter().for_each(|t| {
            updated_transforms.push(TransformStamped {
                header: Header {
                    stamp: time_stamp.clone(),
                    frame_id: t.parent_frame_id.clone(),
                },
                child_frame_id: t.child_frame_id.clone(),
                transform: t.transform.clone()
            });
        });

        let msg = TFMessage {
            transforms: updated_transforms
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(NODE_ID, "Publisher failed to send a message with: '{}'", e);
            }
        };
        timer.tick().await?;
    }
}