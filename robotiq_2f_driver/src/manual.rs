use r2r::robotiq_2f_msgs::msg::CommandState;
// use r2r::robotiq_2f_msgs::msg::MeasuredState;
use fltk::{app, button::Button, frame::Frame, prelude::*, window::Window};
use r2r::QosProfile;
use std::sync::{Arc, Mutex};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "robotiq_2f_manual_control", "")?;

    let publisher = Arc::new(Mutex::new(node.create_publisher::<CommandState>(
        "/robotiq_2f_command",
        QosProfile::default(),
    )?));

    let publisher_clone_1 = publisher.clone();
    let publisher_clone_2 = publisher.clone();

    let app = app::App::default();
    let mut window = Window::default().with_size(400, 300);
    // let mut frame = Frame::default().with_size(200, 100).center_of(&window);
    let mut open_button = Button::new(160, 110, 80, 40, "open");
    let mut close_button = Button::new(160, 210, 80, 40, "close");
    window.end();
    window.show();

    let open_msg = CommandState {
        use_high_level: true,
        command: "open".to_string(),
        ..Default::default()
    };

    let close_msg = CommandState {
        use_high_level: true,
        command: "close".to_string(),
        ..Default::default()
    };

    open_button.set_callback(
        move |_| match publisher_clone_1.lock().unwrap().publish(&open_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "robotiq_2f_driver",
                    "Publisher failed to send a message with: {}",
                    e
                );
            }
        },
    );

    close_button.set_callback(move |_| {
        match publisher_clone_2.lock().unwrap().publish(&close_msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "robotiq_2f_driver",
                    "Publisher failed to send a message with: {}",
                    e
                );
            }
        }
    });

    app.run().unwrap();

    Ok(())
}
