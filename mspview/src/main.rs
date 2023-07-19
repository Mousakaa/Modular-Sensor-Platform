//! # Modular Platform Sensor Data Viewer
//!
//! Intership project at the Nimbus Research Centre, Munster Technological University, Ireland  
//! Author : Arthur Gaudard  
//! Supervisor : Juan F. Martinez
//!
//! This is a graphical user interface made to view data from an array of capacitive sensing tiles
//! that detect both presence and weight. They communicate via Wifi, through the MQTT protocol. The
//! GUI is implemented in Rust using the [`egui`] crate, and the MQTT part is based on the
//! [`paho_mqtt`] crate.

extern crate json;

use std::process;
use paho_mqtt::MQTT_VERSION_5;
use futures::executor::block_on;

mod mqtt;
mod gui;

use crate::gui::App;

fn main() {
    let mqtt_options = paho_mqtt::CreateOptionsBuilder::new()
        .server_uri("mqtt.eclipseprojects.io")
        .client_id("modular_sensor_platform")
        .finalize();

    let mut client = paho_mqtt::AsyncClient::new(mqtt_options).unwrap_or_else(|e| {
        println!("Error creating the client : {e:?}");
        process::exit(1);
    });

    if let Err(error) = block_on(async {
        let stream = client.get_stream(25);

        let last_will = paho_mqtt::Message::new("test", "Async subscriber lost connection", paho_mqtt::QOS_1);

        let connect_options = paho_mqtt::ConnectOptionsBuilder::with_mqtt_version(MQTT_VERSION_5)
            .clean_start(false)
            .properties(paho_mqtt::properties![paho_mqtt::PropertyCode::SessionExpiryInterval => 3600])
            .will_message(last_will)
            .finalize();

        println!("Connecting to the MQTT server...");
        client.connect(connect_options).await?;

        let subscribe_options = paho_mqtt::SubscribeOptions::with_retain_as_published();
        client.subscribe_with_options("nimbus/modular_sensor_platform", 1, subscribe_options, None).await?;

        println!("Waiting for messages...");

        let native_options = eframe::NativeOptions::default();
        eframe::run_native(
            "MSP Viewer",
            native_options,
            Box::new(|_cc| Box::new(App::new(stream)))
        ).unwrap_or_else(|e| {
            println!("Error while building GUI : {e:?}");
        });

        Ok::<(), paho_mqtt::Error>(())

    }) {
        println!("{:?}", error);
    }
}
