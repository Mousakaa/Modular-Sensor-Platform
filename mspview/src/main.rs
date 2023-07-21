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

mod mqtt;
mod gui;

use crate::gui::App;

fn main() {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "MSP Viewer",
        native_options,
        Box::new(|_cc| Box::new(App::new()))
    ).unwrap_or_else(|e| {
        println!("Error while building GUI : {e:?}");
    });
}
