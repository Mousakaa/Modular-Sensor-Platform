# MSPView - The GUI for the project

This tool is developed in the Rust programming language.

## MQTT data retrieval

The [paho-mqtt](https://docs.rs/paho-mqtt/latest/paho_mqtt) crate was used to implement the MQTT connection to retrieve data from the sensors. It connects to the `mqtt.eclipseprojects.io` public MQTT server.

## The GUI

All visual aspects are based on the [egui](https://docs.rs/egui/latest/egui) and [eframe](https://docs.rs/eframe/latest/eframe) crates, with the use of [image](https://docs.rs/image/latest/image) for the image interpolation algorithms.
