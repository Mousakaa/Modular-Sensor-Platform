use paho_mqtt::MQTT_VERSION_5;
use serde::{Serialize, Deserialize};
use serde_json;
use chrono;

use crate::App;

/// Defines the values of a sensor tile at a given time.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct SensorState {
    pub id: u8,
    pub timestamp: chrono::DateTime<chrono::Local>,
    pub received: Option<chrono::DateTime<chrono::Local>>,
    pub pressure: Vec<f32>,
    pub presence: Vec<i16>
}

impl SensorState {

    /// Parses a [`String`] received through MQTT to populate and return an instance of
    /// [`SensorState`].
    pub fn from(s: String) -> serde_json::Result<Self> {
        let mut state: SensorState = serde_json::from_str(s.as_str())?;
        state.received = Some(chrono::Local::now());
        Ok(state)
    }
}

impl App {
    /// Connect to the MQTT server `server` at `topic`.
    pub fn connect(&mut self) -> paho_mqtt::Result<()> {
        let mqtt_options = paho_mqtt::CreateOptionsBuilder::new()
            .server_uri(self.server.clone())
            .client_id("modular_sensor_platform")
            .finalize();

        let client = paho_mqtt::Client::new(mqtt_options)?;

        let last_will = paho_mqtt::Message::new(
            "nimbus/log",
            "Async subscriber lost connection",
            paho_mqtt::QOS_1
        );

        let connect_options = paho_mqtt::ConnectOptionsBuilder::with_mqtt_version(MQTT_VERSION_5)
            .clean_start(false)
            .properties(paho_mqtt::properties![paho_mqtt::PropertyCode::SessionExpiryInterval => 3600])
            .will_message(last_will)
            .finalize();

        println!("Connecting to the MQTT server...");

        if let Some(conn_rsp) = client.connect(connect_options)?.connect_response() {
            if !conn_rsp.session_present {
                let subscribe_options = paho_mqtt::SubscribeOptions::with_retain_as_published();
                client.subscribe_with_options(self.topic.clone(), 1, subscribe_options, None)?;
            }
        }

        self.notifications = Some(client.start_consuming());
        self.mqtt_client = client;

        println!("Waiting for messages...");

        Ok(())
    }

    /// Non-blocking function that polls the MQTT connection to receive messages.
    pub fn recv(&mut self) -> bool {
        for _ in 0..10 { // Up to 10 messages processed on a single frame
            match self.notifications.clone().unwrap().try_recv() {
                Ok(opt) => {
                    if let Some(msg) = opt {
                        if let Ok(text) = String::from_utf8(msg.payload().to_vec()) {
                            if let Ok(state) = SensorState::from(text) {
                                self.msg_queue.push_back(state.clone());
                                self.process(state);
                            }
                        }
                    }
                },
                Err(err) => {
                    if err.is_disconnected() {
                        return false;
                    }
                    //else if err.is_empty() {
                    //    return true;
                    //}
                }
            }
        }
        true
    }
}
