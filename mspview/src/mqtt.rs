//extern crate json;

use paho_mqtt::MQTT_VERSION_5;

use crate::App;

/// Defines the values of a sensor tile at a given time.
#[derive(Debug, Clone)]
pub struct SensorState {
    pub id: u8,
    pub pressure: Vec<f64>,
    pub presence: Vec<f32>
}

impl SensorState {

    /// Parses a [`String`] received through MQTT to populate and return an instance of
    /// [`SensorState`].
    pub fn from(s: String) -> json::Result<Self> {
        match json::parse(s.as_str()) {
            Ok(data) => {
                Ok(Self {
                    id: data["id"].as_u8().unwrap_or(0),
                    pressure: data["pressure"].members().map(|val| val.as_f64().unwrap_or(0.0)).collect(),
                    presence: data["presence"].members().map(|val| val.as_f32().unwrap_or(0.0)).collect()
                })},
            Err(err) => Err(err)
        }
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
