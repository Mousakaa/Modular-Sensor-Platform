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
    /// Non-blocking function that polls the MQTT connection to receive messages.
    pub fn recv(&mut self) {
       if let Ok(Some(msg)) = self.notifications.try_recv() {
           //println!("Message received");
           match String::from_utf8(msg.payload().to_vec()) {
               Ok(msg) => match SensorState::from(msg) {
                   Ok(state) => {
                       self.msg_queue.push_back(state.clone());
                       self.process(state);
                   },
                   Err(_) => println!("JSON parsing error")
               },
               Err(_) => println!("UTF8 conversion error")
           };
       }
    }
}
