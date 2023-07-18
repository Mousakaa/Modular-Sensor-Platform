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

use std::collections::VecDeque;
use std::process;
use eframe::egui;
use paho_mqtt::{self as mqtt, MQTT_VERSION_5, AsyncReceiver, Message};
use futures::executor::block_on;
use image::EncodableLayout;
use colours;

fn main() {
    let mqtt_options = mqtt::CreateOptionsBuilder::new()
        .server_uri("mqtt.eclipseprojects.io")
        .client_id("modular_sensor_platform")
        .finalize();

    let mut client = mqtt::AsyncClient::new(mqtt_options).unwrap_or_else(|e| {
        println!("Error creating the client : {e:?}");
        process::exit(1);
    });

    if let Err(error) = block_on(async {
        let stream = client.get_stream(25);

        let last_will = mqtt::Message::new("test", "Async subscriber lost connection", mqtt::QOS_1);

        let connect_options = mqtt::ConnectOptionsBuilder::with_mqtt_version(MQTT_VERSION_5)
            .clean_start(false)
            .properties(mqtt::properties![mqtt::PropertyCode::SessionExpiryInterval => 3600])
            .will_message(last_will)
            .finalize();

        println!("Connecting to the MQTT server...");
        client.connect(connect_options).await?;

        let subscribe_options = mqtt::SubscribeOptions::with_retain_as_published();
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

        Ok::<(), mqtt::Error>(())

    }) {
        println!("{:?}", error);
    }
}

fn spectrum(value: f32) -> image::Rgb<u8> {
    let rgba = colours::Rgba::from(colours::Hsva::new(
        value / 256.0,
        1.0,
        1.0,
        1.0
    ));
    image::Rgb([(rgba.red * 255.0) as u8, (rgba.green * 255.0) as u8, (rgba.blue * 255.0) as u8])
}

fn value_from(color: egui::Color32) -> f32 {
    let rgba: colours::Rgba<f32> = colours::Rgba::new(color.r(), color.g(), color.b(), color.a()).into();
    let hsva: colours::Hsva<f32> = rgba.into();
    return hsva.hue * 255.0;
}

/// Defines the values of a sensor tile at a given time.
#[derive(Debug, Clone)]
struct SensorState {
    id: u8,
    pressure: Vec<f64>,
    presence: Vec<f32>
}

impl SensorState {

    /// Parses a [`String`] received through MQTT to populate and return an instance of
    /// [`SensorState`].
    fn from(s: String) -> json::Result<Self> {
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

/// Defines the different display modes that the user can navigate.
enum Display {
    /// Displays the raw data.
    MESSAGES,
    /// Visually shows the weight distribution on the array.
    PRESSURE,
    /// Visually shows presence detection on the array.
    PRESENCE,
    /// Mixed view, where the weight is only shown where presence is above a certain threshold.
    BOTH,
    /// Displays a settings screen.
    SETTINGS
}

/// Main structure of the UI.
struct App {
    /// MQTT message stream.
    notifications: AsyncReceiver<Option<Message>>,
    /// Queue of the received messages, filled from front to back.
    msg_queue: VecDeque<SensorState>,
    /// Current display mode.
    current_display: Display,
    /// A [`Vec`] containing the number of rows and columns of capacitive presence sensors on each sensor
    /// tile.
    presence_res: Vec<usize>,
    /// A [`Vec`] containing the number of rows and columns of capacitive pressure sensors on each sensor
    /// tile.
    pressure_res:Vec<usize>,
    /// Number of sensors on each tile.
    nb_sensors: usize,
    /// Number of tiles in each column of the array.
    lins: usize,
    /// Number of tiles in each line of the array.
    cols: usize,
    /// Image showing the current presence state of the array.
    presence_img: image::RgbImage,
    /// Image showing the current pressure state of the array.
    pressure_img: image::RgbImage,
    /// Type of interpolation algorithm used.
    interpolation: image::imageops::FilterType,
    /// Interpolation resolution increase ratio
    inter_size: u32,
    /// Value of the threshold to detect presence
    threshold: f32
}

impl App {
    fn new(notif: AsyncReceiver<Option<Message>>) -> Self {
        Self {
            notifications: notif,
            msg_queue: VecDeque::new(),
            current_display: Display::MESSAGES,
            presence_res: vec![8, 8],
            pressure_res: vec![2, 2],
            nb_sensors: 4,
            lins: 2,
            cols: 2,
            presence_img: image::RgbImage::new(16, 16),
            pressure_img: image::RgbImage::new(4, 4),
            interpolation: image::imageops::Nearest,
            inter_size: 160,
            threshold: 128.0
        }
    }

    /// Non-blocking function that polls the MQTT connection to receive messages.
    fn recv(&mut self) {
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

    /// Processes a [`SensorState`] to incorporate it in the presence and pressure images.
    fn process(&mut self, state: SensorState) {
        let x = (state.id as usize % self.cols) as u32 ;
        let y = (state.id as usize / self.cols) as u32;
        for s_x in 0..self.pressure_res[0] as u32 {
            for s_y in 0..self.pressure_res[1] as u32 {

                let color = if self.pressure_res[0]*self.pressure_res[1] == state.pressure.len() {
                    let value = state.pressure[s_x as usize + s_y as usize * self.pressure_res[0] ];
                    spectrum(value as f32)
                }
                else {
                    image::Rgb([0; 3])
                };

                if self.pressure_img.width() == (self.cols*self.pressure_res[0]) as u32
                && self.pressure_img.height() == (self.lins*self.pressure_res[1]) as u32{
                    self.pressure_img.put_pixel(x*self.pressure_res[0] as u32 + s_x, y*self.pressure_res[1] as u32 + s_y, color);
                }
                else {
                    println!("Error in pressure image size");
                }
            }
        }
        for s_x in 0..self.presence_res[0] as u32 {
            for s_y in 0..self.presence_res[1] as u32 {

                let color = if self.presence_res[0]*self.presence_res[1] == state.presence.len() {
                    let value = state.presence[s_x as usize + s_y as usize * self.presence_res[0] ];
                    spectrum(value)
                }
                else {
                    image::Rgb([0; 3])
                };

                if self.presence_img.width() == (self.cols*self.presence_res[0]) as u32
                && self.presence_img.height() == (self.lins*self.presence_res[1]) as u32{
                    self.presence_img.put_pixel(x*self.presence_res[0] as u32 + s_x, y*self.presence_res[1] as u32 + s_y, color);
                }
                else {
                    println!("Error in presence image size");
                }
            }
        }
    }

    /// Interpolates the presence image using the `interpolation` value to bring it to screen size.
    fn interpolated_presence(&mut self) -> egui::ColorImage {
        let width = self.inter_size;
        let height = self.inter_size * self.presence_img.height() / self.presence_img.width();

        let interpolated = image::imageops::resize(
            &self.presence_img,
            width,
            height,
            self.interpolation
        );

        return egui::ColorImage::from_rgb(
            [width as usize, height as usize],
            interpolated.as_bytes()
        );
    }

    /// Interpolates the pressure image using the `interpolation` value to bring it to screen size.
    fn interpolated_pressure(&mut self) -> egui::ColorImage {
        let width = self.inter_size;
        let height = self.inter_size * self.pressure_img.height() / self.pressure_img.width();

        let interpolated = image::imageops::resize(
            &self.pressure_img,
            width,
            height,
            self.interpolation
        );

        return egui::ColorImage::from_rgb(
            [width as usize, height as usize],
            interpolated.as_bytes()
        );
    }

    /// UI for the Presence screen
    fn presence_screen(&mut self, ui: &mut egui::Ui) {
        let ratio = self.presence_img.width() as f32 / self.presence_img.height() as f32;
        let height = if ratio < ui.available_width() / ui.available_height() {
            ui.available_height()
        }
        else {
            ui.available_width() / ratio
        };


        let texture = ui.ctx().load_texture(
            "presence_img",
            self.interpolated_presence(),
            egui::TextureOptions::NEAREST
        );

        ui.centered_and_justified(|ui| {
            ui.image(&texture, egui::vec2(height*ratio, height));
        });
    }

    /// UI for the Pressure screen
    fn pressure_screen(&mut self, ui: &mut egui::Ui) {
        let ratio = self.pressure_img.width() as f32 / self.pressure_img.height() as f32;
        let height = if ratio < ui.available_width() / ui.available_height() {
            ui.available_height()
        }
        else {
            ui.available_width() / ratio
        };

        let texture = ui.ctx().load_texture(
            "pressure_img",
            self.interpolated_pressure(),
            egui::TextureOptions::NEAREST
        );

        ui.centered_and_justified(|ui| {
            ui.image(&texture, egui::vec2(height*ratio, height));
        });
    }

    /// UI for the Cross View screen
    fn crossview_screen(&mut self, ui: &mut egui::Ui) {
        let ratio = self.presence_img.width() as f32 / self.presence_img.height() as f32;
        let height = if ratio < ui.available_width() / ui.available_height() {
            ui.available_height()
        }
        else {
            ui.available_width() / ratio
        };

        let pressure = self.interpolated_pressure();
        let mut crossview = self.interpolated_presence();

        for i in 0..crossview.pixels.len() {
            crossview.pixels[i] = if value_from(crossview.pixels[i]) >= self.threshold {
                pressure.pixels[i]
            }
            else {
                egui::Color32::BLACK
            };
        }

        let texture = ui.ctx().load_texture(
            "crossview_img",
            crossview,
            egui::TextureOptions::NEAREST
        );

        ui.centered_and_justified(|ui| {
            ui.image(&texture, egui::vec2(height*ratio, height));
        });
    }

    /// UI for the Settings screen
    fn settings_screen(&mut self, ui: &mut egui::Ui) {
        ui.add_space(5.0);
        ui.strong("Interpolation settings");

        ui.add(egui::Slider::new(&mut self.inter_size, 1..=1000).text("Width of the interpolated images (in pixels)"));
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.interpolation, image::imageops::Nearest, "None");
            ui.radio_value(&mut self.interpolation, image::imageops::Triangle, "Bilinear");
            ui.radio_value(&mut self.interpolation, image::imageops::CatmullRom, "Catmull-Rom (Bicubic)");
            ui.radio_value(&mut self.interpolation, image::imageops::Gaussian, "Gaussian");
            ui.radio_value(&mut self.interpolation, image::imageops::Lanczos3, "Lanczos 3");
        });
        ui.add(egui::Slider::new(&mut self.threshold, std::ops::RangeInclusive::new(0.0, 256.0)).text("Presence detection threshold"));

        ui.add_space(5.0);
        ui.separator();
        ui.add_space(5.0);
        ui.strong("Sensor resolution settings");

        ui.horizontal(|ui| {
            ui.label("Presence resolution : ");

            if ui.add(egui::Slider::new(&mut self.presence_res[0], 1..=8).text("X")).changed()
            || ui.add(egui::Slider::new(&mut self.presence_res[1], 1..=8).text("Y")).changed() {
                self.presence_img = image::RgbImage::new(
                    (self.cols*self.presence_res[0]) as u32,
                    (self.lins*self.presence_res[1]) as u32
                );
            }
        });

        ui.horizontal(|ui| {
            ui.label("Pressure resolution : ");

            if ui.add(egui::Slider::new(&mut self.pressure_res[0], 1..=8).text("X")).changed()
            || ui.add(egui::Slider::new(&mut self.pressure_res[1], 1..=8).text("Y")).changed() {
                self.pressure_img = image::RgbImage::new(
                    (self.cols*self.pressure_res[0]) as u32,
                    (self.lins*self.pressure_res[1]) as u32
                );
            }
        });

        ui.add_space(5.0);
        ui.separator();
        ui.add_space(5.0);
        ui.strong("Sensor layout settings");

        if ui.add(egui::Slider::new(&mut self.nb_sensors, 1..=64).text("Number of sensor tiles")).changed()
        || ui.add(egui::Slider::new(&mut self.cols, 1..=self.nb_sensors).text("Columns")).changed() {
            self.lins = (self.nb_sensors as f32 / self.cols as f32).ceil() as usize;
            self.presence_img = image::RgbImage::new(
                (self.cols*self.presence_res[0]) as u32,
                (self.lins*self.presence_res[1]) as u32
            );
            self.pressure_img = image::RgbImage::new(
                (self.cols*self.pressure_res[0]) as u32,
                (self.lins*self.pressure_res[1]) as u32
            );
    }
        ui.add_space(10.0);

        let s_width = f32::min(
            (ui.available_width() - 100.0) / self.cols as f32,
            (ui.available_height() - 50.0) / self.lins as f32
        );

        ui.set_width(self.cols as f32 * s_width);

        ui.vertical_centered(|ui| {

            ui.spacing_mut().item_spacing = egui::Vec2::new(0.0, 0.0);
            ui.set_width(self.cols as f32 * s_width);

            for lin in 0..self.lins {
                ui.horizontal(|ui| {
                    ui.set_width(self.cols as f32 * s_width);
                    for col in 0..self.cols {
                        if (col + lin*self.cols) < self.nb_sensors {
                            egui::Frame::none()
                                .outer_margin(egui::Margin::same(s_width * 0.05))
                                .fill(ui.visuals().weak_text_color())
                                .stroke(egui::Stroke::new(1.0, ui.visuals().text_color()))
                                .rounding(egui::Rounding::same(s_width * 0.05))
                                .show(ui, |ui| {
                                    ui.set_height(s_width * 0.95);
                                    ui.set_width(s_width * 0.95);
                                    ui.centered_and_justified(|ui| {
                                        ui.label(egui::RichText::new((col + lin*self.cols).to_string()).size(s_width * 0.5));
                                    });
                                });
                        }
                    }
                });
            }
        });
    }

}

impl eframe::App for App {
   fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {

       self.recv();

       egui::TopBottomPanel::top("top").show(ctx, |ui| {
           ui.vertical_centered(|inner_ui| {
               inner_ui.heading("Modular Sensor Platform");
           });
        });

       egui::CentralPanel::default().show(ctx, |ui| {
           egui::menu::bar(ui, |inner_ui| {
               if inner_ui.button("Messages").clicked() {
                   self.current_display = Display::MESSAGES;
               }
               if inner_ui.button("Pressure").clicked() {
                   self.current_display = Display::PRESSURE;
               }
               if inner_ui.button("Presence").clicked() {
                   self.current_display = Display::PRESENCE;
               }
               if inner_ui.button("Cross View").clicked() {
                   self.current_display = Display::BOTH;
               }
               if inner_ui.button("Settings").clicked() {
                   self.current_display = Display::SETTINGS;
               }
           });
           ui.separator();

           match self.current_display {
               Display::MESSAGES => {
                   ui.add_space(5.0);
                   ui.strong("Message log");
                   ui.add_space(5.0);
                   egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                       for state in self.msg_queue.clone() {
                           ui.label(format!("id: {:?}\npressure: {:?}\npresence: {:?}\n", state.id, state.pressure, state.presence));
                           ui.separator();
                       }
                   });
               },
               Display::PRESSURE => self.pressure_screen(ui),
               Display::PRESENCE => self.presence_screen(ui),
               Display::BOTH => self.crossview_screen(ui),
               Display::SETTINGS => self.settings_screen(ui)
           };
           
           ui.ctx().request_repaint();
       });
   }
}
