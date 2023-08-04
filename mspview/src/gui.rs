use eframe::egui;
use std::collections::VecDeque;
use paho_mqtt::{Client, Message, Receiver};
use image::EncodableLayout;
use colours;

use crate::mqtt::SensorState;

/// Defines the different display modes that the user can navigate.
pub enum Display {
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
pub struct App {
    /// MQTT client
    pub mqtt_client: Client,
    /// MQTT message stream.
    pub notifications: Option<Receiver<Option<Message>>>,
    /// Queue of the received messages, filled from front to back.
    pub msg_queue: VecDeque<SensorState>,
    /// MQTT server URI.
    pub server: String,
    /// MQTT topic.
    pub topic: String,
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
    threshold: u8
}

impl App {
    pub fn new() -> Self {
        Self {
            mqtt_client: Client::new(paho_mqtt::CreateOptionsBuilder::new().finalize()).unwrap(),
            notifications: None,
            msg_queue: VecDeque::new(),
            server: String::from("mqtt.eclipseprojects.io"),
            topic: String::from("nimbus/modular_sensor_platform"),
            current_display: Display::MESSAGES,
            presence_res: vec![8, 8],
            pressure_res: vec![2, 2],
            nb_sensors: 1,
            lins: 1,
            cols: 1,
            presence_img: image::RgbImage::new(8, 8),
            pressure_img: image::RgbImage::new(2, 2),
            interpolation: image::imageops::Nearest,
            inter_size: 80,
            threshold: 80
        }
    }

    /// Processes a [`SensorState`] to incorporate it in the presence and pressure images.
    pub fn process(&mut self, state: SensorState) {
        let x = (state.id as usize % self.cols) as u32 ;
        let y = (state.id as usize / self.cols) as u32;
        for s_x in 0..self.pressure_res[0] as u32 {
            for s_y in 0..self.pressure_res[1] as u32 {

                let color = if self.pressure_res[0]*self.pressure_res[1] == state.pressure.len() {
                    let value = state.pressure[s_x as usize + s_y as usize * self.pressure_res[0] ];
                    spectrum(value as u8)
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
                    spectrum((value.abs() / 8) as u8)
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

    ///UI for the reconnecting screen
    fn connect_screen(&mut self, ui: &mut egui::Ui) -> paho_mqtt::Result<()> {

        ui.vertical_centered(|ui| {
            ui.set_width(250.0);
            ui.horizontal(|ui| {
                ui.set_width(250.0);
                ui.label("Server URI : ");
                ui.text_edit_singleline(&mut self.server);
            });
        });

        ui.vertical_centered(|ui| {
            ui.set_width(250.0);
            ui.horizontal(|ui| {
                ui.set_width(250.0);
                ui.label("Topic : ");
                ui.text_edit_singleline(&mut self.topic);
            });
        });

        ui.add_space(20.0);

        if ui.button("Connect").clicked() {
            ui.heading(format!("Connecting to MQTT server at\n'{}'\non topic\n'{}'...", self.server, self.topic));
            ui.ctx().request_repaint();
            self.connect()
        }
        else {
            Ok(())
        }
    }

    /// UI for the message log screen
    fn log_screen(&mut self, ui: &mut egui::Ui) {
        ui.add_space(5.0);
        ui.strong("Message log");
        ui.add_space(5.0);
        egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
            for state in self.msg_queue.clone() {
                ui.label(format!(
                    "Message received from sensor {} :\nSent at {}\nArrived at {}\nContaining :\n\t- {} pressure values\n\t- {} presence values\n",
                    state.id,
                    state.timestamp.time(),
                    match state.received {
                        Some(datetime) => datetime.time().format("%H:%M:%S").to_string(),
                        None => String::from("???")
                    },
                    state.pressure.len(),
                    state.presence.len()
                ));
                ui.separator();
            }
        });
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
                if i < pressure.pixels.len() {
                    pressure.pixels[i]
                }
                else {
                    egui::Color32::WHITE
                }
            }
            else {
                egui::Color32::BLACK
            }
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
        ui.add(egui::Slider::new(&mut self.threshold, 0..=255).text("Presence detection threshold"));

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

       egui::TopBottomPanel::top("top").show(ctx, |ui| {
           ui.vertical_centered(|inner_ui| {
               inner_ui.heading("Modular Sensor Platform");
           });
       });

       if self.notifications.is_some() {
           if !self.mqtt_client.is_connected() {
               egui::TopBottomPanel::bottom("errors").show(ctx, |ui| {
                   ui.add_space(5.0);
                   ui.horizontal(|ui| {
                       ui.label(format!("Disconnected from MQTT server, attempting to reconnect..."));
                       if self.mqtt_client.reconnect().is_ok() {
                           ui.label(" Connection restored.");
                       }
                   });
               });
           }
           else {
               if !self.recv() {
                   egui::TopBottomPanel::bottom("errors").show(ctx, |ui| {
                       ui.add_space(5.0);
                       ui.label(format!("MQTT channel disconnected"));
                   });
               }
           }

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
                   inner_ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                       if ui.button("Disconnect").clicked() {
                           self.notifications = None;
                       }
                   });
               });
               ui.separator();

               match self.current_display {
                   Display::MESSAGES => self.log_screen(ui),
                   Display::PRESSURE => self.pressure_screen(ui),
                   Display::PRESENCE => self.presence_screen(ui),
                   Display::BOTH => self.crossview_screen(ui),
                   Display::SETTINGS => self.settings_screen(ui)
               };
               ui.ctx().request_repaint();
           });
       }
       else {
           let mut conn_res = Ok(());
           egui::CentralPanel::default().show(ctx, |ui| {
               ui.add_space(100.0);
               ui.with_layout(egui::Layout::top_down(egui::Align::Center), |ui| {
                   conn_res = self.connect_screen(ui);
               });
               ui.ctx().request_repaint();
           });
           egui::TopBottomPanel::bottom("errors").show(ctx, |ui| {
               if let Err(conn_err) = conn_res {
                   ui.label(format!("Connection error : {conn_err:?}"));
               }
           });
       }
   }
}

/// Maps a `float` value to a RGB color on the spectrum, using the HSV colorspace.
pub fn spectrum(value: u8) -> image::Rgb<u8> {
    let rgba = colours::Rgba::from(colours::Hsva::new(
        value as f32 / 256.0,
        1.0,
        1.0,
        1.0
    ));
    image::Rgb([(rgba.red * 255.0) as u8, (rgba.green * 255.0) as u8, (rgba.blue * 255.0) as u8])
}


/// Reverts back from the color to the value it codes for.
pub fn value_from(color: egui::Color32) -> u8 {
    let rgba: colours::Rgba<f32> = colours::Rgba::new(color.r(), color.g(), color.b(), color.a()).into();
    let hsva: colours::Hsva<f32> = rgba.into();
    return (hsva.hue * 255.0) as u8;
}
