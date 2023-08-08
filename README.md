# Modular Sensor Platform

Project conducted at Nimbus Research Centre by Arthur Gaudard and supervised by Juan F. Martinez.

## Purpose

This 15-week research and development project aims at developing an intelligent flooring solution capable of tracking presence and monitoring activity. The project aims to develop a state-of-the-art modular mat with a multilayer material stack that can accurately detect and recognize one or more pressure points, with sufficient precision as to determine the shape and possibly nature of what is interacting with it. The mat will capture data using capacitive sensor technology and transmit it to a server for analysis. Scalability is a key consideration, allowing for replication and deployment of the module in an array configuration.

## Objective

The ultimate goal is to develop a functional prototype capable of detecting and differentiating between the presence, location, and weight of humans and/or objects, and of extracting from that data as much contextual information as possible, including displacement and orientation of individuals, detection of falls, etc...  
Ideally, the prototype would be comprised of several tiles interacting to form the sensor itself, to allow further scalability.


## Dependencies

### Cargo

[The UI](mspview) being written in Rust and relying on a number of libraries, compiling it requires `cargo`, the Rust package manager and compiler. To install it on Linux or MacOS, run the following command :

```sh
curl https://sh.rustup.rs -sSf | sh
```

### ESP-IDF

The ESP32-C3 based board used for this project [was programmed](mspsensor) using the ESP-IDF framework, and thus it is needed to compile the code and flash the microcontroller. To install it, follow the instructions on [the ESP-IDF Github page](https://github.com/espressif/esp-idf). You will need to know the path to your ESP-IDF folder in order to flash the code.

## Usage

Once you have cloned this repository and `cd`'ed inside it, run :

```sh
make build-ui
```

This will build the UI and copy it in you `$PATH`. You should then be able to start it by typing `mspview` in the command line.

To configure, compile and flash the code to the ESP32-C3, first plug the board to one of the USB ports of your computer. Then, run the following (replacing `path/to/esp-idf` by the path on your machine):

```sh
make flash esp_idf_path=path/to/esp-idf
```

This should open the ESP-IDF configuration manager, which allows you to change the settings of the application before compiling.

The important settings are located in the `MSP Sensor Configuration` and `MSP Network Configuration` menus.

* In `MSP Sensor Configuration`, you can set the vertical and horizontal resolutions of the presence sensor array.

* In `MSP Network Configuration`, you can setup the wifi router info and the size of your wifi mesh, which depends on the number of devices you have.

When you are done configuring, hit `q` to quit the manager and `y` to save the changes. It should then automatically build and flash the binary on the board.
