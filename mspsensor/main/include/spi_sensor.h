#include "driver/spi_master.h"

#define X_LEN 8 // Vertical resolution
#define Y_LEN 7 // Horizontal resolution

// AT42QT commands
#define CMD_CALIBRATE_ALL 0xff
#define CMD_ENABLE_WRITE_SETUP 0xfe
// Unused :
// #define CMD_REQUEST_FREQ_HOP 0xfc
// #define CMD_REQUEST_SLEEP 0xfb
// #define CMD_FORCE_RESET 0x40

// Tests if any touch is detected
bool at42qt_has_changed(void);

// Writes `n` bytes of data to the AT42QT2640, starting at the address `addr`
// (which is a 12-byte integer). `n` must be smaller than 512.
void at42qt_spi_write(spi_device_handle_t device, uint16_t addr, uint8_t* data, size_t n);

// Reads `n` bytes of data from the AT42QT2640, starting at the address `addr`
// (which is a 12-byte integer). `n` must be smaller than 512.
// The function return 1 if successful, 0 if the data is corrupted.
uint8_t at42qt_spi_read(spi_device_handle_t device, uint16_t addr, uint8_t* data, size_t n);

// Hard reset of the AT42QT2640
void at42qt_spi_reset(spi_device_handle_t device);

// SPI bus initialization. This function must be called prior to any communication.
void at42qt_spi_init(spi_device_handle_t* device);

// Send a command to the AT42QT2640. Sending a key number starts the calibration process
// for that key.
void at42qt_send_command(spi_device_handle_t device, uint8_t cmd);

// Check relevent registers for various errors. Recalibrates all keys if a calibration error
// is encountered. `prev_errors` points to a counter which must be initialized to 0.
void at42qt_check_errors(spi_device_handle_t device, uint8_t* prev_errors);

// AT42QT2640 setup. Writes in internal EEPROM, so this setup is persistent. Resets the device
// when done.
void at42qt_setup(spi_device_handle_t device);

// Reads the status of each key in the array (0 for untouched and 1 for touched), and stores it
// in `values`.
void at42qt_get_status(spi_device_handle_t device, uint8_t* values);

// Reads the raw measurements of every key (on 13 bits), and stores the signed difference
// between each key's measurement and its reference value into `values`.
void at42qt_get_values(spi_device_handle_t device, int16_t* values);

