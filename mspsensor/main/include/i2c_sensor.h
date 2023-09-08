#include "driver/i2c.h"

// I2C initialization for FDC1004, needs to be called prior to any measurement
void i2c_master_init(void);

// Setups and requests a measurement on one of the four channels with the provided
// offset (value between 0 and 100), blocks while waiting for the result and writes
// it in `mesurement`. All values are in pF.
void fdc1004_measure(uint8_t channel, float offset, int32_t* measurement);

// Automatically adjust the offset value for a channel to be in the middle of
// the dynamic range of measurement
void fdc1004_calibrate(uint8_t channel, float* offset);

// Automatically calibrate every channel (offsets is an array of size 4)
void fdc1004_calibrate_all(float* offsets);

// Set the internal gain of the FDC1004 (measurements are no longer in pF)
void fdc1004_set_gain(uint16_t gain);
