#include "driver/i2c.h"

// I2C initialization for FDC1004, needs to be called prior to any measurement
void i2c_master_init(void);

// Setups and requests a measurement on one of the four channels with the provided
// offset (value between 0 and 100), blocks while waiting for the result and writes
// it in `mesurement`. All values are in pF.
void fdc1004_measure(uint8_t channel, float offset, float* measurement);
