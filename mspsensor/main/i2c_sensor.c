/* I2C driver for FDC1004 sensor chip
 * Author : Arthur Gaudard - 2023
 * Based on ESP-IDF framework
 */

#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "i2c_sensor.h"

static const char *TAG = "FDC1004";

#define I2C_MASTER_SCL_IO           7		//GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           6		//GPIO number used for I2C master data
#define I2C_MASTER_NUM              0		//I2C master i2c port number
#define I2C_MASTER_FREQ_HZ          100000	//I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0		//I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0		//I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

#define FDC1004_SENSOR_ADDR         0b1010000		//Slave address of the FDC1004 sensor

// Internal register addresses
#define MEAS1_MSB                   0x00
#define MEAS1_LSB                   0x01
#define MEAS2_MSB                   0x02
#define MEAS2_LSB                   0x03
#define MEAS3_MSB                   0x04
#define MEAS3_LSB                   0x05
#define MEAS4_MSB                   0x06
#define MEAS4_LSB                   0x07
#define CONF_MEAS1                  0x08
#define CONF_MEAS2                  0x09
#define CONF_MEAS3                  0x0a
#define CONF_MEAS4                  0x0b
#define FDC_CONF                    0x0c
#define GAIN_CAL_CIN1               0x11
#define GAIN_CAL_CIN2               0x12
#define GAIN_CAL_CIN3               0x13
#define GAIN_CAL_CIN4               0x14

// Read a 16-bit integer from a FDC1004 register
void fdc1004_read(uint8_t reg_addr, uint16_t *data) {
	uint8_t buf[2];

    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_MASTER_NUM, FDC1004_SENSOR_ADDR, &reg_addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	*data = ((uint16_t)buf[0] << 8) | buf[1];
}

// Write a 16-bit integer to a FDC1004 sensor register
void fdc1004_write(uint8_t reg_addr, uint16_t data) {
	uint8_t buf[] = {reg_addr, (uint8_t)(data >> 8), (uint8_t)data};

    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, FDC1004_SENSOR_ADDR, buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
}

// I2C master initialization
void i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

    ESP_LOGI(TAG, "I2C for FDC1004 initialized successfully");
}

// Set the internal gain of the FDC1004 (measurements are no longer in pF)
void fdc1004_set_gain(uint16_t gain) {
	fdc1004_write(GAIN_CAL_CIN1, gain);
	fdc1004_write(GAIN_CAL_CIN2, gain);
	fdc1004_write(GAIN_CAL_CIN3, gain);
	fdc1004_write(GAIN_CAL_CIN4, gain);
}

// Setup and wait for data from one of the four channels of the device
void fdc1004_measure(uint8_t channel, float offset, int32_t* measurement) {

	uint8_t ch_id = channel - 1;

	uint16_t is_done;
	uint16_t meas1_msb;
	uint16_t meas1_lsb;

	fdc1004_write(
		CONF_MEAS1, // MEASUREMENT 1 CONFIGURATION
		(ch_id << 13) | // Measure on channel 1
		(0b100 << 10) | // Enable CAPDAC (offset)
		(((uint8_t)(offset / 3.125) & 0b11111) << 5) // Set offset
	);

	fdc1004_write(
		FDC_CONF, // REGISTER DESCRIPTION
		(0b01 << 10) | // Set sample rate at 100 S/s
		(0b1000 << 4) // Enable measurement 1
	);

	do {
		fdc1004_read(FDC_CONF, &is_done);
	}
	while((is_done & 0x000f) != 0x0008); // Wait until measurement 1 is done
	
	fdc1004_read(MEAS1_MSB, &meas1_msb);
	fdc1004_read(MEAS1_LSB, &meas1_lsb);

	*measurement = (int32_t)((meas1_msb << 16) | meas1_lsb) / 256;
}

// Automatically adjust the offset value for a channel
void fdc1004_calibrate(uint8_t channel, float* offset) {
	int32_t meas;
	float A = 0.0;
	float B = 100.0;
	while(B-A > 1.0) {
		*offset = (A + B) / 2.0;
		fdc1004_measure(channel, *offset, &meas);
		if(meas > 0) {
			A = *offset;
		}
		else {
			B = *offset;
		}
	}
	ESP_LOGI(TAG, "Channel %d offset : %f", channel, *offset);
}

// Automatically calibrate every channel
void fdc1004_calibrate_all(float* offsets) {
	for(uint8_t ch = 1; ch <= 4; ch++) {
		fdc1004_calibrate(ch, &(offsets[ch-1]));
	}
}
