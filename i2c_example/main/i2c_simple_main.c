/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <time.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           7                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           6                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define FDC1004_SENSOR_ADDR         0b1010000                  /*!< Slave address of the FDC1004 sensor */

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

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
void fdc1004_read(uint8_t reg_addr, uint16_t *data) {
	uint8_t buf[2];

    ESP_ERROR_CHECK(i2c_master_write_read_device(I2C_MASTER_NUM, FDC1004_SENSOR_ADDR, &reg_addr, 1, buf, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));

	*data = ((uint16_t)buf[0] << 8) | buf[1];
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
void fdc1004_write(uint8_t reg_addr, uint16_t data) {
	uint8_t buf[] = {reg_addr, (uint8_t)(data >> 8), (uint8_t)data};

    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, FDC1004_SENSOR_ADDR, buf, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
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

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void fdc1004_measure(uint8_t channel, float offset, float* measurement) {

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

	*measurement = (float)((int32_t)((meas1_msb << 16) | meas1_lsb) / 256) / 524288.0 + offset;
}

void app_main(void) {

	float offset_cap[] = {0.0, 0.0, 0.0, 0.0}; // Offset on each channel, in pF (between 0 and 100)
	float measurement;

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

	while(true) {
		fdc1004_measure(1, offset_cap[0], &measurement);
		ESP_LOGI(TAG, "CIN1 = %fpF", measurement);
		fdc1004_measure(2, offset_cap[1], &measurement);
		ESP_LOGI(TAG, "CIN2 = %fpF", measurement);
		fdc1004_measure(3, offset_cap[2], &measurement);
		ESP_LOGI(TAG, "CIN3 = %fpF", measurement);
		fdc1004_measure(4, offset_cap[3], &measurement);
		ESP_LOGI(TAG, "CIN4 = %fpF", measurement);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		ESP_LOGI("", "-----------------------------------\033[5A");
	}
}
