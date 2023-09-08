/* SPI driver for AT42QT2640 sensor chip
 * Author : Arthur Gaudard - 2023
 *
 * Key layout :
 *
 *            Y0 Y1 Y2 Y3 Y4 Y5 Y6 Y7
 *         X0  0  8 16 24 32 40 48 56
 *         X1  1  9 17 25 33 41 49 57
 *         X2  2 10 18 26 34 42 50 58
 *         X3  3 11 19 27 35 43 51 59
 *         X4  4 12 20 28 36 44 52 60
 *         X5  5 13 21 29 37 45 53 61
 *         X6  6 14 22 30 38 46 54 62
 *         X7  7 15 23 31 39 47 55 63
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "spi_sensor.h"

#define SPI_CLOCK_FREQ 1000000 //1MHz (<4MHz)

// Pin layout
#define PIN_NUM_RST  2
#define PIN_NUM_CS   3
#define PIN_NUM_CH   4
#define PIN_NUM_DRDY 5
#define PIN_NUM_CLK  8
#define PIN_NUM_MISO 9
#define PIN_NUM_MOSI 10

// Internal registers and address blocks
#define ID_ADDR 0
#define FAIL_CNT_ADDR 3
#define DEVICE_STATUS_ADDR 5
#define DETECT_BLOCK_ADDR 6
#define COMMAND_ADDR 768
#define SETUP_BLOCK_ADDR 769
#define THR_SETUP_BLOCK_ADDR 769
#define DRIFT_SETUP_BLOCK_ADDR 833
#define DIL_SETUP_BLOCK_ADDR 897
#define NRD_SETUP_BLOCK_ADDR 961
#define BL_SETUP_BLOCK_ADDR 1025
#define THRM_HYST_SETUP_ADDR 1217
#define DWELL_SETUP_ADDR 1218
#define LSL_LSB_SETUP_ADDR 1219
#define LSL_MSB_SETUP_ADDR 1220
#define DHT_SETUP_ADDR 1222
#define FHM_SETUP_ADDR 1224
#define FREQ0_SETUP_ADDR 1225
#define HCRC_LSB_SETUP_ADDR 1246
#define HCRC_MSB_SETUP_ADDR 1247
#define SETUP_SIZE 477 // Size of the setup block (in bytes) without the 2 final CRC bytes

// Addresses of the per-key registers (8 bytes per key)
// Call like this : KEY_DATA_BLOCK_ADDR[y][x]
static const uint16_t KEY_DATA_BLOCK_ADDR[8][8] = {
	{ 26,  90, 154, 218, 282, 346, 410, 474 },
	{ 34,  98, 162, 226, 290, 354, 418, 482 },
	{ 42, 106, 170, 234, 298, 362, 426, 490 },
	{ 50, 114, 178, 242, 306, 370, 434, 498 },
	{ 58, 122, 186, 250, 314, 378, 442, 506 },
	{ 66, 130, 194, 258, 322, 386, 450, 514 },
	{ 74, 138, 202, 266, 330, 394, 458, 522 },
	{ 82, 146, 210, 274, 338, 402, 466, 530 }
};

static const char TAG[] = "AT42QT2640";

// CRC computing algorithm from AT42QT2640 datasheet
uint16_t sixteen_bit_crc(uint16_t crc, uint8_t data) {
	uint8_t index;// shift counter
	crc ^= (uint16_t)(data) << 8;
	index = 8;
	do {
		if(crc & 0x8000) {
			crc= (crc << 1) ^ 0x1021;
		}
		else {
			crc= crc << 1;
		}
	} while(--index);
	return crc;
}

// Blocks while DRDY is low
void at42qt_spi_wait_for_ready(void) {
	vTaskDelay(0.1 / portTICK_PERIOD_MS);
	while(!gpio_get_level(PIN_NUM_DRDY));
}

// Tests if there was a detection
bool at42qt_has_changed(void) {
	return(!gpio_get_level(PIN_NUM_CH));
}

// Sends a byte of data to the device on the MOSI line, and simultaneously reads a byte
// on the MISO line.
void at42qt_spi_transmit_byte(spi_device_handle_t device, uint8_t send, uint8_t* receive) {
	spi_transaction_t transaction = {
		.length = 8,
		.tx_buffer = &send,
		.rx_buffer = receive
	};

	at42qt_spi_wait_for_ready();

	ESP_ERROR_CHECK(spi_device_transmit(device, &transaction));
}

// Writes `n` bytes of data to the AT42QT2640, starting at the address `addr`
// (which is a 12-byte integer). `n` must be smaller than 512.
void at42qt_spi_write(spi_device_handle_t device, uint16_t addr, uint8_t* data, size_t n) {
	at42qt_spi_transmit_byte(device, (uint8_t)addr, NULL);
	at42qt_spi_transmit_byte(device, ((n & (uint16_t)(1<<8)) >> 2) | ((addr & (uint16_t)(0b111 << 8)) >> 8), NULL);
	at42qt_spi_transmit_byte(device, (uint8_t)n, NULL);

	for(size_t i = 0; i < n; i++) {
		at42qt_spi_transmit_byte(device, data[i], NULL);
	}
}

// Reads `n` bytes of data from the AT42QT2640, starting at the address `addr`
// (which is a 12-byte integer). `n` must be smaller than 512.
// The function return 1 if successful, 0 if the data is corrupted.
uint8_t at42qt_spi_read(spi_device_handle_t device, uint16_t addr, uint8_t* data, size_t n) {
	uint8_t crc_lsb;
	uint8_t crc_msb;
	uint16_t crc = 0;

	crc = sixteen_bit_crc(crc, (uint8_t)addr);
	crc = sixteen_bit_crc(crc, (1 <<7) | ((n & (uint16_t)(1<<8)) >> 2) | ((addr & (uint16_t)(0b111 << 8)) >> 8));
	crc = sixteen_bit_crc(crc, (uint8_t)n);

	at42qt_spi_transmit_byte(device, (uint8_t)addr, NULL);
	at42qt_spi_transmit_byte(device, (1 << 7) | ((n & (uint16_t)(1<<8)) >> 2) | ((addr & (uint16_t)(0b111 << 8)) >> 8), NULL);
	at42qt_spi_transmit_byte(device, (uint8_t)n, NULL);

	for(size_t i = 0; i < n; i++) {
		at42qt_spi_transmit_byte(device, 0x00, &(data[i]));
		crc = sixteen_bit_crc(crc, data[i]);
	}

	at42qt_spi_transmit_byte(device, 0x00, &crc_lsb);
	at42qt_spi_transmit_byte(device, 0x00, &crc_msb);

	if(crc != ((uint16_t)(crc_msb << 8) | crc_lsb)) {
		ESP_LOGE(TAG, "CRC check error\033[K");
		ESP_LOGD(TAG, "Calulated CRC : %x\033[K", crc);
		ESP_LOGD(TAG, "Received CRC : %x\033[K", (uint16_t)(crc_msb << 8) | crc_lsb);
		return 0;
	}
	return 1;
}

// Function to call directly after power-up or reset, to ensure the communication
// has started.
void at42qt_spi_wait_for_response(spi_device_handle_t device) {
	uint8_t id = 0;

	ESP_LOGI(TAG, "Waiting for response...");

	while(id != 0x1a) {
		vTaskDelay(25 / portTICK_PERIOD_MS);
		at42qt_spi_read(device, 0, &id, 1);
	}
}

// Hard reset of the AT42QT2640
void at42qt_spi_reset(spi_device_handle_t device) {
	ESP_LOGI(TAG, "Resetting...");
	gpio_set_level(PIN_NUM_RST, 0);
	vTaskDelay(1);
	gpio_set_level(PIN_NUM_RST, 1);

	at42qt_spi_wait_for_response(device);
}

// Initialization of non-SPI pins
void gpio_init() {
	gpio_set_direction(PIN_NUM_DRDY, GPIO_MODE_INPUT);
	gpio_set_intr_type(PIN_NUM_DRDY, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(PIN_NUM_DRDY, GPIO_FLOATING);

	gpio_set_direction(PIN_NUM_CH, GPIO_MODE_INPUT);
	gpio_set_intr_type(PIN_NUM_CH, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(PIN_NUM_CH, GPIO_FLOATING);

	gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
	gpio_set_intr_type(PIN_NUM_RST, GPIO_INTR_DISABLE);
	gpio_set_pull_mode(PIN_NUM_RST, GPIO_PULLUP_ENABLE);
}

// SPI bus initialization
void at42qt_spi_init(spi_device_handle_t* device) {
    esp_err_t ret;

	gpio_init();

    ESP_LOGI(TAG, "Initializing bus SPI2...");
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing device...");
	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SPI_CLOCK_FREQ,
		.command_bits = 0,
		.address_bits = 0,
		.mode = 3,
		.spics_io_num = PIN_NUM_CS,
		.cs_ena_pretrans = 100,
		.cs_ena_posttrans = 100,
		.queue_size = 1
	};

	ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, device));

	// Reset
	at42qt_spi_reset(*device);
}

// Send a command to the AT42QT2640
void at42qt_send_command(spi_device_handle_t device, uint8_t cmd) {
	uint8_t command = cmd;
	at42qt_spi_write(device, COMMAND_ADDR, &command, 1);
}

// Check relevent registers for various errors. Recalibrates all keys if a calibration error
// is encountered
void at42qt_check_errors(spi_device_handle_t device, uint8_t* prev_errors) {
	uint8_t error_cnt;
	uint8_t status;

	if(prev_errors != NULL && at42qt_spi_read(device, FAIL_CNT_ADDR, &error_cnt, 1) && error_cnt != *prev_errors) {
		ESP_LOGE(TAG, "Signal capture failure\033[K");
		*prev_errors = error_cnt;
	}

	if(at42qt_spi_read(device, DEVICE_STATUS_ADDR, &status, 1)) {
		if(status & (1 << 2)) {
			ESP_LOGE(TAG, "LSL failure\033[K");
		}
		if(status & (1 << 3)) {
			ESP_LOGE(TAG, "Mains sync error\033[K");
		}
		if(status & (1 << 4)) {
			ESP_LOGE(TAG, "Setup CRC mismatch\033[K");
		}
		if(status & (1 << 5)) {
			ESP_LOGE(TAG, "FMEA failure\033[K");
		}
		if(status & (1 << 6)) {
			ESP_LOGE(TAG, "Calibration error\033[K");
			at42qt_send_command(device, CMD_CALIBRATE_ALL);
		}
	}
}

// AT42QT2640 setup. Writes in internal EEPROM, so this setup is persistent.
void at42qt_setup(spi_device_handle_t device) {
	uint16_t crc = 0;
	uint8_t setup_block[SETUP_SIZE + 2];

	while(!at42qt_spi_read(device, SETUP_BLOCK_ADDR, setup_block, SETUP_SIZE));

	// MODIFY SETUP
	for(uint8_t y = 0; y < 8; y++) {
		for(uint8_t x = 0; x < 8; x++) {
			// Set negative thresholds to 26 cycles and positive thresholds to 6 cycles
			// to allow for quick recalibration on positive detection.
			setup_block[THR_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] = (0 << 4) | 10;
			// Set negative drift recalibration period to 1s and positive drift
			// recalibration period to 0.1s.
			setup_block[DRIFT_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] = 1;
			// Disable unused keys
			if(x >= CONFIG_X_LEN || y >= CONFIG_Y_LEN) {
				setup_block[DIL_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] &= 0xf << 4;
			}
			else {
				setup_block[DIL_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] = (5 << 4) | 2;
			}
			// Disable automatic recalibration for all keys
			setup_block[NRD_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] = 0;
			// Set burst length to 48 pulses for all keys
			setup_block[BL_SETUP_BLOCK_ADDR - SETUP_BLOCK_ADDR + x + y*8] |= 2 << 4;
		}
	}
	// Set dwell time to 2.1 microseconds
	setup_block[DWELL_SETUP_ADDR - SETUP_BLOCK_ADDR] = 10;
	// Set LSL to 100
	setup_block[LSL_LSB_SETUP_ADDR - SETUP_BLOCK_ADDR] = 100;
	setup_block[LSL_MSB_SETUP_ADDR - SETUP_BLOCK_ADDR] = 0;
	// Disable frequency hopping and set positive recalibration delay to 0.5s.
	setup_block[FHM_SETUP_ADDR - SETUP_BLOCK_ADDR] = (0 << 6) | 4;
	// Set burst frequency to
	setup_block[FREQ0_SETUP_ADDR - SETUP_BLOCK_ADDR] = 10;

	// CALCULATE CRC
	for(uint16_t i = 0; i < SETUP_SIZE; i++) {
		crc = sixteen_bit_crc(crc, setup_block[i]);
	}

	setup_block[HCRC_MSB_SETUP_ADDR - SETUP_BLOCK_ADDR] = (uint8_t)(crc >> 8);
	setup_block[HCRC_LSB_SETUP_ADDR - SETUP_BLOCK_ADDR] = (uint8_t)crc;
	
	// WRITE TO DEVICE
	at42qt_send_command(device, CMD_ENABLE_WRITE_SETUP);
	at42qt_spi_write(device, SETUP_BLOCK_ADDR, setup_block, SETUP_SIZE + 2);

	at42qt_check_errors(device, NULL);

	// RESET
	at42qt_spi_reset(device);
}

// Reads the status of each key in the array (0 for untouched and 1 for touched), and stores it
// in `values`.
void at42qt_get_status(spi_device_handle_t device, uint8_t* values) {
	uint8_t val;
	for(uint8_t y = 0; y < CONFIG_Y_LEN; y++) {
		if(at42qt_spi_read(device, DETECT_BLOCK_ADDR + y, &val, 1)) {
			for(uint8_t x = 0; x < CONFIG_X_LEN; x++) {
				values[y+x*CONFIG_Y_LEN] = (val & (1 << x)) >> x;
			}
		}
	}
}

// Reads the raw measurements of every key (on 13 bits), and stores the signed difference
// between each key's measurement and its reference value into `values`.
void at42qt_get_values(spi_device_handle_t device, int16_t* values) {
	uint8_t val[4];
	for(uint8_t y = 0; y < CONFIG_Y_LEN; y++) {
		for(uint8_t x = 0; x < CONFIG_X_LEN; x++) {
			if(at42qt_spi_read(device, KEY_DATA_BLOCK_ADDR[y][x], val, 2)) {
				values[y+x*CONFIG_Y_LEN] = (((int16_t)(val[1] & 0b11111) << 8) | val[0])
										 - (((int16_t)(val[3] & 0b11111) << 8) | val[2]);
			}
		}
	}
}
