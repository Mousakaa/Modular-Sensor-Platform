/* SPI Master Half Duplex EEPROM example.
 * Key layout :
 *    Y0 Y1 Y2 Y3 Y4 Y5 Y6 Y7
 * X0  0  8 16 24 32 40 48 56
 * X1  1  9 17 25 33 41 49 57
 * X2  2 10 18 26 34 42 50 58
 * X3  3 11 19 27 35 43 51 59
 * X4  4 12 20 28 36 44 52 60
 * X5  5 13 21 29 37 45 53 61
 * X6  6 14 22 30 38 46 54 62
 * X7  7 15 23 31 39 47 55 63
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#define SPI_CLOCK_FREQ 1000000 //1MHz (<4MHz)

#define PIN_NUM_RST  2
#define PIN_NUM_CS   3
#define PIN_NUM_CH   4
#define PIN_NUM_DRDY 5
#define PIN_NUM_CLK  8
#define PIN_NUM_MISO 9
#define PIN_NUM_MOSI 10

#define X_LEN 8
#define Y_LEN 8

#define ID_ADDR 0
#define DETECT_BLOCK_ADDR 6
#define COMMAND_ADDR 768
#define THR_SETUP_BLOCK_ADDR 769
#define DRIFT_SETUP_BLOCK_ADDR 833
#define DIL_SETUP_BLOCK_ADDR 897
#define NRD_SETUP_BLOCK_ADDR 961
#define THRM_HYST_SETUP_ADDR 1217
#define DWELL_SETUP_ADDR 1218
#define LSL_LSB_SETUP_ADDR 1219
#define LSL_MSB_SETUP_ADDR 1220
#define DHT_SETUP_ADDR 1222

#define CMD_CALIBRATE_ALL 0xff
#define CMD_ENABLE_WRITE_SETUP 0xfe
#define CMD_REQUEST_FREQ_HOP 0xfc
#define CMD_REQUEST_SLEEP 0xfb
#define CMD_FORCE_RESET 0x40

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

static const char TAG[] = "main";

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

void at42qt_spi_wait_for_ready(void) {
	vTaskDelay(0.1 / portTICK_PERIOD_MS);
	while(!gpio_get_level(PIN_NUM_DRDY));
}

void at42qt_spi_wait_for_change(void) {
	if(!gpio_get_level(PIN_NUM_CH)) {
		while(!gpio_get_level(PIN_NUM_CH));
	}
	while(gpio_get_level(PIN_NUM_CH));
}

void at42qt_spi_transmit_byte(spi_device_handle_t device, uint8_t send, uint8_t* receive) {
	spi_transaction_t transaction = {
		.length = 8,
		.tx_buffer = &send,
		.rx_buffer = receive
	};

	at42qt_spi_wait_for_ready();

	ESP_ERROR_CHECK(spi_device_transmit(device, &transaction));
}

void at42qt_spi_write(spi_device_handle_t device, uint16_t addr, uint8_t* data, size_t n) {
	at42qt_spi_transmit_byte(device, (uint8_t)addr, NULL);
	at42qt_spi_transmit_byte(device, ((n & (uint16_t)(1<<8)) >> 2) | ((addr & (uint16_t)(0b111 << 8)) >> 8), NULL);
	at42qt_spi_transmit_byte(device, (uint8_t)n, NULL);

	for(size_t i = 0; i < n; i++) {
		at42qt_spi_transmit_byte(device, data[i], NULL);
	}
}

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
		ESP_LOGE(TAG, "CRC check error");
		ESP_LOGD(TAG, "Calulated CRC : %x", crc);
		ESP_LOGD(TAG, "Received CRC : %x", (uint16_t)(crc_msb << 8) | crc_lsb);
		return 0;
	}
	return 1;
}

void at42qt_spi_wait_for_response(spi_device_handle_t device) {
	uint8_t id = 0;

	ESP_LOGI(TAG, "Waiting for response...");

	while(id != 0x1a) {
		vTaskDelay(25 / portTICK_PERIOD_MS);
		at42qt_spi_read(device, 0, &id, 1);
	}
}

void at42qt_spi_reset(spi_device_handle_t device) {
	ESP_LOGI(TAG, "Resetting...");
	gpio_set_level(PIN_NUM_RST, 0);
	vTaskDelay(1);
	gpio_set_level(PIN_NUM_RST, 1);

	at42qt_spi_wait_for_response(device);
}

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
    //Initialize the SPI bus
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

	at42qt_spi_reset(*device);
}

void at42qt_get_values(spi_device_handle_t device, uint16_t* values) {
	for(uint8_t y = 0; y < Y_LEN; y++) {
		for(uint8_t x = 0; x < X_LEN; x++) {
			uint8_t val[2];
			if(at42qt_spi_read(device, KEY_DATA_BLOCK_ADDR[y][x], val, 2)) {
				values[y+x*Y_LEN] = ((uint16_t)(val[1] & 0b11111) << 8) | val[0];
			}
		}
	}
}

void at42qt_send_command(spi_device_handle_t device, uint8_t cmd) {
	uint8_t command = cmd;
	at42qt_spi_write(device, COMMAND_ADDR, &command, 1);
}

void app_main(void) {
	spi_device_handle_t device;

	uint16_t values[X_LEN*Y_LEN];

	at42qt_spi_init(&device);
	at42qt_send_command(device, CMD_CALIBRATE_ALL);

	while(1) {
		at42qt_get_values(device, values);
		ESP_LOGI(TAG, "\033[2K\033[20D\tY0\tY1\tY2\tY3\tY4\tY5\tY6\tY7");
		ESP_LOGI(TAG, "\033[2K\033[20DX0\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7]);
		ESP_LOGI(TAG, "\033[2K\033[20DX1\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[8], values[9], values[10], values[11], values[12], values[13], values[14], values[15]);
		ESP_LOGI(TAG, "\033[2K\033[20DX2\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[16], values[17], values[18], values[19], values[20], values[21], values[22], values[23]);
		ESP_LOGI(TAG, "\033[2K\033[20DX3\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[24], values[25], values[26], values[27], values[28], values[29], values[30], values[31]);
		ESP_LOGI(TAG, "\033[2K\033[20DX4\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[32], values[33], values[34], values[35], values[36], values[37], values[38], values[39]);
		ESP_LOGI(TAG, "\033[2K\033[20DX5\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[40], values[41], values[42], values[43], values[44], values[45], values[46], values[47]);
		ESP_LOGI(TAG, "\033[2K\033[20DX6\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[48], values[49], values[50], values[51], values[52], values[53], values[54], values[55]);
		ESP_LOGI(TAG, "\033[2K\033[20DX7\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", values[56], values[57], values[58], values[59], values[60], values[61], values[62], values[63]);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		ESP_LOGI(TAG, "\033[10A");
	}
}
