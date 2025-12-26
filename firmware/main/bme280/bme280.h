/*
 * bme280.h
 *
 *  Created on: 18 Oct 2025
 *      Author: dontk
 */

#ifndef MAIN_BME280_BME280_H_
#define MAIN_BME280_BME280_H_

#include "esp_err.h"
#include "hal/gpio_types.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdint.h>

#define PIN_NUM_MISO		19
#define PIN_NUM_MOSI		23
#define PIN_NUM_CLK			18
#define PIN_NUM_CS_BME		15
//add later
//#define PIN_NUM_CS_LCD		5

//#define BME280_ADDRESS		0x76

extern spi_device_handle_t spi_bme;
//spi_device_handle_t spi_lcd;

typedef struct{
	int32_t raw_temp;
	int32_t raw_press;
	int32_t raw_hum;
}bme280_raw_data_t;

typedef struct{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
}bme280_cali_data_t;

void spi_dev_init(void);

esp_err_t bme280_spi_write(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t data);
esp_err_t bme280_spi_read(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t *data, size_t data_len);

esp_err_t bme280_read_raw(spi_device_handle_t spi_handle, bme280_raw_data_t *raw);
esp_err_t bme280_read_calibration(spi_device_handle_t spi_handle,bme280_cali_data_t *cali);

int32_t bme280_compensate_t_int32(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);
uint32_t bme280_compensate_p_uint32(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);
float bme280_compensate_h_float(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);

#endif /* MAIN_BME280_BME280_H_ */
