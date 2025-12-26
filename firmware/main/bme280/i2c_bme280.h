/*
 * i2c_bme280.h
 *
 *  Created on: 13 Nov 2025
 *      Author: dontk
 */

#ifndef MAIN_BME280_I2C_BME280_H_
#define MAIN_BME280_I2C_BME280_H_

#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

#define BME280_ADDRESS		0x76
#define BME280_SDA_PIN		21
#define BME280_SCL_PIN		22	
#define I2C_MASTER_FREQ		100*1000	

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

esp_err_t i2c_init(void);

esp_err_t bme280_init(void);

//make read & write functions
esp_err_t i2c_read_reg(uint8_t dev,uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_write_reg(uint8_t dev,uint8_t reg, uint8_t *data, size_t len);

esp_err_t bme280_read_raw(bme280_raw_data_t *raw);
esp_err_t bme280_read_calibration(bme280_cali_data_t *cali);

int32_t bme280_compensate_t_int32(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);
uint32_t bme280_compensate_p_uint32(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);
float bme280_compensate_h_float(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data);

#endif /* MAIN_BME280_I2C_BME280_H_ */
