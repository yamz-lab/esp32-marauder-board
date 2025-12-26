/*
 * bme280.c
 *
 *  Created on: 18 Oct 2025
 *      Author: dontk
 */
#include <stdint.h>
#include <string.h>
#include "bme280.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "hal/spi_types.h"

static const char *TAG = "SPI_DEVICES";

spi_device_handle_t spi_bme;

int32_t t_fine; //used in 2 compensate functions

void spi_dev_init(){
	
	esp_err_t ret;
	
	// Configure SPI bus
	spi_bus_config_t bus_config = {
		.miso_io_num = PIN_NUM_MISO,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
	};
	
	ret = spi_bus_initialize(HSPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);
	
	// Add BME280 device
	spi_device_interface_config_t bme_dev_config = {
		.clock_speed_hz = 5 * 1000 * 1000,
		.mode = 0,
		.spics_io_num = PIN_NUM_CS_BME,
		.queue_size = 3,
	};
	
	ret = spi_bus_add_device(HSPI_HOST, &bme_dev_config, &spi_bme);
	ESP_ERROR_CHECK(ret);
	
	ESP_LOGI(TAG, "SPI bus with BME280 initialized successfully!");
	
}

esp_err_t bme280_spi_write(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t data){
	
	// Create transmit buffer
	uint8_t tx_data[2];
	tx_data[0] = reg_addr & 0x7f;	// Make MSB 0 for WRITE
	tx_data[1] = data;				// Byte to send
	
	// Prepare SPI transaction
	spi_transaction_t spi_trans = {
		.length = 8 * sizeof(tx_data),	// Total bits to transfer
		.tx_buffer = tx_data,			// Data transmitting
		.rx_buffer = NULL,				// Not reading anything
	};
	
	return spi_device_polling_transmit(spi_handle, &spi_trans);
	
}

esp_err_t bme280_spi_read(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t *data, size_t data_len){
	
	uint8_t tx_buf[1] = {reg_addr | 0x80};
	
	uint8_t rx_buf[1 + data_len];
	
	spi_transaction_t spi_trans = {
		.length = 8 * (1 + data_len),
		.tx_buffer = tx_buf,
		.rx_buffer = rx_buf,
	};
	
	esp_err_t ret = spi_device_polling_transmit(spi_handle, &spi_trans);
	
	if (ret == ESP_OK){
		memcpy(data, &rx_buf[1], data_len);
		ESP_LOGI(TAG, "SPI read successful!");
	}
	
	return ret;
	
}

esp_err_t bme280_read_raw(spi_device_handle_t spi_handle, bme280_raw_data_t *raw){
	
	uint8_t data[8];
	
	esp_err_t ret = bme280_spi_read(spi_handle, 0xF7, data, 8);
	
	if (ret != ESP_OK) return ret;
	
	raw->raw_press = (int32_t)((((uint32_t)data[0] << 12) |
                                ((uint32_t)data[1] << 4) |
                                ((uint32_t)data[2] >> 4)));
    raw->raw_temp = (int32_t)((((uint32_t)data[3] << 12) |
                               ((uint32_t)data[4] << 4) |
                               ((uint32_t)data[5] >> 4)));
    raw->raw_hum  = (int32_t)(((uint32_t)data[6] << 8) | data[7]);
    
    ESP_LOGI(TAG, "BME280 data read successfully!");
    
    return ESP_OK;
	
}

esp_err_t bme280_read_calibration(spi_device_handle_t spi_handle,bme280_cali_data_t *cali){
	
	uint8_t buf[26];
	uint8_t buf_h[7];
	
	esp_err_t ret;
	
	// Reading Coefficients
	ret = bme280_spi_read(spi_handle, 0x88, buf, 26);
	if (ret != ESP_OK) return ret;
	
	cali->dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
	cali->dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
	cali->dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
	cali->dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
	cali->dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
	cali->dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
	cali->dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
	cali->dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
	cali->dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
	cali->dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
	cali->dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
	cali->dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);
	cali->dig_H1 = buf[25];	// 0xA0 skipped. Check Datasheet
	
	ret = bme280_spi_read(spi_handle, 0xE1, buf_h, 7);
	if (ret != ESP_OK) return ret;
	
	cali->dig_H2 = (int16_t)((buf_h[1] << 8) | buf_h[0]);
	cali->dig_H3 = buf_h[2];
	cali->dig_H4 = (int16_t)((buf_h[3] << 4) | (buf_h[4] & 0x0F));
	cali->dig_H5 = (int16_t)((buf_h[5] << 4) | (buf_h[4] >> 4));
	cali->dig_H6 = (int8_t)buf_h[6];
	
	return ESP_OK;
	
}

int32_t bme280_compensate_t_int32(bme280_raw_data_t *raw_data, bme280_cali_data_t *cali_data){
	
	// Get temperature from conversion -- check datasheet
	int32_t var1, var2;
    var1 = ((((raw_data->raw_temp >> 3) - ((int32_t)cali_data->dig_T1 << 1))) * ((int32_t)cali_data->dig_T2)) >> 11;
    var2 = (((((raw_data->raw_temp >> 4) - ((int32_t)cali_data->dig_T1)) * ((raw_data->raw_temp >> 4) - ((int32_t)cali_data->dig_T1))) >> 12) *
            ((int32_t)cali_data->dig_T3)) >> 14;
    t_fine = var1 + var2;
    int32_t t = (t_fine * 5 + 128) >> 8;
    return t;
		
}

uint32_t bme280_compensate_p_uint32(bme280_raw_data_t *raw_data, bme280_cali_data_t *cali_data){
	
	int32_t var1, var2;
	var1 = ((t_fine >> 1) - (int32_t)64000);
	var2 = (((var1 >> 2) * (var1 >> 2) >> 11) * (int32_t)cali_data->dig_P6);
	var2 = var2 + ((var1 * ((int32_t)cali_data->dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)cali_data->dig_P4) << 16);
	var1 = (((cali_data->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) 
			+ ((((int32_t)cali_data->dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)cali_data->dig_P1)) >> 15);
	
	if (var1 == 0) return 0;
	
	uint32_t p = (((uint32_t)(((int32_t)1048576 - raw_data->raw_press) - (var2 >> 12))) * 3125);
	
	if (p < 0x80000000){
		p = (p << 1) / ((uint32_t)var1);
	}
	else {
		p = (p / (uint32_t)var1) * 2;
	}
	
	var1 = (((int32_t)cali_data->dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)cali_data->dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + cali_data->dig_P7) >> 4));
	return p;
	
}

float bme280_compensate_h_float(bme280_raw_data_t *raw_data,bme280_cali_data_t *cali_data){
	
	float var_h;
	
	var_h = (((float)t_fine) - 76800.0);
	var_h = (raw_data->raw_hum - (((float)cali_data->dig_H4) * 64.0 + ((float)cali_data->dig_H5) / 16384.0 * 
			var_h)) * (((float)cali_data->dig_H2) / 65536.0 * (1.0 + ((float)cali_data->dig_H6) / 67108864.0
			* var_h * (1.0 + ((float)cali_data->dig_H3) / 67108864.0 * var_h)));
	var_h = var_h * (1.0 - ((float)cali_data->dig_H1) * var_h / 524288.0);
	
	if (var_h > 100.0){
		var_h = 100.0;
	}else if (var_h < 0.0){
		var_h = 0.0;
	}
	
	return var_h;
	
}

