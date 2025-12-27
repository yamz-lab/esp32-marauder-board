/*
 * i2c_bme280.c
 *
 *  Created on: 13 Nov 2025
 *      Author: dontk
 */
 
#include "i2c_bme280.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"

static const char *I2C_TAG = "I2C_BME280";

int32_t t_fine; //used in 2 compensate functions

esp_err_t i2c_init(void){
	
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = BME280_SDA_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = BME280_SCL_PIN,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ,
	};
		
	esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);	
	
	if(ret != ESP_OK) return ret; // check param_config
	
	ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
	
	if(ret != ESP_OK) return ret; // check driver_install for errors
	
	ESP_LOGI(I2C_TAG,"I2C INITIALISED SUCCESSFULLY");
	
	i2c_set_timeout(I2C_NUM_0, 0xFFFFF);
	
	return ESP_OK;
	
}

esp_err_t bme280_init(void){
	
	uint8_t hum_oversample = 0x01; // ctrl_hum
	uint8_t ctrl_meas = 0x27; // ctrl_meas: temp x1, pressure x1, normal mode
	
	esp_err_t ret = i2c_write_reg(BME280_ADDRESS, 0xF2, &hum_oversample, 1);
	if(ret != ESP_OK) return ret;
           
	ret = i2c_write_reg(BME280_ADDRESS, 0xF4, &ctrl_meas, 1);
	if(ret != ESP_OK) return ret;
	
	ESP_LOGI(I2C_TAG,"BME280 INITIALISED SUCCESSFULLY");
	
	return ret;
	
}

esp_err_t i2c_read_reg(uint8_t dev_addr,uint8_t reg, uint8_t *data, size_t len){
	
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	esp_err_t ret;
	
	// Write register addr
	ret = i2c_master_start(cmd);
	if(ret != ESP_OK) return ret;
	
	ret = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
	if(ret != ESP_OK) return ret;
	
	ret = i2c_master_write_byte(cmd, reg, true);
	if(ret != ESP_OK) return ret;
	
	// Restart and read
	ret = i2c_master_start(cmd);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (ret != ESP_OK) return ret;

    // read len bytes
    ret = i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
	
	return ret;
	
}

esp_err_t i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len){
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    ret = i2c_master_start(cmd);
    if (ret != ESP_OK) return ret;

    // Device address + write bit
    ret = i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    if (ret != ESP_OK) return ret;

    // Register address to write to
    ret = i2c_master_write_byte(cmd, reg, true);
    if (ret != ESP_OK) return ret;

    // Write the actual data bytes
    ret = i2c_master_write(cmd, data, len, true);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_stop(cmd);
    if (ret != ESP_OK) return ret;

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t bme280_read_raw(bme280_raw_data_t *raw){
	
	uint8_t data[8];
	
	//esp_err_t ret = bme280_spi_read(spi_handle, 0xF7, data, 8);
	esp_err_t ret = i2c_read_reg(BME280_ADDRESS,0xF7,data,8);
	
	if (ret != ESP_OK) return ret;
	
	raw->raw_press = (int32_t)((((uint32_t)data[0] << 12) |
                                ((uint32_t)data[1] << 4) |
                                ((uint32_t)data[2] >> 4)));
    raw->raw_temp = (int32_t)((((uint32_t)data[3] << 12) |
                               ((uint32_t)data[4] << 4) |
                               ((uint32_t)data[5] >> 4)));
    raw->raw_hum  = (int32_t)(((uint32_t)data[6] << 8) | data[7]);
    
    ESP_LOGI(I2C_TAG, "BME280 data read successfully!");
    
    return ESP_OK;
	
}

esp_err_t bme280_read_calibration(bme280_cali_data_t *cali){
	
	uint8_t buf[26];
	uint8_t buf_h[7];
	
	esp_err_t ret;
	
	// Reading Coefficients
	//ret = bme280_spi_read(spi_handle, 0x88, buf, 26);
	ret = i2c_read_reg(BME280_ADDRESS, 0x88, buf, 26);
	
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
	
	//ret = bme280_spi_read(spi_handle, 0xE1, buf_h, 7);
	ret = i2c_read_reg(BME280_ADDRESS, 0xE1, buf_h, 7);	
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
