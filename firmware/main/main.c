#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "bme280/i2c_bme280.h"
#include "lcd_driver/lcd_driver.h"
#include "gui/gui.h"
#include "wifi/wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"


typedef struct{
	int32_t temperature;
	float pressure;
	float humidity;
}bme280_actual_data_t;

bme280_actual_data_t calibrated_reading;

static const char *MAIN_TAG = "MAIN";

// Define tasks
void I2C_BME280_TASK(void *parameters);

void app_main(void)
{
	//i2c_init();
	//bme280_init();
	//lcd_driver_init();
	nvs_flash_init();
	wifi_sniffer_init();
	
	xTaskCreate(channel_hopper, "channel_hopper", 2048, NULL, 1, NULL);
	
	//lcd_fill_screen(COLOR_BLACK); // background
	
	// Displaying Icons
	//display_16x16_icon(wifi_icon_false, 10, 10);
	//display_16x16_icon(blue_icon_true, 30, 10); // Test, first get wifi and ble working and then have icons work with bool variable
	//display_16x16_icon(temp_icon, 10, 50);
	//display_16x16_icon(pressure_icon, 10, 70);
	//display_16x16_icon(humidity_icon, 10, 90);
	
	//xTaskCreate(I2C_BME280_TASK, "BME280_TASK", 4096, NULL, 5, NULL);
	
}

void I2C_BME280_TASK(void *parameters){
	
	bme280_cali_data_t cali;
	bme280_raw_data_t adc_reading;
	
	bme280_read_calibration(&cali);
		
	while(true){
		bme280_read_raw(&adc_reading);
		
		// Convert
		calibrated_reading.temperature = bme280_compensate_t_int32(&adc_reading, &cali)/100;
		calibrated_reading.pressure = bme280_compensate_p_uint32(&adc_reading, &cali)/1000.0f;
		calibrated_reading.humidity = bme280_compensate_h_float(&adc_reading, &cali);
		
		ESP_LOGI(MAIN_TAG, "Temperature: %" PRId32 " C | Pressure: %.2f kPa | Humidity: %.2f %%", 
         calibrated_reading.temperature, 
         calibrated_reading.pressure, 
         calibrated_reading.humidity);
         
        char buf[32];
         
        snprintf(buf, sizeof(buf), "%" PRId32 " C", calibrated_reading.temperature);
        lcd_draw_text(30, 50, buf, COLOR_WHITE, COLOR_BLACK);
        snprintf(buf, sizeof(buf), "%.2f kPa", calibrated_reading.pressure);
		lcd_draw_text(30, 70, buf, COLOR_WHITE, COLOR_BLACK);
		snprintf(buf, sizeof(buf), "%.2f %%", calibrated_reading.humidity);
		lcd_draw_text(30, 90, buf, COLOR_WHITE, COLOR_BLACK);
		
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}