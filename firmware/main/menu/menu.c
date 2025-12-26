/*
 * menu.c
 *
 *  Created on: 30 Nov 2025
 *      Author: dontk
 */

#include "menu.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "hal/gpio_types.h"
#include "esp_attr.h"

static void IRAM_ATTR gpio_isr_handler(void *arg){
	
	gpio_num_t btn = (gpio_num_t) arg;
	
	switch(btn){
		case GPIO_UP:
			menu_event |= MENU_EVENT_UP;
			break;
		case GPIO_DOWN:
			menu_event |= MENU_EVENT_DOWN;
			break;
		case GPIO_RIGHT:
			menu_event |= MENU_EVENT_RIGHT;
			break;
		case GPIO_LEFT:
			menu_event |= MENU_EVENT_LEFT;
			break;
		default:
			break;
	}

}

esp_err_t gpio_buttons_init(void){
	
	esp_err_t ret;
	
	gpio_config_t io_conf = {
		.mode = GPIO_MODE_INPUT,
		.intr_type = GPIO_INTR_POSEDGE,
		.pin_bit_mask = (1ULL << GPIO_LEFT) | 
						(1ULL << GPIO_RIGHT) | 
						(1ULL << GPIO_UP) | 
						(1ULL << GPIO_DOWN),
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
	};
	
	ret = gpio_config(&io_conf);
	if (ret != ESP_OK) return ret;
	
	ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	if (ret != ESP_OK) return ret;
	
	ret = gpio_isr_handler_add(GPIO_LEFT, gpio_isr_handler, (void*)GPIO_LEFT);
	if (ret != ESP_OK) return ret;
	ret = gpio_isr_handler_add(GPIO_RIGHT, gpio_isr_handler, (void*)GPIO_RIGHT);
	if (ret != ESP_OK) return ret;
	ret = gpio_isr_handler_add(GPIO_UP, gpio_isr_handler, (void*)GPIO_UP);
	if (ret != ESP_OK) return ret;
	ret = gpio_isr_handler_add(GPIO_DOWN, gpio_isr_handler, (void*)GPIO_DOWN);
	if (ret != ESP_OK) return ret;
	
	return ESP_OK;
}



