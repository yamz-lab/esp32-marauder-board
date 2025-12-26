/*
 * wifi.h
 *
 *  Created on: 07 Dec 2025
 *      Author: dontk
 */

#ifndef MAIN_WIFI_WIFI_H_
#define MAIN_WIFI_WIFI_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

//#define CONNECTED_WIFI_SSID		"DEFAULT"
//#define WIFI_PASSWORD			"DEFAULT"

typedef struct {
    uint8_t subtype : 4;
    uint8_t type    : 2;
    uint8_t version : 2;
    uint8_t flags;
} wifi_frame_ctrl_t;

//static EventGroupHandle_t wifi_event_group;
void channel_hopper(void *arg);
void wifi_sniffer_init(void);
void wifi_sniffer_deinit(void);

#endif /* MAIN_WIFI_WIFI_H_ */
