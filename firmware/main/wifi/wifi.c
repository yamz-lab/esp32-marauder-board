/*
 * wifi.c
 *
 *  Created on: 07 Dec 2025
 *      Author: dontk
 */

#include "wifi.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_wifi_types.h"
#include <stdint.h>
#include <string.h>

static const char *WIFI_TAG = "WIFI";

static void sniffer_packet_cb(void *buf, wifi_promiscuous_pkt_type_t type){
	//handle the packets
	const wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t*)buf;
	const uint8_t *payload = packet->payload;
	const wifi_frame_ctrl_t *fc = (wifi_frame_ctrl_t*)payload;
	
	int rssi = packet->rx_ctrl.rssi;
	
	//MAC Addresses in position
	const uint8_t *addr1 = payload + 4;
	const uint8_t *addr2 = payload + 10;
	const uint8_t *addr3 = payload + 16;
	
	ESP_LOGI(WIFI_TAG, "RSSI: %d dBm ", rssi);
	ESP_LOGI(WIFI_TAG, "Frame type: %d Subtype: %d", fc->type, fc->subtype);
	ESP_LOGI(WIFI_TAG, "RSSI: %d dBm ", rssi);
	
	char macbuf[32];
	
	sprintf(macbuf, "%02X:%02X:%02X:%02X:%02X:%02X",
        addr1[0], addr1[1], addr1[2], addr1[3], addr1[4], addr1[5]);
    ESP_LOGI(WIFI_TAG, "Addr1: %s", macbuf);
	
	sprintf(macbuf, "%02X:%02X:%02X:%02X:%02X:%02X",
        addr2[0], addr2[1], addr2[2], addr2[3], addr2[4], addr2[5]);
    ESP_LOGI(WIFI_TAG, "Addr2: %s", macbuf);
	
	sprintf(macbuf, "%02X:%02X:%02X:%02X:%02X:%02X",
        addr3[0], addr3[1], addr3[2], addr3[3], addr3[4], addr3[5]);
    ESP_LOGI(WIFI_TAG, "Addr3: %s", macbuf);
    
    // Isolate Management Frames type -> 00
    if(fc->type == 0){
		
		if(fc->subtype == 8){ // Beacon frame
			ESP_LOGI(WIFI_TAG,"Beacon Frame Detected");
			
			const uint8_t *ie = payload + 36; // Directly to Frame Body to SSID; check beacon layout @ https://howiwifi.com/2020/07/13/802-11-frame-types-and-formats/
			uint8_t ssid_len = ie[1];
			
			char ssid[33] = {0};
			if(ssid_len > 0 && ssid_len < 32){
				memcpy(ssid, &ie[2], ssid_len);
			}
			
			ESP_LOGI(WIFI_TAG, "SSID: %s", ssid[0] ? ssid : "<hidden>");
			ESP_LOGI(WIFI_TAG, "Channel: %d", packet->rx_ctrl.channel);
			
		}else if (fc->subtype == 4){ // Probe Request frame
			ESP_LOGI(WIFI_TAG, "Probe Request Detected");
			
			const uint8_t *ie = payload + 24; // Check link
			if (ie[0] == 0){
				uint8_t ssid_len = ie[1];
			
				char ssid[33] = {0};
				if(ssid_len > 0 && ssid_len < 32){
					memcpy(ssid, &ie[2], ssid_len);
					ESP_LOGI(WIFI_TAG, "Requested SSID: %s", ssid);
				} else {
					ESP_LOGI(WIFI_TAG, "Requested SSID: <broadcast>");	
				}
			}		
		// maybe add more stuff later
		} 
		
	}
	
}

// Task that switches between channels and sniffs
void channel_hopper(void *arg) {
    uint8_t channel = 1;
    while (1) {
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        printf("\n--- Switched to Channel %d ---\n", channel);

        channel++;
        if (channel > 13) channel = 1;
        vTaskDelay(pdMS_TO_TICKS(2000)); // hop every x sec
    }
}

void wifi_sniffer_init(void){
	
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
	
	const wifi_promiscuous_filter_t filter = {
		.filter_mask = 
				WIFI_PROMIS_FILTER_MASK_MGMT |
				WIFI_PROMIS_FILTER_MASK_DATA |
				WIFI_PROMIS_FILTER_MASK_CTRL 
	};
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(sniffer_packet_cb));
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous(1));
	
	//esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
	
}

void wifi_sniffer_deinit(void){
	ESP_ERROR_CHECK(esp_wifi_set_promiscuous(0));	// Exit promiscuous mode
	ESP_ERROR_CHECK(esp_wifi_stop());		// Stop
	ESP_ERROR_CHECK(esp_wifi_deinit());		// Free resources allocated to wifi_init
}
