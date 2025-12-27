/*
 * lcd_driver.c
 *
 *  Created on: 20 Jul 2025
 *      Author: dontk
 */

#include "lcd_driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "LCD";

static spi_device_handle_t lcd_spi;

void lcd_send_cmd(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0
    };
    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
}

void lcd_send_data(const uint8_t *data, int len) {
    if (len == 0) return;
    
    gpio_set_level(PIN_NUM_DC, 1);
    
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1
    };
    ESP_ERROR_CHECK(spi_device_transmit(lcd_spi, &t));
}

void lcd_reset() {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void lcd_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    lcd_send_cmd(0x2A);
    data[0] = x0 >> 8; data[1] = x0 & 0xFF;
    data[2] = x1 >> 8; data[3] = x1 & 0xFF;
    lcd_send_data(data, 4);

    lcd_send_cmd(0x2B);
    data[0] = y0 >> 8; data[1] = y0 & 0xFF;
    data[2] = y1 >> 8; data[3] = y1 & 0xFF;
    lcd_send_data(data, 4);

    lcd_send_cmd(0x2C);
}

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    lcd_set_address_window(x, y, x, y);
    uint8_t data[2] = { color >> 8, color & 0xFF };
    lcd_send_data(data, 2);
}

void lcd_fill_screen(uint16_t color) {
    lcd_set_address_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

    // Split the 16-bit color into two bytes
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    // Prepare a line buffer for speed (optional optimization)
    #define LINE_BUF_PIXELS 240  // Full width of screen
    uint8_t line_buf[LINE_BUF_PIXELS * 2];  // 2 bytes per pixel

    for (int i = 0; i < LINE_BUF_PIXELS; i++) {
        line_buf[2 * i] = hi;
        line_buf[2 * i + 1] = lo;
    }

    // Now send all lines
    for (int y = 0; y < LCD_HEIGHT; y++) {
        lcd_send_data(line_buf, sizeof(line_buf));
    }
}


void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    lcd_set_address_window(x, y, x + w - 1, y + h - 1);
    uint32_t len = w * h;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t data[2] = { color >> 8, color & 0xFF };
        lcd_send_data(data, 2);
    }
}

void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg) {
    if (c < 0x20 || c > 0x7F) c = '?';
    const uint8_t *bitmap = font5x7[c - 0x20];

    for (int col = 0; col < 5; col++) {
        uint8_t line = bitmap[col];
        for (int row = 0; row < 7; row++) {
            if (line & 0x01)
                lcd_draw_pixel(x + col, y + row, color);
            else
                lcd_draw_pixel(x + col, y + row, bg);
            line >>= 1;
        }
    }
}

void lcd_draw_text(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg) {
    while (*text) {
        lcd_draw_char(x, y, *text, color, bg);
        x += 6; // 5px wide + 1px spacing
        text++;
    }
}

void lcd_driver_init() {

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_HEIGHT*LCD_WIDTH*2 + 8
    };
    
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 26 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN));
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &lcd_spi));

    lcd_reset();

    lcd_send_cmd(0xEF);
    uint8_t data[] = {0x03, 0x80, 0x02};
    lcd_send_data(data, 3);

    lcd_send_cmd(0xCF);
    uint8_t data_cf[] = {0x00, 0xC1, 0x30};
    lcd_send_data(data_cf, 3);

    lcd_send_cmd(0xED);
    uint8_t data_ed[] = {0x64, 0x03, 0x12, 0x81};
    lcd_send_data(data_ed, 4);

    lcd_send_cmd(0xE8);
    uint8_t data_e8[] = {0x85, 0x00, 0x78};
    lcd_send_data(data_e8, 3);

    lcd_send_cmd(0xCB);
    uint8_t data_cb[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
    lcd_send_data(data_cb, 5);

    lcd_send_cmd(0xF7);
    uint8_t data_f7 = 0x20;
    lcd_send_data(&data_f7, 1);

    lcd_send_cmd(0xEA);
    uint8_t data_ea[] = {0x00, 0x00};
    lcd_send_data(data_ea, 2);

    lcd_send_cmd(0xC0); // Power control
    uint8_t data_c0 = 0x23;
    lcd_send_data(&data_c0, 1);

    lcd_send_cmd(0xC1); // Power control
    uint8_t data_c1 = 0x10;
    lcd_send_data(&data_c1, 1);

    lcd_send_cmd(0xC5); // VCM control
    uint8_t data_c5[] = {0x3e, 0x28};
    lcd_send_data(data_c5, 2);

    lcd_send_cmd(0xC7); // VCM control2
    uint8_t data_c7 = 0x86;
    lcd_send_data(&data_c7, 1);

    lcd_send_cmd(0x36); // Memory Access Control
    uint8_t data_36 = 0x48;
    lcd_send_data(&data_36, 1);

    lcd_send_cmd(0x3A);
    uint8_t data_3a = 0x55;
    lcd_send_data(&data_3a, 1);

    lcd_send_cmd(0xB1);
    uint8_t data_b1[] = {0x00, 0x18};
    lcd_send_data(data_b1, 2);

    lcd_send_cmd(0xB6);
    uint8_t data_b6[] = {0x08, 0x82, 0x27};
    lcd_send_data(data_b6, 3);

    lcd_send_cmd(0xF2);
    uint8_t data_f2 = 0x00;
    lcd_send_data(&data_f2, 1);

    lcd_send_cmd(0x26);
    uint8_t data_26 = 0x01;
    lcd_send_data(&data_26, 1);

    lcd_send_cmd(0xE0);
    uint8_t data_e0[] = {0x0F, 0x31, 0x2B, 0x0C,
                         0x0E, 0x08, 0x4E, 0xF1,
                         0x37, 0x07, 0x10, 0x03,
                         0x0E, 0x09, 0x00};
    lcd_send_data(data_e0, 15);

    lcd_send_cmd(0xE1);
    uint8_t data_e1[] = {0x00, 0x0E, 0x14, 0x03,
                         0x11, 0x07, 0x31, 0xC1,
                         0x48, 0x08, 0x0F, 0x0C,
                         0x31, 0x36, 0x0F};
    lcd_send_data(data_e1, 15);

    lcd_send_cmd(0x11);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_send_cmd(0x29);
    ESP_LOGI(TAG, "LCD Initialized");

	gpio_set_level(PIN_NUM_BCKL, 1);
	ESP_LOGI(TAG, "Blacklight ON");
	
}

void lcd_backlight_off(void)
{
	gpio_set_level(PIN_NUM_BCKL, 0);
	ESP_LOGI(TAG, "Blacklight test");
}

void lcd_sleep(void) {
    // Turn display off
    lcd_send_cmd(0x28); //turn display off
    vTaskDelay(pdMS_TO_TICKS(100)); // Small delay

    // Enter sleep mode
    lcd_send_cmd(0x10); //go to sleep
}



