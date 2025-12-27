/*
 * gui.c
 *
 *  Created on: 28 Nov 2025
 *      Author: dontk
 */
 
 #include "gui.h"

static const char* GUI_TAG = "GUI";

void display_16x16_icon(uint16_t const *icon, uint16_t x, uint16_t y){
	gui_draw_icon_16x16(x, y, icon, COLOR_WHITE, COLOR_BLACK);
	ESP_LOGI(GUI_TAG, "Icon displayed onto screen");
}

void gui_draw_icon_16x16(uint16_t x, uint16_t y, const uint16_t icon[16], uint16_t color, uint16_t bg){
	
	for (int row = 0; row < 16; row++){
		uint16_t bits = icon[row];
		for (int col = 0; col < 16; col++){
			if (bits & (1 << (15 - col))){
				lcd_draw_pixel(x + col, y + row, color);
			}else if (bg != 0xFFFF){
				lcd_draw_pixel(x + col, y + row, bg);
			}
		}	
	}
}
