/*
 * menu.h
 *
 *  Created on: 30 Nov 2025
 *      Author: dontk
 */

#ifndef MAIN_MENU_MENU_H_
#define MAIN_MENU_MENU_H_

#include "esp_err.h"
#include <stdint.h>

#define GPIO_LEFT	GPIO_NUM_27
#define GPIO_RIGHT	GPIO_NUM_14
#define GPIO_UP		GPIO_NUM_12
#define GPIO_DOWN	GPIO_NUM_13

// MENU_EVENTs to use in interrupts
#define MENU_EVENT_LEFT		(1 << 0)
#define MENU_EVENT_RIGHT	(1 << 1)
#define MENU_EVENT_UP		(1 << 2)
#define MENU_EVENT_DOWN		(1 << 3)

volatile uint8_t menu_event = 0;

typedef struct MenuItem MenuItem;

struct MenuItem {
	const char *name;		// Display name
	void (*action)(void);	// Function Pointer - Function to be executed
	
	MenuItem *parent;		// Pointer to parent node
	
	MenuItem **children;	// Pointer to multiple children nodes
	uint8_t num_children;	// Number of children
};

void menu_init(void);
void menu_next(void);
void menu_prev(void);
void menu_enter(void);
void menu_back(void);

MenuItem* menu_get_current(void);
uint8_t menu_get_cursor_index(void);

esp_err_t gpio_buttons_init(void);

#endif /* MAIN_MENU_MENU_H_ */
