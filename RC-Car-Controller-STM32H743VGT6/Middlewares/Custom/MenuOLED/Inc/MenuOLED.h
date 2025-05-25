/*
 * MenuOLED.h
 *
 *  Created on: May 25, 2025
 *      Author: Loochis
 */

#ifndef CUSTOM_MENUOLED_INC_MENUOLED_H_
#define CUSTOM_MENUOLED_INC_MENUOLED_H_

#include "SSD1306.h"

// STRUCTS
// ------------------------------------------------------------------------------------

typedef struct
{
	uint8_t					*name;			// string: name of this property

	uint8_t					packet_byte;	// which byte of the packet stores this value

	uint8_t					num_options;	// number of values this property can choose from

	uint8_t					**option_names;	// array of strings: values this property can choose from

	uint8_t					*option_values;	// array of numbers: values these options are stored as

} Menu_Property;

typedef struct
{

	uint8_t					*title;			// string: title of this menu page

	uint8_t					num_properties;	// number of properties this menu page has

	Menu_Property			*properties;	// array of properties (contiguous)

} Menu_Page;

typedef struct
{
	SSD1306_HandleTypeDef 	*ssdL_handle;	// ptr to SSD1306_HandleTypeDef which renders the LEFT menu

	SSD1306_HandleTypeDef 	*ssdR_handle;	// ptr to SSD1306_HandleTypeDef which renders the RIGHT menu

	uint8_t					num_pages;		// number of pages this menu has

	Menu_Page				*pages;			// array of pages (contiguous)

} Menu_HandleTypeDef;

// FUNCS
// ------------------------------------------------------------------------------------

// Utilities
uint8_t*	   	AllocateValueArr(uint8_t num);
uint8_t*       	AllocateString(uint8_t* str);
uint8_t**      	AllocateStringArr(uint8_t num);
Menu_Property* 	AllocateProperties(uint8_t num);
Menu_Page*     	AllocatePages(uint8_t num);

uint8_t MENU_Init(Menu_HandleTypeDef *hmenu);
uint8_t MENU_Draw(Menu_HandleTypeDef *hmenu, uint32_t delta_t);

#endif /* CUSTOM_MENUOLED_INC_MENUOLED_H_ */
