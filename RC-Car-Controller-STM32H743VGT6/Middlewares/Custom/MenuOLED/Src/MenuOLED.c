/*
 * MenuOLED.c
 *
 *  Created on: May 25, 2025
 *      Author: Loochis
 */

#include "MenuOLED.h"

// FUNCS
// ------------------------------------------------------------------------------------

uint8_t* AllocateValueArr(uint8_t num) {
	return (uint8_t*)malloc(sizeof(uint8_t)*num);
}

uint8_t* AllocateString(uint8_t* src) {
	uint8_t str_tmp[100];
	uint8_t* dst = (uint8_t*)malloc(strlen(src)+1);
	strcpy(dst, src);
	return dst;
}

uint8_t** AllocateStringArr(uint8_t num) {
	return (uint8_t**)malloc(sizeof(uint8_t*)*num);
}

Menu_Property* AllocateProperties(uint8_t num) {
	return (Menu_Property*)malloc(sizeof(Menu_Property)*num);
}

Menu_Page* AllocatePages(uint8_t num) {
	return (Menu_Page*)malloc(sizeof(Menu_Page)*num);
}

uint8_t MENU_Init(Menu_HandleTypeDef *hmenu) {
	// a buncha of work incoming

	// Allocate pages
	hmenu->num_pages = 2;
	hmenu->pages = AllocatePages(hmenu->num_pages);

	// PAGE 0 (CAMERA)
	hmenu->pages[0].title = AllocateString("CAMERA");

	hmenu->pages[0].num_properties = 2;
	hmenu->pages[0].properties = AllocateProperties(hmenu->pages[0].num_properties);

	// Camera Mode
	hmenu->pages[0].properties[0].name = AllocateString("ENCODING");
	hmenu->pages[0].properties[0].packet_byte = 0;

	hmenu->pages[0].properties[0].num_options = 3;
	hmenu->pages[0].properties[0].option_names = AllocateStringArr(hmenu->pages[0].properties[0].num_options);
	hmenu->pages[0].properties[0].option_names[0] = AllocateString("JPEG");
	hmenu->pages[0].properties[0].option_names[1] = AllocateString("GRAYSCALE");
	hmenu->pages[0].properties[0].option_names[2] = AllocateString("COLOUR");

	hmenu->pages[0].properties[0].option_values = AllocateValueArr(hmenu->pages[0].properties[0].num_options);
	hmenu->pages[0].properties[0].option_names[0] = 0x00;
	hmenu->pages[0].properties[0].option_names[1] = 0x01;
	hmenu->pages[0].properties[0].option_names[2] = 0x02;

	// Camera Quality
	hmenu->pages[0].properties[1].name = AllocateString("QUALITY");
	hmenu->pages[0].properties[1].packet_byte = 1;

	hmenu->pages[0].properties[1].num_options = 4;
	hmenu->pages[0].properties[1].option_names = AllocateStringArr(hmenu->pages[0].properties[1].num_options);
	hmenu->pages[0].properties[1].option_names[0] = AllocateString("F**KED");
	hmenu->pages[0].properties[1].option_names[1] = AllocateString("WORSE");
	hmenu->pages[0].properties[1].option_names[2] = AllocateString("BAD");
	hmenu->pages[0].properties[1].option_names[3] = AllocateString("EVENTUALLY");

	hmenu->pages[0].properties[1].option_values = AllocateValueArr(hmenu->pages[0].properties[1].num_options);
	hmenu->pages[0].properties[1].option_names[0] = 0x00;
	hmenu->pages[0].properties[1].option_names[1] = 0x01;
	hmenu->pages[0].properties[1].option_names[2] = 0x02;
	hmenu->pages[0].properties[1].option_names[3] = 0x03;

	// PAGE 1 (LIGHTS)
	hmenu->pages[1].title = AllocateString("LIGHTING");

	hmenu->pages[1].num_properties = 4;
	hmenu->pages[1].properties = AllocateProperties(hmenu->pages[1].num_properties);

	hmenu->pages[1].properties[0].name = AllocateString("HEADLIGHTS");
	hmenu->pages[1].properties[0].packet_byte = 2;

	hmenu->pages[1].properties[0].num_options = 5;
	hmenu->pages[1].properties[0].option_names = AllocateStringArr(hmenu->pages[1].properties[0].num_options);
	hmenu->pages[1].properties[0].option_names[0] = AllocateString("[    ]");
	hmenu->pages[1].properties[0].option_names[1] = AllocateString("[|   ]");
	hmenu->pages[1].properties[0].option_names[2] = AllocateString("[||  ]");
	hmenu->pages[1].properties[0].option_names[3] = AllocateString("[||| ]");
	hmenu->pages[1].properties[0].option_names[4] = AllocateString("[||||]");

	hmenu->pages[1].properties[0].option_values = AllocateValueArr(hmenu->pages[1].properties[0].num_options);
	hmenu->pages[1].properties[0].option_names[0] = 0x00;
	hmenu->pages[1].properties[0].option_names[1] = 0x01;
	hmenu->pages[1].properties[0].option_names[2] = 0x02;
	hmenu->pages[1].properties[0].option_names[3] = 0x03;
	hmenu->pages[1].properties[0].option_names[4] = 0x04;

	hmenu->pages[1].properties[1].name = AllocateString("INTERIOR R");
	hmenu->pages[1].properties[1].packet_byte = 3;

	hmenu->pages[1].properties[1].num_options = 5;
	hmenu->pages[1].properties[1].option_names = AllocateStringArr(hmenu->pages[1].properties[1].num_options);
	hmenu->pages[1].properties[1].option_names[0] = AllocateString("[    ]");
	hmenu->pages[1].properties[1].option_names[1] = AllocateString("[|   ]");
	hmenu->pages[1].properties[1].option_names[2] = AllocateString("[||  ]");
	hmenu->pages[1].properties[1].option_names[3] = AllocateString("[||| ]");
	hmenu->pages[1].properties[1].option_names[4] = AllocateString("[||||]");

	hmenu->pages[1].properties[1].option_values = AllocateValueArr(hmenu->pages[1].properties[1].num_options);
	hmenu->pages[1].properties[1].option_names[0] = 0x00;
	hmenu->pages[1].properties[1].option_names[1] = 0x01;
	hmenu->pages[1].properties[1].option_names[2] = 0x02;
	hmenu->pages[1].properties[1].option_names[3] = 0x03;
	hmenu->pages[1].properties[1].option_names[4] = 0x04;

	hmenu->pages[1].properties[2].name = AllocateString("INTERIOR G");
	hmenu->pages[1].properties[2].packet_byte = 4;

	hmenu->pages[1].properties[2].num_options = 5;
	hmenu->pages[1].properties[2].option_names = AllocateStringArr(hmenu->pages[1].properties[2].num_options);
	hmenu->pages[1].properties[2].option_names[0] = AllocateString("[    ]");
	hmenu->pages[1].properties[2].option_names[1] = AllocateString("[|   ]");
	hmenu->pages[1].properties[2].option_names[2] = AllocateString("[||  ]");
	hmenu->pages[1].properties[2].option_names[3] = AllocateString("[||| ]");
	hmenu->pages[1].properties[2].option_names[4] = AllocateString("[||||]");

	hmenu->pages[1].properties[2].option_values = AllocateValueArr(hmenu->pages[1].properties[2].num_options);
	hmenu->pages[1].properties[2].option_names[0] = 0x00;
	hmenu->pages[1].properties[2].option_names[1] = 0x01;
	hmenu->pages[1].properties[2].option_names[2] = 0x02;
	hmenu->pages[1].properties[2].option_names[3] = 0x03;
	hmenu->pages[1].properties[2].option_names[4] = 0x04;

	hmenu->pages[1].properties[3].name = AllocateString("INTERIOR B");
	hmenu->pages[1].properties[3].packet_byte = 5;

	hmenu->pages[1].properties[3].num_options = 5;
	hmenu->pages[1].properties[3].option_names = AllocateStringArr(hmenu->pages[1].properties[3].num_options);
	hmenu->pages[1].properties[3].option_names[0] = AllocateString("[    ]");
	hmenu->pages[1].properties[3].option_names[1] = AllocateString("[|   ]");
	hmenu->pages[1].properties[3].option_names[2] = AllocateString("[||  ]");
	hmenu->pages[1].properties[3].option_names[3] = AllocateString("[||| ]");
	hmenu->pages[1].properties[3].option_names[4] = AllocateString("[||||]");

	hmenu->pages[1].properties[3].option_values = AllocateValueArr(hmenu->pages[1].properties[3].num_options);
	hmenu->pages[1].properties[3].option_names[0] = 0x00;
	hmenu->pages[1].properties[3].option_names[1] = 0x01;
	hmenu->pages[1].properties[3].option_names[2] = 0x02;
	hmenu->pages[1].properties[3].option_names[3] = 0x03;
	hmenu->pages[1].properties[3].option_names[4] = 0x04;

	return 0;
}

uint8_t MENU_Draw(Menu_HandleTypeDef *hmenu, uint32_t delta_t) {
	// Do the animations
	if (hmenu->page_anim != 0xFF)
		hmenu->page_anim++;

	if (hmenu->property_anim != 0xFF)
		hmenu->property_anim++;

	// Draw the title on the left
	hmenu->ssdL_handle->str_cursor = (128 - strlen(hmenu->pages[hmenu->current_page].title)*6);
	MENU_AnimateString(hmenu, hmenu->ssdL_handle,
			hmenu->pages[hmenu->current_page].title,
			hmenu->page_anim, 0);

	// Draw the properties
	for (uint8_t i = 0; i < hmenu->pages[hmenu->current_page].num_properties; i++) {
		// Compute offset using property anim
		hmenu->ssdL_handle->str_cursor = 10 + (2+i)*128;
		if (i == hmenu->current_property) {
			uint8_t num_bars = hmenu->property_anim;
			if (num_bars > 3) num_bars = 3;
			SSD1306_DrawString(hmenu->ssdL_handle, ">> ", num_bars);
		}

		// De-animate the previous property
		if (i == hmenu->last_property && hmenu->property_anim/2 <= 2) {
			uint8_t num_bars = 2 - hmenu->property_anim/2;
			if (num_bars > 2) num_bars = 2;
			SSD1306_DrawString(hmenu->ssdL_handle, ">> ", num_bars);
		}

		MENU_AnimateString(hmenu, hmenu->ssdL_handle,
				hmenu->pages[hmenu->current_page].properties[i].name,
				hmenu->page_anim, 6+i*2);
	}

	// Draw the selected property on the RIGHT
	hmenu->ssdR_handle->str_cursor = 0;
	MENU_AnimateString(hmenu, hmenu->ssdR_handle,
				hmenu->pages[hmenu->current_page].properties[hmenu->current_property].name,
				hmenu->property_anim, 1);
}

void MENU_ParseInput(Menu_HandleTypeDef *hmenu, uint8_t inputs[4]) {
	if (inputs[0]) {
		if (hmenu->current_page == 0) {
			hmenu->current_page = hmenu->num_pages - 1;
		} else {
			hmenu->current_page--;
		}
		hmenu->page_anim = 0;
		hmenu->property_anim = 0;
		hmenu->current_property = 0;
		hmenu->last_property = 0xFF;
	} else if (inputs[1]) {
		if (hmenu->current_page == hmenu->num_pages - 1) {
			hmenu->current_page = 0;
		} else {
			hmenu->current_page++;
		}
		hmenu->page_anim = 0;
		hmenu->property_anim = 0;
		hmenu->current_property = 0;
		hmenu->last_property = 0xFF;
	}

	if (inputs[3]) {
		hmenu->last_property = hmenu->current_property;
		if (hmenu->current_property == hmenu->pages[hmenu->current_page].num_properties - 1)
			hmenu->current_property = 0;
		else
			hmenu->current_property++;
		hmenu->property_anim = 0;
	}
}

void MENU_AnimateString(Menu_HandleTypeDef *hmenu, SSD1306_HandleTypeDef *hssd, uint8_t *str, uint8_t anim_val, uint8_t anim_start) {
	if (hmenu->page_anim < anim_start) return;
	uint8_t min_len = strlen(str);
	if (min_len > anim_val - anim_start)
		min_len = anim_val - anim_start;

	SSD1306_DrawString(hssd, str, min_len);
}

// UTIL
float Lerp(float a, float b, float t) {
	if (t >= 1) return b;
	if (t <= 0) return a;
	return a*(1.0-t) + b*t;
}
