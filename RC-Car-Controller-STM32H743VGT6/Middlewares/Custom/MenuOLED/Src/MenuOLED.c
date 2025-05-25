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

	hmenu->pages[1].properties[0].name = AllocateString("LIGHTS 1");
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

	hmenu->pages[1].properties[1].name = AllocateString("LIGHTS 2");
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

	hmenu->pages[1].properties[2].name = AllocateString("LIGHTS 3");
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

	hmenu->pages[1].properties[3].name = AllocateString("LIGHTS 4");
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
	// Set the cursor to the top left
	hmenu->ssdL_handle->str_cursor = 26;
	SSD1306_DrawString(hmenu->ssdL_handle);
}
