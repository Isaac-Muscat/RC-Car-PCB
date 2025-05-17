/*
 * ST7789.c
 *
 *  Created on: May 17, 2025
 *      Author: Loochis
 */

#include "ST7789.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------
// Im gonna keep it a buck, I have no idea what any of this does
// See: https://www.waveshare.com/wiki/2inch_LCD_Module#Software_description
// See: https://www.mouser.com/datasheet/2/744/ST7789VW-2320339.pdf
const uint8_t ST7789_INITCMDS[] = {
	19,                                                       // number of initializers
	1,  0x36, 0x00,
	1,  0x3A, 0x05,
	0,  0x21,
	4,  0x2A, 0x00, 0x00, 0x01, 0x3F,
	4,  0x2B, 0x00, 0x00, 0x00, 0xEF,
	5,  0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33,
	1,  0xB7, 0x35,
	1,  0xBB, 0x1F,
	1,  0xC0, 0x2C,
	1,  0xC2, 0x01,
	1,  0xC3, 0x12,
	1,  0xC4, 0x20,
	1,  0xC6, 0x0F,
	2,  0xD0, 0xA4, 0xA1,
	14, 0xE0, 0xD0, 0x08, 0x11, 0x08, 0x0C,
	          0x15, 0x39, 0x33, 0x50, 0x36,
			  0x13, 0x14, 0x29, 0x2D,
	14, 0xE1, 0xD0, 0x08, 0x10, 0x08, 0x06,
	          0x06, 0x39, 0x44, 0x51, 0x0B,
			  0x16, 0x14, 0x2F, 0x31,
	0,  0x21,
	0,  0x11,
	0,  0x29
};

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

// Writes a single command byte to the LCD
uint8_t ST7789_SendByte_Command(ST7789_HandleTypeDef *hst7789, uint8_t command) {
	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_RESET);	// assert CS LO
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_RESET);	// assert DC LO (~CMD)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, &command, 1, 500))
		return ERROR;
	return SUCCESS;

	// Don't reassert CS (This is how it's done in the example code)
	// See: https://www.waveshare.com/wiki/2inch_LCD_Module#Software_description
}

uint8_t ST7789_SendByte_Data(ST7789_HandleTypeDef *hst7789, uint8_t data) {
	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_RESET);	// assert CS LO
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);		// assert DC HI (DATA)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, &data, 1, 500))
		return ERROR;

	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_SET);	// assert CS HI
	return SUCCESS;
}

uint8_t ST7789_SendWord_Data(ST7789_HandleTypeDef *hst7789, uint16_t data) {
	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_RESET);	// assert CS LO
	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);		// assert DC HI (DATA)

	// Write the data
	if (HAL_SPI_Transmit(hst7789->spi_handle, (uint8_t*)(&data), 2, 500))
		return ERROR;

	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_SET);	// assert CS HI
	return SUCCESS;
}

uint8_t ST7789_Init(ST7789_HandleTypeDef *hst7789) {

	// Some control variables
	uint16_t n_commands = ST7789_INITCMDS[0];
	uint16_t n_arguments;
	uint16_t cmd_idx = 1;

	// Read the init sequence
	while (n_commands--) {
		// Get no. of Args
	    n_arguments = ST7789_INITCMDS[cmd_idx];
	    cmd_idx++;

	    // Send initial command
	    if (ST7789_SendByte_Command(hst7789, ST7789_INITCMDS[cmd_idx])) return cmd_idx;
	    cmd_idx++;

	    // Send argumemts
	    while (n_arguments--) {
	    if (ST7789_SendByte_Data(hst7789, ST7789_INITCMDS[cmd_idx])) return cmd_idx;
	        cmd_idx++;
	    }
	}
	return SUCCESS;
}

// Set the cursor
void ST7789_SetCursor(ST7789_HandleTypeDef *hst7789, uint16_t x, uint16_t y) {
	ST7789_SendByte_Command(hst7789, 0x2a);
	ST7789_SendByte_Data(hst7789, x >> 8);
	ST7789_SendByte_Data(hst7789, x);
	ST7789_SendByte_Data(hst7789, x >> 8);
	ST7789_SendByte_Data(hst7789, x);

	ST7789_SendByte_Command(hst7789, 0x2b);
	ST7789_SendByte_Data(hst7789, y >> 8);
	ST7789_SendByte_Data(hst7789, y);
	ST7789_SendByte_Data(hst7789, y >> 8);
	ST7789_SendByte_Data(hst7789, y);

	ST7789_SendByte_Command(hst7789, 0x2C);
}

// Sets the "window"?
void ST7789_SetWindow(ST7789_HandleTypeDef *hst7789, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t  yEnd) {
	ST7789_SendByte_Command(hst7789, 0x2a);
	ST7789_SendByte_Data(hst7789, xStart >>8);
	ST7789_SendByte_Data(hst7789, xStart & 0xff);
	ST7789_SendByte_Data(hst7789, (xEnd - 1) >> 8);
	ST7789_SendByte_Data(hst7789, (xEnd - 1) & 0xff);

	ST7789_SendByte_Command(hst7789, 0x2b);
	ST7789_SendByte_Data(hst7789, yStart >>8);
	ST7789_SendByte_Data(hst7789, yStart & 0xff);
	ST7789_SendByte_Data(hst7789, (yEnd - 1) >> 8);
	ST7789_SendByte_Data(hst7789, (yEnd - 1) & 0xff);

	ST7789_SendByte_Command(hst7789, 0x2C);
}


uint8_t ST7789_Clear(ST7789_HandleTypeDef *hst7789, uint8_t col) {
	// fill VRAM with white
	memset(hst7789->vram, col, LCD_WIDTH*LCD_HEIGHT*4);

	ST7789_SetWindow(hst7789, 0, 0, LCD_WIDTH, LCD_HEIGHT);

	HAL_GPIO_WritePin(hst7789->dc_gpio_handle, hst7789->dc_gpio_pin, GPIO_PIN_SET);	// assert DC HI (~CMD)
	HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_RESET);	// assert CS LO

	// Push the screen
	//HAL_SPI_Transmit_DMA(hst7789->spi_handle, hst7789->vram, LCD_WIDTH*LCD_HEIGHT*2);
	//for(uint16_t i = 0; i < LCD_HEIGHT/64; i++){
		//HAL_SPI_Transmit(hst7789->spi_handle, hst7789->vram + i*LCD_WIDTH*8, LCD_WIDTH*8, 1000);
	HAL_SPI_Transmit_DMA(hst7789->spi_handle, hst7789->vram, 0xFFFF);
		//HAL_Delay(10);
	 //}

	//HAL_GPIO_WritePin(hst7789->cs_gpio_handle, hst7789->cs_gpio_pin, GPIO_PIN_SET);	// assert CS HI
}
