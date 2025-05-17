/*
 * ST7789.h
 *
 *  Created on: May 17, 2025
 *      Author: Loochis
 */

#ifndef ST7789_DRIVER_INC_ST7789_H_
#define ST7789_DRIVER_INC_ST7789_H_

#include "stm32h7xx_hal.h"

#define LCD_WIDTH   240 //LCD width
#define LCD_HEIGHT  320 //LCD height

// STRUCTS
// ------------------------------------------------------------------------------------
typedef struct
{
	SPI_HandleTypeDef 	*spi_handle;	// ptr to I2C_HandleTypeDef which interfaces with the ST7789

	GPIO_TypeDef		*cs_gpio_handle;// ptr to GPIO handle of CS line

	uint16_t			cs_gpio_pin;	// GPIO pin no. of CS line

	GPIO_TypeDef		*dc_gpio_handle;// ptr to GPIO handle of DC line

	uint16_t			dc_gpio_pin;	// GPIO pin no. of DC line

	uint16_t			*vram;			// ptr to the MCU side copy of ST7789 VRAM

} ST7789_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

	uint8_t ST7789_SendByte_Command(ST7789_HandleTypeDef *hst7789, uint8_t command);
	uint8_t ST7789_SendByte_Data(ST7789_HandleTypeDef *hst7789, uint8_t data);
	uint8_t ST7789_SendWord_Data(ST7789_HandleTypeDef *hst7789, uint16_t data);
	uint8_t ST7789_Init(ST7789_HandleTypeDef *hst7789);

	void ST7789_SetCursor(ST7789_HandleTypeDef *hst7789, uint16_t x, uint16_t y);
	void ST7789_SetWindow(ST7789_HandleTypeDef *hst7789, uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t  yEnd);
	uint8_t ST7789_Clear(ST7789_HandleTypeDef *hst7789, uint8_t col);
	uint8_t ST7789_Update(ST7789_HandleTypeDef *hst7789);


#endif /* ST7789_DRIVER_INC_ST7789_H_ */
