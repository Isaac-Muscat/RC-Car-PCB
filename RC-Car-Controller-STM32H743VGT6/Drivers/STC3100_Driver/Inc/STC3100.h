/*
 * STC3100.h
 *
 *  Created on: Jun 18, 2025
 *      Author: Loochis
 */

#ifndef STC3100_DRIVER_INC_STC3100_H_
#define STC3100_DRIVER_INC_STC3100_H_

#include "stm32h7xx_hal.h"

	// STRUCTS
    // ------------------------------------------------------------------------------------
	typedef struct
	{
		I2C_HandleTypeDef 	*i2c_handle;	// ptr to I2C_HandleTypeDef which interfaces with the SSD1306

		uint8_t				address;		// address of the SSD1306, usually 0x3C

	} STC3100_HandleTypeDef;

	// FUNCS
	// ------------------------------------------------------------------------------------

	uint8_t STC3100_SendCommand(STC3100_HandleTypeDef *hssd, uint8_t command);
	uint8_t STC3100_Init(STC3100_HandleTypeDef *hstc);
	uint8_t STC3100_Get(STC3100_HandleTypeDef *hstc);

#endif /* STC3100_DRIVER_INC_STC3100_H_ */
