/*
 * ST7789.c
 *
 *  Created on: June 10, 2025
 *      Author: Loochis
 */

#include "../Inc/INA229.h"

// INITIALIZATION COMMAND LIST
// ------------------------------------------------------------------------------------

// FUNCTION IMPLEMENTEATIONS
// ------------------------------------------------------------------------------------

uint8_t INA229_ReadRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint8_t *pRead, uint8_t len) {
	// A5 A4 A3 A2 A1 A0 XX RR
	uint8_t dataToWrite = (addr << 2) | 0b1;
	uint8_t ret = 0;

	//uint8_t rVal[2] = {0, 0};

	// Assert the CS low
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_RESET);

	HAL_Delay(1);

	ret = HAL_SPI_Transmit(ina229->spi_handle, &dataToWrite, 1, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return ERROR;
	}

	ret = HAL_SPI_Receive(ina229->spi_handle, pRead, len, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return  ERROR;
	}

//	ret = HAL_SPI_TransmitReceive(ina229->spi_handle, &dataToWrite, rVal, 3, 100);
//	if (ret) {
//		// Release the CS
//		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
//		return ERROR;
//	}

	// Release the CS
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);

	return SUCCESS;
}


uint8_t INA229_WriteRegister(INA229_HandleTypeDef *ina229, uint8_t addr, uint16_t write, uint8_t *pRead) {
	// A5 A4 A3 A2 A1 A0 XX WW DF DE DD DC DB DA D9 D8 7 D6 D5 D4 D3 D2 D1 D0
	uint8_t dataToWrite[3] = {addr << 2, write >> 8, write};
	uint8_t ret = 0;

	// Assert the CS low
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_RESET);
	ret = HAL_SPI_Transmit(ina229->spi_handle, &dataToWrite, 3, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return ERROR;
	}


	ret = HAL_SPI_Receive(ina229->spi_handle, pRead, 16, 100);
	if (ret) {
		// Release the CS
		HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);
		return  ERROR;
	}

	// Release the CS
	HAL_GPIO_WritePin(ina229->cs_gpio_handle, ina229->cs_gpio_pin, GPIO_PIN_SET);

	return SUCCESS;
}

uint8_t INA229_Init(INA229_HandleTypeDef *ina229) {

	// Wake up the SPI line
//	uint8_t dummy = 0x00;
//	HAL_SPI_Transmit(ina229->spi_handle, &dummy, 1, 100);
//	HAL_Delay(10);

	// Let's just test to start
	uint8_t readVal[2];
	INA229_ReadRegister(ina229, 0x01, readVal, 2);

	return SUCCESS;
}
