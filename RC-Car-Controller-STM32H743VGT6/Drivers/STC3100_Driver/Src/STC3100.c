/*
 * STC3100.c
 *
 *  Created on: Jun 18, 2025
 *      Author: Loochis
 */

#include "STC3100.h"

uint8_t STC3100_ReadRegister(STC3100_HandleTypeDef *hstc, uint8_t reg, uint8_t *pData, uint8_t len) {
	// Start transfer, write reg addr as WRITE
	if (HAL_I2C_Master_Transmit(hstc->i2c_handle, (hstc->address) << 1, &reg, 1, 10))
		return ERROR;

	// Read 2 bytes in
	if (HAL_I2C_Master_Receive(hstc->i2c_handle, ((hstc->address) << 1) | 1, pData, len, 10))
		return ERROR;
}

uint8_t STC3100_Init(STC3100_HandleTypeDef *hstc) {
	return HAL_I2C_IsDeviceReady(hstc->i2c_handle, (hstc->address) << 1, 3, 100);
}

uint8_t STC3100_Get(STC3100_HandleTypeDef *hstc) {

	uint8_t counter[2] = {0, 0};
	uint8_t reg = 0;

	if (STC3100_ReadRegister(hstc, REG_COUNTER_LOW, counter, 1))
		return ERROR;

//	if (HAL_I2C_Master_Transmit(hstc->i2c_handle, (hstc->address) << 1, REG_CHARGE_HIGH, 1, 10))
//		return ERROR;
//	if (HAL_I2C_Master_Receive(hstc->i2c_handle, (hstc->address) << 1, &charge, 1, 10))
//		return ERROR;

	return SUCCESS;
}
