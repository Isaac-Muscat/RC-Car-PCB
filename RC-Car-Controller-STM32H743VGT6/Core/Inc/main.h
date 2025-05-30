/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI4_DC_Pin GPIO_PIN_5
#define SPI4_DC_GPIO_Port GPIOE
#define BTN_LB_Pin GPIO_PIN_5
#define BTN_LB_GPIO_Port GPIOA
#define WIPE_L_Pin GPIO_PIN_6
#define WIPE_L_GPIO_Port GPIOA
#define WIPE_R_Pin GPIO_PIN_4
#define WIPE_R_GPIO_Port GPIOC
#define BTN_RB_Pin GPIO_PIN_5
#define BTN_RB_GPIO_Port GPIOC
#define BTN_LF_Pin GPIO_PIN_8
#define BTN_LF_GPIO_Port GPIOC
#define BTN_RF_Pin GPIO_PIN_9
#define BTN_RF_GPIO_Port GPIOC
#define SPI4_RST_Pin GPIO_PIN_10
#define SPI4_RST_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
