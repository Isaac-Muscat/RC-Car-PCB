/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "XBEE.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OV7670_ADDR_READ 0x43
#define OV7670_ADDR_WRITE 0x42

// CAMERA SETTINGS
#define CAM_WIDTH 158
#define CAM_HEIGHT 121
#define CAM_BYTES_PER_PIXEL 2

// JPEG SETTINGS
#define JPEG_QUALITY 20
#define JPEG_OUTBUF_SIZE 64
#define JPEG_HEADERSIZE 526

#define JPEG_MCU_WIDTH 6
#define JPEG_MCU_HEIGHT 9

#define CAM_GRAYSIZE CAM_WIDTH * CAM_HEIGHT
#define CAM_422SIZE CAM_WIDTH * CAM_HEIGHT * 2
#define CAM_444SIZE CAM_WIDTH * CAM_HEIGHT * 3

// UART SETTINGS
#define UART_TXSIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c2;

JPEG_HandleTypeDef hjpeg;
MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
MDMA_HandleTypeDef hmdma_jpeg_outfifo_ne;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_MDMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DCMI_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_JPEG_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void WriteDebug(uint8_t *str_ptr, uint8_t str_len);

HAL_StatusTypeDef CAM_GetRegister(uint8_t addr, uint8_t* pData, uint8_t haltOnError);
HAL_StatusTypeDef CAM_SetRegister(uint8_t addr, uint8_t data, uint8_t haltOnError);
uint8_t GenerateJPEGMCUBlock();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// USB VARIABLES
uint8_t usb_msg[100] = {0};	// Reserve 100 bytes for USB debug messages
uint8_t usb_device_rxFlag = 0x00; // Extern in usbd_cdc_if.h

// CAMERA VARIABLES
uint8_t camera_mem[CAM_GRAYSIZE];	// Reserve u8 array for camera DMA transfer
volatile uint8_t camera_state = 0;			// Camera state variable
// ----------------------------------- 0: Camera idle
// ----------------------------------- 1: Camera DMA Queued
// ----------------------------------- 2: Camera capturing

// JPEG VARIABLES
uint8_t jpeg_mcu[64];				// Reserve u8 array for JPEG MCU block
uint32_t jpeg_block = 0;			// Current JPEG MCU block being processed

uint8_t jpeg_out[CAM_GRAYSIZE];		// Reserve u8 array for JPEG result
volatile uint8_t jpeg_ready = 1;				// JPEG status (ready to start?)
volatile uint32_t jpeg_size = 0;				// Current size of the JPEG result

// XBEE VARIABLES
XBEE_HandleTypeDef hxbee;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_DCMI_Init();
  MX_I2C2_Init();
  MX_TIM14_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_JPEG_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

	// ------------------------------------------------------------ SETUP USB MESSAGING -- //
	//uint8_t usb_msg[100] = {0};	// Reserve 100 bytes for USB Debug messages

	// ------------------------------------------------------------ SETUP CAMERA INTERFACE -- //
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);	// XCLK - Start the camera's core clock

	// SET CAMERA REGISTERS
	// See: https://web.mit.edu/6.111/www/f2016/tools/OV7670_2006.pdf

	// (0x0C) COM3  [3] = 1 (Enable Scaling)

	// (0x12) COM7  [4] = 1 (QVGA Resolution)
	//	    COM7  [2] = 0 (YUV Format)
	//        COM7  [0] = 0 (YUV Format)

	// (0x17) HSTART
	// (0x18) HSTOP

	// (0x32) HREF [7:6] = 10  (Default value)
	//        HREF [2:0] = 110 (increase LSB of HSTART by 6 to get rid of black bar)

	// UNUSED

	// X (0x15) COM10 [6] = 1 (Switch HREF to HSYNC)

	// X (0x40) COM15 [7] = 1 (Data range [00-FF])
	// X 		  COM15 [6] = 1 (Data range [00-FF])
	// X 		  COM15 [5] = 0 (RGB 565)
	// X 		  COM15 [4] = 1 (RGB 565)
	uint8_t cam_regCache;

	cam_regCache = 0b00001000;
	while (CAM_SetRegister(0x0C, cam_regCache, 0)) {}

	cam_regCache = 0b00010000;
	while (CAM_SetRegister(0x12, cam_regCache, 0)) {}

	cam_regCache = 0b10000110;
	while (CAM_SetRegister(0x32, cam_regCache, 0)) {}

	//cam_regCache = 0b01000000;
	//while (CAM_SetRegister(0x15, cam_regCache, 0)) {}

	//cam_regCache = 0b11010000;
	//while (CAM_SetRegister(0x40, cam_regCache, 0)) {}

	// ------------------------------------------------------------ SETUP JPEG ENCODING -- //
	// Set the CONFIG
	JPEG_ConfTypeDef* jpeg_config;
	jpeg_config->ColorSpace = JPEG_GRAYSCALE_COLORSPACE;
	//jpeg_config->ColorSpace = JPEG_YCBCR_COLORSPACE;
	//jpeg_config->ChromaSubsampling = JPEG_422_SUBSAMPLING;
	jpeg_config->ImageWidth = JPEG_MCU_WIDTH*8;
	jpeg_config->ImageHeight = JPEG_MCU_HEIGHT*8;
	jpeg_config->ImageQuality = JPEG_QUALITY;
	HAL_JPEG_ConfigEncoding(&hjpeg, jpeg_config);

	// ------------------------------------------------------------ SETUP XBEE -- //
	hxbee.uart_handle = &huart1;
	hxbee.pktRx_max = 5;
	hxbee.pktTx_max = 5;

	if (XBEE_Init(&hxbee)) {
		sprintf(usb_msg, " Failed to Init XBEE");
		WriteDebug(usb_msg, strlen(usb_msg));
		while (1) { }
	}

	//TODO: Implement all JPEG Callbacks
	//TODO: Try to Interleave CAM/JPEG DMAs using JPEG GetDataCallback

	// SETUP MOTOR
	//TIM2->CCR1 = 0;
	//TIM2->CCR2 = 400;

	//TIM4->CCR4 = 0;
	//TIM4->CCR3 = 400;
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // LEFT_PWM_1
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // RIGHT_PWM_1

	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // LEFT_PWM_2
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // RIGHT_PWM_2

	// Setup lights
	//  TIM1->CCR4 = 1000;
	//  TIM3->CCR4 = 1000;
	//  TIM3->CCR3 = 1000;
	//  TIM2->CCR4 = 1000;
	//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // LIGHTS_PWM_1
	//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // LIGHTS_PWM_2
	//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // LIGHTS_PWM_3
	//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // LIGHTS_PWM_4

	// Delay for goofiness
	//HAL_Delay(3000);

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // Motor_en

	// XBEE READ MODE
//	uint8_t uart_xbee_buffer[256] = {0};	// Get a reference to the start of buffer data
//	uint8_t uart_xbee_len = 0;			// Get a reference to the start of buffer data
//	while (1) {
//		  HAL_UARTEx_ReceiveToIdle(&huart1, uart_xbee_buffer, 256, &uart_xbee_len, 1000);
//		  if (uart_xbee_len > 0) {
//			  CDC_Transmit_FS(uart_xbee_buffer, uart_xbee_len);
//			  uart_xbee_len = 0;
//			  memset(uart_xbee_buffer, 0x00, 256);
//		  } else {
//			  //sprintf(usb_msg, "err\r\n");
//			  //HAL_UART_Transmit(&huart1, usb_msg, strlen(usb_msg), 1000);
//		  }
//	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// Take a snapshot
		camera_state = 1;	// Flag Camera as DMA Queued
		HAL_StatusTypeDef ovStat = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, camera_mem, CAM_GRAYSIZE / 4);
		if (ovStat) {
			//while (1) {
			sprintf(usb_msg, "DCMI/DMA ERROR - Code 0x%X\r\n", ovStat);
			CDC_Transmit_FS(usb_msg, strlen(usb_msg));
			HAL_Delay(1000);
			//}
		}

		// Wait until the camera is idle again
		while (camera_state) { }

		// Start the JPEG Encode
		jpeg_ready = 0;
		jpeg_block = 0;
		jpeg_size = 0;

		GenerateJPEGMCUBlock();
		HAL_JPEG_Encode_DMA(&hjpeg, jpeg_mcu, 64, jpeg_out, JPEG_OUTBUF_SIZE);

		// Wait until the JPEG is done frying
		while (!jpeg_ready) { }

//		sprintf(usb_msg, "Begin Transmission\r\n");
//		CDC_Transmit_FS(usb_msg, strlen(usb_msg));

		// Transmit the camera data
		for (uint16_t i = 0; i < ((jpeg_size - JPEG_HEADERSIZE) / UART_TXSIZE) + 1; i++) {
//			uart_txRaw_buffer[0] = 0b10101010;
//			uart_txRaw_buffer[1] = i >> 8;	// Store MSB of idx in header
//			uart_txRaw_buffer[2] = i & 0x00FF;	// Store LSB of idx in header
//			uint8_t checksum = 0;
//			for (uint8_t z = 0; z < UART_TXSIZE; z++) {
//				checksum += jpeg_out[i*UART_TXSIZE + JPEG_HEADERSIZE];
//			}
//			uart_txRaw_buffer[3] = 0x00;
//			memcpy(uart_tx_buffer, jpeg_out + i*UART_TXSIZE + JPEG_HEADERSIZE, UART_TXSIZE);	// Copy jpeg vram into the TX buffer data segment
//			HAL_UART_Transmit(&huart1, uart_txRaw_buffer, UART_TXSIZE + 4, 30);	// Transmit the buffer
			XBEE_TXPacket(&hxbee, jpeg_out + i*UART_TXSIZE + JPEG_HEADERSIZE, i);
			// Debug
//			if (i % 10 == 0) {
//				sprintf(usb_msg, "0x%X\r\n", i);
//				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
//			}
			HAL_Delay(5);
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALTERNATE_4;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_EVEN;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALTERNATE_2;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00C0EAFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
static void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 5;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 111111;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * Enable MDMA controller clock
  */
static void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ------------------------------------------------------------ OVERRIDE CALLBACKS -- //
// Frame captured event, called when the DMA buffer is full of new frame data
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi) {
	if (!camera_state) return;	// Do nothing if the camera is IDLE

	if (camera_state == 1) {	// Transition flag to CAPTURING
		camera_state = 2;
		return;
	}

	if (camera_state == 2) {	// Transition flag to IDLE
		HAL_DCMI_Stop(hdcmi);
		camera_state = 0;
		return;
	}
}

// JPEG hardware is requesting the next MCU block
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData) {
	//	if (jpeg_block % 64 == 0) {
	//		uint8_t usb_msg[100] = {0};
	//		sprintf(usb_msg, "JPEG: Block: %d\r\n", jpeg_block);
	//		CDC_Transmit_FS(usb_msg, strlen(usb_msg));
	//	}

	// Restock the input buffer with new MCU
	if (GenerateJPEGMCUBlock()) {
		// ERROR while generating MCU, probable memory leak - recover JPEG peripheral by restarting
		// Reset JPEG variables
		jpeg_ready = 1;
		jpeg_block = 0;
	}
	else {
		// Configure the buffer to be the same as before, it's contents have changed
		HAL_JPEG_ConfigInputBuffer(hjpeg, jpeg_mcu, 64);
	}
}

// JPEG hardware is requesting a larger output buffer.
// This is done to manage JPEG output size, give it a buffer only as large as required
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength) {
	// Setup new output buffer location (part of contiguous super-buffer)
	jpeg_size += JPEG_OUTBUF_SIZE;
	HAL_JPEG_ConfigOutputBuffer(hjpeg, jpeg_out + jpeg_size, JPEG_OUTBUF_SIZE);
	// Debug MSG
	//	sprintf(usb_msg, "JPEG: DataReady - output: %d\r\n", jpeg_size);
	//	CDC_Transmit_FS(usb_msg, strlen(usb_msg));
}

// JPEG hardware has completed the current image
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg) {
	// Reset JPEG variables
	jpeg_ready = 1;
	jpeg_block = 0;
	// Debug MSG
	//	sprintf(usb_msg, "JPEG: Finished encode\r\n");
	//	CDC_Transmit_FS(usb_msg, strlen(usb_msg));
}

// ------------------------------------------------------------ UTILITY FUNCTIONS -- //
// Transmit wrapper, this is for continuity across Controller project which debugs by printing to OLEDs
void WriteDebug(uint8_t *str_ptr, uint8_t str_len) {
	CDC_Transmit_FS(str_ptr, str_len);
}

HAL_StatusTypeDef CAM_GetRegister(uint8_t addr, uint8_t* pData, uint8_t haltOnError) {
	HAL_StatusTypeDef ov_result;
	uint8_t usb_msg[100] = {0};

	ov_result = HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR_READ, &addr, 1, 100);
	if (ov_result) {
		if (haltOnError) {
			sprintf(usb_msg, "TX ERROR: Cannot read camera register 0x%X - Code 0x%X\r\n", addr, ov_result);
			while (1) {
				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
				HAL_Delay(1000);
			}
		}
		return ov_result;
	}


	ov_result = HAL_I2C_Master_Receive(&hi2c2, OV7670_ADDR_READ, pData, 1, 100);
	if (ov_result) {
		if (haltOnError) {
			sprintf(usb_msg, "RX ERROR: Cannot read camera register 0x%X - Code 0x%X\r\n", addr, ov_result);
			while (1) {
				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
				HAL_Delay(1000);
			}
		}
		return ov_result;
	}


	return HAL_OK;
}

HAL_StatusTypeDef CAM_SetRegister(uint8_t addr, uint8_t data, uint8_t haltOnError) {
	HAL_StatusTypeDef ov_result;
	uint8_t usb_msg[100] = {0};

	uint8_t reg_set[2] = {addr, data};

	ov_result = HAL_I2C_Master_Transmit(&hi2c2, OV7670_ADDR_WRITE, reg_set, 2, 100);
	if (ov_result) {
		if (haltOnError) {
			sprintf(usb_msg, "TX ERROR: Cannot write camera register 0x%X - Code 0x%X\r\n", addr, ov_result);
			while (1) {
				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
				HAL_Delay(1000);
			}
		}
		return ov_result;
	}

	// Confirm write
	uint8_t reg_get = 0x00;
	ov_result = CAM_GetRegister(addr, &reg_get, 1);
	if (ov_result) {
		if (haltOnError) {
			sprintf(usb_msg, "CF ERROR: Cannot confirm camera register 0x%X - Code 0x%X\r\n", addr, ov_result);
			while (1) {
				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
				HAL_Delay(1000);
			}
		}
		else {
			return ov_result;
		}
	}

	if (reg_get != data) {
		// Error, bad write
		if (haltOnError) {
			sprintf(usb_msg, "CF ERROR: Bad write to register 0x%X - EXPECTED 0x%X, GOT 0x%X\r\n", addr, data, reg_get);
			while (1) {
				CDC_Transmit_FS(usb_msg, strlen(usb_msg));
				HAL_Delay(1000);
			}
		}
		return HAL_ERROR;
	}

	return HAL_OK;
}

// GENERATE JPEG MCU BLOCK
uint8_t GenerateJPEGMCUBlock() {
	if (jpeg_block >= JPEG_MCU_WIDTH*JPEG_MCU_HEIGHT) { return 1; }
	int xStart = (jpeg_block % JPEG_MCU_WIDTH) * 8;
	int yStart = (jpeg_block / JPEG_MCU_WIDTH) * 8;
	int i = 0;
	for (int y = yStart; y < yStart + 8; y++) {
		for (int x = xStart; x < xStart + 8; x++) {
			// Pad to 8x8
			if ((x*3.33) >= CAM_WIDTH) {
				jpeg_mcu[i] = 0x00;
			} else {
				jpeg_mcu[i] = camera_mem[(uint16_t)(x*3.33) + (uint16_t)(y*2) * CAM_WIDTH];
			}
			i++;
		}
	}
	jpeg_block++;
	return 0;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
