/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_subghz_phy.h"
#include "usart.h"
#include "sys_app.h"
#include "sht30_sensor.h"
#include "max44009_sensor.h"
#include "kalman_filter.h"
#include "rd_ccs811.h"
#include "rd_flash.h"
#include "subghz_phy_app.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define    MODE_I2C1		 1
#define    MODE_I2C3     2

#define MODE_I2C   MODE_I2C3
extern unsigned char radio_rx_flag;
extern unsigned char radio_tx_flag;
extern unsigned short rd_device_id;
extern unsigned char rd_netkey_char[16];
extern uint32_t rd_net_key_32bit[4];
extern unsigned char network_status;
extern unsigned int rd_tx_opcode;
extern unsigned char add_device_flag;
extern uint32_t dev_key_32;
extern uint32_t random_time;
unsigned int trick_time = 0;
unsigned char listen_fb_talk_flag = 0;
unsigned char radio_free_flag = 0;

unsigned char device_send_message_cnt = 0;
unsigned char device_send_flag = 0;
unsigned int sht30_temp = 0;
unsigned int sht30_hum = 0;
unsigned int ccs811_co2 = 0;
unsigned int ccs811_tvoc = 0;
unsigned char ccs811_current = 0;
unsigned char set_up_co2 = 0;
unsigned char check_one_time = 0;
unsigned char device_send_flag_old,device_send_flag_new;


/* USER CODE END Includ\es */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
unsigned char cnt = 0;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

uint32_t data_max44009 = 0;
rd_ccs811_data_t CO2_data;






									
									



uint32_t check_random = 0;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void RD_button_detect(unsigned char *cnt);
 void RD_GPIO_Init(void);
void RD_UART_MspInit(UART_HandleTypeDef* uartHandle);
uint32_t RD_ADC_read();
unsigned char net_key [16] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
 void RD_MX_I2C1_Init(void);
 void RD_MX_I2C3_Init(void);
 void RD_ADC_Init(void);
 void RD_LED_Init(void);

//static void RD_RNG_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern unsigned char radio_rx_flag;
ADC_HandleTypeDef hadc;
uint32_t ground_hum_data = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
unsigned char  save_flash_flag = 0;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_SubGHz_Phy_Init();
	
  /* USER CODE BEGIN 2 */
	unsigned int check_i2c = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */  

		MX_SubGHz_Phy_Process();
	  

		
		if(check_one_time == 0){

			check_one_time = 1;
			device_gen_dev_key(&dev_key_32);
			uint64_t *Rd_flash_read_ptr = (uint64_t *)rd_net_key_32bit;
			if(*Rd_flash_read_ptr == 0){
				//_save_netkey_device_id(rd_net_key_32bit,&rd_device_id,&dev_key_32);
				Rd_flash_read_ptr = (uint64_t *)RD_FLASH_ADDR;
				if(*Rd_flash_read_ptr == 0 || *Rd_flash_read_ptr == 0xffffffffffffffff){
					rd_tx_opcode = OP_JOIN_REQUEST;
					add_device_flag = 1;
					network_status = NOT_JOINT;
				}
				else {
					read_key_device_id(rd_netkey_char,&rd_device_id,&dev_key_32,&random_time);
					rd_tx_opcode = RD_OPCODE_SEND;
					add_device_flag = 0;
					network_status = JOINED;
				}
			}
		}
		
		if(save_flash_flag == 1){
			save_flash_flag = 0;
			
			
			device_gen_random_time(&random_time);
			_save_netkey_device_id (rd_netkey_char,&rd_device_id,&dev_key_32,&random_time);
		}
		
		device_send_message_cnt ++;
		if(device_send_message_cnt >= 50){
			
			RD_MX_I2C3_Init();
			HAL_I2C_MspInit(&hi2c3);
			sht30_data_tdef sht30_data;
			sht30_data = RD_sht30_caculate_data (sht30_read_i2c_data());
			sht30_temp = (unsigned int)(sht30_data.temperature*100);
			sht30_hum = (unsigned int)(sht30_data.humitidy*10);
			
			device_send_message_cnt = 0;
			device_send_flag = 1;
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

  /** Configure LSE Drive Capability
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void RD_button_detect(unsigned char *cnt){
    RD_GPIO_Init();
    if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){

    	while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
    		HAL_Delay(20);
    	}
    	*cnt = 1 ;
    	radio_tx_flag = 1;
    }
}

 void RD_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : button_trigger_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


 void RD_LED_Init(void)
{
	
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	
}

#if (MODE_I2C == MODE_I2C3)
 void RD_MX_I2C3_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x20303E5D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    C1     ------> I2C3_SDA
    PC0     ------> I2C3_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

#elif(MODE_I2C == MODE_I2C1)
 void RD_MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

#endif

 void RD_ADC_Init(void)
{
  /* USER CODE BEGIN ADC_Init 0 */
	  ADC_ChannelConfTypeDef sConfig = {0};
  /* USER CODE END ADC_Init 0 */
  /* USER CODE BEGIN ADC_Init 1 */
  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}


void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hadc->Instance==ADC)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC GPIO Configuration
    PB3     ------> ADC INPUT
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = ;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

uint32_t RD_ADC_read(){
	uint32_t Adc_data = 0;
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 50);
	Adc_data = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	return  Adc_data;
}


//static void RD_RNG_Init(void)
//{

//  /* USER CODE BEGIN RNG_Init 0 */

//  /* USER CODE END RNG_Init 0 */

//  /* USER CODE BEGIN RNG_Init 1 */

//  /* USER CODE END RNG_Init 1 */
//  hrng.Instance = RNG;
//  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
//  if (HAL_RNG_Init(&hrng) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RNG_Init 2 */

//  /* USER CODE END RNG_Init 2 */

//}

//void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
//{
//  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
//  if(hrng->Instance==RNG)
//  {
//  /* USER CODE BEGIN RNG_MspInit 0 */


//  /* USER CODE END RNG_MspInit 0 */
//  /** Initializes the peripherals clocks
//  */
//    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
//    PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
//    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
//    {
//      Error_Handler();
//    }

//    /* Peripheral clock enable */
//    __HAL_RCC_RNG_CLK_ENABLE();
//  /* USER CODE BEGIN RNG_MspInit 1 */

//  /* USER CODE END RNG_MspInit 1 */
//  }

//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1){

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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
