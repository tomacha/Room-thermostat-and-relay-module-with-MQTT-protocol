/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "hm_bt4502.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
BT4502_t BT4502 = {
		.INT_GPIO = INT_GPIO_Port,
		.INT_GPIO_Pin = INT_Pin,
		.PDN_GPIO = PDN_GPIO_Port,
		.PDN_GPIO_Pin = PDN_Pin,
		.WAKEUP_GPIO = WAKEUP_GPIO_Port,
		.WAKEUP_GPIO_Pin = WAKEUP_Pin,
		.huart = &hlpuart1,
		.recv_buf = {0},
		.state = BT4502_State_Uninitialized,
		.auth_code = "1234"};

uint8_t rtc_reset = 1;
uint8_t bt_not_connected_count = 0;
uint8_t test = 1;

extern Ds18b20Sensor_t	ds18b20[_DS18B20_MAX_SENSORS];
float temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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
  MX_RTC_Init();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  BT4502_Init(&BT4502);
  if(!Ds18b20_Init())
  {
	  return -1;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	Ds18b20_ManualConvert();

	if(BT4502.state == BT4502_State_Advertising)
	{
		bt_not_connected_count++;
		if(bt_not_connected_count == 2)
		{
			//BT4502_Sleep(&BT4502);
			BT4502.state = BT4502_State_Data_Sent;
		}
	}

	if(BT4502.state == BT4502_State_Data_Received)
	{
		//bt_not_connected_count = 0;
		if(strcmp((char *)BT4502.recv_buf, "TTM:CONNECT\r\n") == 0)
		{
			BT4502.state = BT4502_State_Connected;
		}
		else if(strcmp((char *)BT4502.recv_buf, "TTM:DISCONNECT\r\n") == 0)
		{
			if(BT4502.state != BT4502_State_Sleep)
			{
				BT4502.state = BT4502_State_Advertising;
			}
		}
		else if(strcmp((char *)BT4502.recv_buf, "TTM:DISCONNECT FOR TIMEOUT\r\n") == 0)
		{
			if(BT4502.state != BT4502_State_Sleep)
			{
				BT4502.state = BT4502_State_Advertising;
			}
		}
		memset(BT4502.recv_buf, '\0', sizeof(BT4502.recv_buf));
  	}

	if(BT4502.state == BT4502_State_Connected)
	{
		if(ds18b20[0].DataIsValid)
		{
			HAL_Delay(1000);
			BT4502_Send_Data(&BT4502, &ds18b20[0].Temperature, sizeof(ds18b20[0].Temperature));
			HAL_Delay(1000);
		}
		else
		{
			BT4502.state = BT4502_State_Data_Sent;
		}
  	}

	if(BT4502.state == BT4502_State_Data_Sent)
	{
		bt_not_connected_count = 0;
		BT4502_Sleep(&BT4502);
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		rtc_reset = 1;
	}
	else if(BT4502.state == BT4502_State_Sleep)
	{
		BT4502_WakeUp(&BT4502);
	}

	if(rtc_reset == 1)
	{
		rtc_reset = 0;
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (10000-1), 32000/16); //10000-1 to 5s
	}
	HAL_SuspendTick();
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SystemClock_Config();
	HAL_ResumeTick();

	if(GPIO_Pin == BT4502.INT_GPIO_Pin)
	{
		BT4502_INT_Pin_Callback(&BT4502);
	}
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	SystemClock_Config();
	HAL_ResumeTick();

	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);

	rtc_reset = 1;
}
/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
