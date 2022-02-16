/**
  ******************************************************************************
  * @file    Examples_LL/RTC/RTC_Calendar/Src/main.c
  * @author  MCD Application Team
  * @brief   This sample code shows how to use STM32F4xx RTC LL API to configure
  *          Time and Date.
  *          Peripheral initialization done using LL unitary services functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */


#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(USE_RTC)

//#define DEBUG_OUT 1
#include "../../core/debug_out.h"

#include "../../MarlinCore.h"

#include "stm32f4xx_hal.h"
//#include "stm32f4xx_nucleo.h"
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Defines related to Clock configuration */
/* Uncomment to enable the adaquate Clock Source */
#define RTC_CLOCK_SOURCE_LSE
//#define RTC_CLOCK_SOURCE_LSI

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0xF9
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* RTC handler declaration */
RTC_HandleTypeDef RtcHandle = {0};

/* Buffer used for displaying Time */
uint8_t aShowTime[50] = {0};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TM_RTC_Config() {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/* We are updating RTC clock */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;

	/* Do not use PLL */
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	/* LSI is used as RTC clock */
	#if defined(RTC_CLOCK_SOURCE_LSI)
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
		RCC_OscInitStruct.LSIState = RCC_LSI_ON;

		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	#else
		/* LSE is used */
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
		RCC_OscInitStruct.LSEState = RCC_LSE_ON;

		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	#endif

	/* Config oscillator */
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Select peripheral clock */
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	/* Enable RTC Clock */
	__HAL_RCC_RTC_ENABLE();
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
   __HAL_RCC_RTC_DISABLE();
}

#if defined(STM32F7xx)
#define RTC_STATUS_REG      			RTC_BKP_DR31 /* Status Register */
#else
#define RTC_STATUS_REG      			RTC_BKP_DR19 /* Status Register */
#endif
#define RTC_STATUS_INIT_OK              0x1234       /* RTC initialised */
#define RTC_STATUS_TIME_OK              0x4321       /* RTC time OK */
#define	RTC_STATUS_ZERO                 0x0000

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void rtc_init(int force)
{
  DEBUG_ECHOLNPAIR("echo: rtc_init:275 force = ", force);

  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  /* Enable PWR peripheral clock */
	__HAL_RCC_PWR_CLK_ENABLE();

	/* Allow access to BKP Domain */
	HAL_PWR_EnableBkUpAccess();

  /* Get RTC status */
  uint32_t status = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_STATUS_REG);

  if (status == RTC_STATUS_TIME_OK) {
		/* Start internal clock if we choose internal clock */
		#if defined(RTC_CLOCK_SOURCE_LSI)
			TM_RTC_Config();
		#endif

    DEBUG_ECHOLNPAIR("echo: rtc_init:310 status = ", status);

		/* Wait for RTC APB registers synchronisation (needed after start-up from Reset) */
		HAL_RTC_WaitForSynchro(&RtcHandle);

		/* Clear reset flags */
		__HAL_RCC_CLEAR_RESET_FLAGS();

		/* Return OK */
		// return 1;
  }
  else {
		/* Start RTC clock */
		TM_RTC_Config();

		/* Init RTC */
		HAL_StatusTypeDef res = HAL_RTC_Init(&RtcHandle);

    RTC_DateTypeDef RTC_DateStruct = {0};
    RTC_TimeTypeDef RTC_TimeStruct = {0};

		/* Set date */
		RTC_DateStruct.Year = 0x01;
		RTC_DateStruct.Month = 0x02;
		RTC_DateStruct.Date = 0x03;
		RTC_DateStruct.WeekDay = RTC_WEEKDAY_THURSDAY;

		/* Set date */
		HAL_RTC_SetDate(&RtcHandle, &RTC_DateStruct, RTC_FORMAT_BCD);

		/* Set time */
		RTC_TimeStruct.Hours = 0x04;
		RTC_TimeStruct.Minutes = 0x05;
		RTC_TimeStruct.Seconds = 0x06;
		RTC_TimeStruct.TimeFormat = RTC_HOURFORMAT_24;
		RTC_TimeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		RTC_TimeStruct.StoreOperation = RTC_STOREOPERATION_RESET;

		/* Set time */
		HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BCD);
    UNUSED(res);


    DEBUG_ECHOLNPAIR("echo: rtc_init:352 res = ", res);

		/* Save data to backup regiser */
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_STATUS_REG, RTC_STATUS_TIME_OK);

		/* RTC was initialized now */
		// return 0;
  }

  DEBUG_ECHOLNPAIR("echo: rtc_init: RCC->BDCR = ", RCC->BDCR);
  DEBUG_ECHOLNPAIR("echo: rtc_init: RTC_STATUS_REG = ", HAL_RTCEx_BKUPRead(&RtcHandle, RTC_STATUS_REG));

  DEBUG_ECHOLN("rtc_init done");
}

void rtc_set_date(uint8_t year, uint8_t month, uint8_t day, uint8_t weekday) {

  RTC_DateTypeDef sdatestructureget = {0};
  sdatestructureget.Year = year;
  sdatestructureget.Month = month;
  sdatestructureget.Date = day;
  sdatestructureget.WeekDay = weekday;

  HAL_StatusTypeDef resd = HAL_RTC_SetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  UNUSED(resd);
	// HAL_StatusTypeDef res = HAL_RTC_Init(&RtcHandle);
  // HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_STATUS_REG, RTC_STATUS_TIME_OK);
}

void rtc_set_time(uint8_t hour, uint8_t minute, uint8_t second) {

  RTC_TimeTypeDef stimestructureget = {0};
  stimestructureget.Hours = hour;
  stimestructureget.Minutes = minute;
  stimestructureget.Seconds = second;

  HAL_StatusTypeDef rest = HAL_RTC_SetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  UNUSED(rest);
  // HAL_StatusTypeDef res = HAL_RTC_Init(&RtcHandle);
  // HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_STATUS_REG, RTC_STATUS_TIME_OK);
}

void rtc_print_datetime() {

  RTC_DateTypeDef sdatestructureget = {0};
  RTC_TimeTypeDef stimestructureget = {0};

  /* Get the RTC current Time */
  HAL_StatusTypeDef rest = HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);

  /* Get the RTC current Date */
  HAL_StatusTypeDef resd = HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);

  char buffer[50];
  if (resd == HAL_OK) {
    snprintf(buffer, sizeof(buffer), "D%02d%02d%02d W%d", sdatestructureget.Year, sdatestructureget.Month, sdatestructureget.Date, sdatestructureget.WeekDay);
    SERIAL_ECHO(buffer);
  }
  if (rest == HAL_OK) {
    snprintf(buffer, sizeof(buffer), " T%02d%02d%02d", stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
    SERIAL_ECHO(buffer);
  }
}

uint32_t rtc_read_status_reg() {
  return HAL_RTCEx_BKUPRead(&RtcHandle, RTC_STATUS_REG);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif




#endif

#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
