/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"
#include "tx_api.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_nucleo.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DEMO_STACK_SIZE      (1024)
#define DEMO_BYTE_POOL_SIZE  (32 * 1024)
#define DEMO_BLOCK_POOL_SIZE (100)
#ifdef TX_ENABLE_EVENT_TRACE
#define TRACE_MEM_SIZE       (64*1024)
#define TRACE_SIZE           (24)
#define TRACE_FILTER (TX_TRACE_INTERNAL_EVENTS|TX_TRACE_INTERRUPT_CONTROL_EVENT|TX_TRACE_TIME_EVENTS)
#endif
//#define USE_NUCLEO

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

// /* USER CODE BEGIN EFP */
// /** \addtogroup Port_A_Pin_Defintions
//  *  @{
//  */
// #define LF_STIM_RSTn_Pin  	GPIO_PIN_0
// #define HF_STIM_SCK_Pin		GPIO_PIN_1
// #define LF_STIM_PROG_Pin 	GPIO_PIN_2
// #define LF_STIM_RAMP_Pin    GPIO_PIN_3
// #define FRAM_CSn_Pin   		GPIO_PIN_4
// #define FRAM_SCK_Pin   		GPIO_PIN_5
// #define FRAM_MISO_Pin    	GPIO_PIN_6
// #define FRAM_MOSI_Pin	    GPIO_PIN_7
// #define BAT_CHG_ENn_Pin		GPIO_PIN_8				//old-ST_DEBUG_CK_Pin
// #define IMP_SCL_Pin			GPIO_PIN_9				//old-LF_STIM_SYNC_Pin
// #define IMP_SDA_Pin		   	GPIO_PIN_10				//old-LF_STIM_SEL_Pin
// #define HF_STIM_MISO_Pin    GPIO_PIN_11
// #define SAT_VHV_SW_Pin		GPIO_PIN_12
// #define SWDAT_Pin		  	GPIO_PIN_13
// #define SWCLK_Pin			GPIO_PIN_14
// #define HF_STIM_CSn_Pin		GPIO_PIN_15
// /** @}*/

// /** \addtogroup Port_B_Pin_Defintions
//  *  @{
//  */
// #define IMP_EN_Pin		   	GPIO_PIN_0
// #define SD_PWR_EN_Pin		GPIO_PIN_1
// #define USB_RSTn_Pin 	   	GPIO_PIN_2
// #define SWO_Pin_Pin       	GPIO_PIN_3
// #define BAT_CHG_STAT_Pin   	GPIO_PIN_4
// #define HF_STIM_MOSI_Pin   	GPIO_PIN_5
// #define USB_RX_2V5_Pin	    GPIO_PIN_6				//old-ST_DEBUG_TX_Pin
// #define USB_TX_2V5_Pin	    GPIO_PIN_7				//old-ST_DEBUG_RX_Pin
// #define LF_STIM_SYNC_Pin	GPIO_PIN_8				//old-IMP_SCL_Pin
// #define HF_STIM_RUN_Pin	   GPIO_PIN_9
// #define HF_STIM_STOP_Pin   GPIO_PIN_10
// #define LF_STIM_SEL_Pin		GPIO_PIN_11				//old-IMP_SDA_Pin
// #define SD_CS_Pin		   GPIO_PIN_12
// #define SD_SCLK_Pin		   GPIO_PIN_13
// #define SD_MISO_Pin		   GPIO_PIN_14
// #define SD_MOSI_Pin		   GPIO_PIN_15
// /** @}*/

// /** \addtogroup Port_C_Pin_Defintions
//  *  @{
//  */
// #define ST_DEBUG_RX_2V5_Pin	 GPIO_PIN_0				//old-USB_RX_Pin
// #define ST_DEBUG_TX_2V5_Pin	 GPIO_PIN_1				//old-USB_TX_Pin
// #define BATTMON_EN_Pin	  GPIO_PIN_2
// #define BATT_LVL_Pin	  GPIO_PIN_3
// #define ST_IMP_INA_Pin  GPIO_PIN_4
// #define ST_IMP_INB_Pin    GPIO_PIN_5
// #define HF_STIM_CONT_Pin  GPIO_PIN_6
// #define HF_STIM_SEL_Pin   GPIO_PIN_7
// #define HF_STIM_SYNC_Pin  GPIO_PIN_8
// #define HF_STIM_RAMP_Pin  GPIO_PIN_9
// #define HF_STIM_PROG_Pin  GPIO_PIN_10
// #define HF_STIM_ERR_Pin   GPIO_PIN_11
// #define HF_STIM_TFLG_Pin  GPIO_PIN_12
// #define HF_STIM_RSTn_Pin  GPIO_PIN_13
// #define OSC32_IN_Pin      GPIO_PIN_14
// #define OSC32_OUT_Pin     GPIO_PIN_15
// /** @}*/

// /** \addtogroup Port_D_Pin_Defintions
//  *  @{
//  */
// #define LF_STIM_CSn_Pin   GPIO_PIN_0
// #define LF_STIM_SCK_Pin   GPIO_PIN_1
// #define PD2_Pin		  	  GPIO_PIN_2				//old-TRIG_1_Pin
// #define LF_STIM_MISO_Pin	GPIO_PIN_3
// #define LF_STIM_MOSI_Pin    GPIO_PIN_4
// #define LF_STIM_CONT_Pin    GPIO_PIN_5
// #define LF_STIM_STOP_Pin  GPIO_PIN_6
// #define LF_STIM_RUN_Pin   GPIO_PIN_7
// #define SAT_BOOST_EN_Pin  GPIO_PIN_8
// #define LF_STIM_ERR_Pin	  GPIO_PIN_9
// #define LF_STIM_TFLG_Pin  GPIO_PIN_10
// #define PWR_LED_Pin  	  GPIO_PIN_11				//old-STIM_LED_BLINKn_Pin
// #define PD12_Pin		  GPIO_PIN_12				//old-BAT_LED_CHG_GOODn_Pin
// #define STIM_LED_Pin	  GPIO_PIN_13				//old-BAT_LED_LOWn_Pin
// #define DVDD_3V3_EN		  GPIO_PIN_14				//old-BAT_LED_MUSTCHGin_Pin
// #define BLE_LED_Pin    	  GPIO_PIN_15				//old-BLE_LED_STATn_Pin
// /** @}*/

// /** \addtogroup Port_E_Pin_Defintions
//  *  @{
//  */
// #define AVDD_EN_Pin      GPIO_PIN_0
// #define FRAM_WPn_Pin	 GPIO_PIN_1
// #define FRAM_PWR_Pin     GPIO_PIN_2
// #define LF_STIM_CLK_Pin   GPIO_PIN_3
// #define HF_STIM_CLK_Pin   GPIO_PIN_4


// /** @}*/
// /** \addtogroup Port_H_Pin_Defintions
//  *  @{
//  */
// #define PB1_OUTn_Pin	GPIO_PIN_0
// #define DVDD_ENn_Pin	GPIO_PIN_1
// #define DVDDIO_EN_Pin	GPIO_PIN_3
// /** @}*/


// #ifndef All_Port_Defintions_Code_Folding
// #define LF_STIM_RSTn_Port   GPIOA
// #define HF_STIM_SCK_Port		GPIOA
// #define LF_STIM_PROG_Port    GPIOA
// #define LF_STIM_RAMP_Port    GPIOA
// #define FRAM_CSn_Port   		GPIOA
// #define FRAM_SCK_Port   		GPIOA
// #define FRAM_MISO_Port    	GPIOA
// #define FRAM_MOSI_Port  	GPIOA
// #define BAT_CHG_ENn_Port		GPIOA
// #define IMP_SCK_Port	GPIOA
// #define IMP_SDA_Port   	GPIOA
// #define HF_STIM_MISO_Port   	GPIOA
// #define SAT_VHV_SW_Port		GPIOA
// #define SWDAT_Port		   	GPIOA
// #define SWCLK_Port		   	GPIOA
// #define HF_STIM_CSn_Port	GPIOA

// #define IMP_EN_Port		   GPIOB
// #define SD_PWR_EN_Port		GPIOB
// #define USB_RSTn_Port 	   GPIOB
// #define SWO_Pin_Port        GPIOB
// #define BAT_CHG_STAT_Port   GPIOB
// #define HF_STIM_MOSI_Port   GPIOB
// #define USB_RX_2V5_Port    GPIOB
// #define USB_TX_2V5_Port    GPIOB
// #define LF_STIM_SYNC_Port		   GPIOB
// #define HF_STIM_RUN_Port	   GPIOB
// #define HF_STIM_STOP_Port   GPIOB
// #define LF_STIM_SEL_Port		   GPIOB
// #define SD_CS_Port		   GPIOB
// #define SD_SCLK_Port		   GPIOB
// #define SD_MISO_Port		   GPIOB
// #define SD_MOSI_Port		   GPIOB

// #define ST_DEBUG_RX_2V5_Port	     GPIOC
// #define ST_DEBUG_TX_2V5_Port	     GPIOC
// #define BATTMON_EN_Port	  GPIOC
// #define BATT_LVL_Port	  GPIOC
// #define ST_IMP_INA_Port    GPIOC
// #define ST_IMP_INB_Port    GPIOC
// #define HF_STIM_CONT_Port  GPIOC
// #define HF_STIM_SEL_Port   GPIOC
// #define HF_STIM_SYNC_Port  GPIOC
// #define HF_STIM_RAMP_Port  GPIOC
// #define HF_STIM_PROG_Port  GPIOC
// #define HF_STIM_ERR_Port   GPIOC
// #define HF_STIM_TFLG_Port  GPIOC
// #define HF_STIM_RSTn_Port  GPIOC
// #define OSC32_IN_Port      GPIOC
// #define OSC32_OUT_Port     GPIOC

// #define LF_STIM_CSn_Port   GPIOD
// #define LF_STIM_SCK_Port   GPIOD
// #define PD2_Port		  GPIOD
// #define LF_STIM_MISO_Port	GPIOD
// #define LF_STIM_MOSI_Port    GPIOD
// #define LF_STIM_CONT_Port    GPIOD
// #define LF_STIM_STOP_Port  GPIOD
// #define LF_STIM_RUN_Port   GPIOD
// #define SAT_BOOST_EN_Port  GPIOD
// #define LF_STIM_ERR_Port	  GPIOD
// #define LF_STIM_TFLG_Port  GPIOD
// #define PWR_LED_Port  GPIOD
// #define PD12_Port  GPIOD
// #define STIM_LED_Port  GPIOD
// #define DVDD_3V3_EN_Port  GPIOD
// #define BLE_LED_Port     GPIOD

// #define AVDD_EN_Port      GPIOE
// #define FRAM_WPn_Port	 GPIOE
// #define FRAM_PWR_Port     GPIOE
// #define LF_STIM_CLK_Port   GPIOE
// #define HF_STIM_CLK_Port   GPIOE

// #define PB1_OUTn_Port	GPIOH
// #define DVDD_ENn_Port	GPIOH
// #define DVDDIO_EN_Port	GPIOH

// #endif
// /* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
void   MX_LPUART1_UART_Init(void);
//void   MX_USART1_UART_Init(void);
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
