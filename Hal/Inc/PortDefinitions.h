/// @file    PortDefinitions.h
/// @brief   Defines the the GPIO ports and pins for both the actual
#pragma once

#include "stm32wbxx.h"

// Some defines that are common to both EVAL and DEV
#define SerIntfUsart USART3

/** \addtogroup SPI_Defintions
 *  @{
 */
// #define SATURN1_SPI    SPI1
// #define SATURN2_SPI	SPI2
// #define SD_SPI SPI2

/** @}*/


/** \addtogroup Port_A_Pin_Defintions
 *  @{
 */
#define IMP_OUT_P_Pin    GPIO_PIN_0
#define IMP_OUT_N_Pin    GPIO_PIN_1
#define VRECT_MON_Pin    GPIO_PIN_3
#define HF_STIM_CSn_Pin  GPIO_PIN_4
#define HF_STIM_SCK_Pin  GPIO_PIN_5
#define HF_STIM_MISO_Pin GPIO_PIN_6
#define HF_STIM_MOSI_Pin GPIO_PIN_7
#define SAT_VHV_SW_Pin   GPIO_PIN_8
#define HV_MEAS_BUF_Pin  GPIO_PIN_9
#define LF_STIM_CLK_Pin  GPIO_PIN_10
#define LF_STIM_TFLG_Pin GPIO_PIN_11
#define LF_STIM_ERR_Pin  GPIO_PIN_12
#define SWDAT_Pin        GPIO_PIN_13
#define SWCLK_Pin        GPIO_PIN_14
#define BATT_SW_EN_Pin   GPIO_PIN_15
/** @}*/

/** \addtogroup Port_B_Pin_Defintions
 *  @{
 */

#define LF_STIM_PROG_Pin GPIO_PIN_1
#define LF_STIM_RSTn_Pin GPIO_PIN_2
#define SWO_Pin          GPIO_PIN_3
#define LF_STIM_SEL_Pin  GPIO_PIN_4
#define HF_STIM_STOP_Pin GPIO_PIN_5
#define LF_STIM_SYNC_Pin GPIO_PIN_7
#define LF_STIM_RUN_Pin  GPIO_PIN_8
#define LF_STIM_STOP_Pin GPIO_PIN_9
#define NFC_RXD_Pin      GPIO_PIN_10
#define NFC_TXD_Pin      GPIO_PIN_11
#define FRAM_CSn_Pin     GPIO_PIN_12
#define FRAM_SCK_Pin     GPIO_PIN_13
#define FRAM_MISO_Pin    GPIO_PIN_14
#define FRAM_MOSI_Pin    GPIO_PIN_15
/** @}*/

/** \addtogroup Port_C_Pin_Defintions
 *  @{
 */
#define ST_DEBUG_RX_2V5_Pin GPIO_PIN_0
#define ST_DEBUG_TX_2V5_Pin GPIO_PIN_1
#define BATT_LVL_Pin        GPIO_PIN_2
#define SAT_BOOST_SW_Pin    GPIO_PIN_3
#define FRAM_PWR_Pin        GPIO_PIN_4
#define FRAM_WPn_Pin        GPIO_PIN_5
#define AVDD_EN_Pin         GPIO_PIN_6
#define DVDDIO_EN_Pin       GPIO_PIN_7
#define CHG_EN_Pin          GPIO_PIN_8
#define MAG_DET_Pin         GPIO_PIN_9
#define LF_STIM_RAMP_Pin    GPIO_PIN_10
#define IMP_ENn_Pin         GPIO_PIN_11
#define CARRIER_DET_Pin     GPIO_PIN_13
#define OSC32_IN_Pin        GPIO_PIN_14
#define OSC32_OUT_Pin       GPIO_PIN_15
/** @}*/

/** \addtogroup Port_D_Pin_Defintions
 *  @{
 */
#define LF_STIM_CSn_Pin  GPIO_PIN_0
#define LF_STIM_SCK_Pin  GPIO_PIN_1
#define IMP_MUX_SEL_Pin  GPIO_PIN_2
#define LF_STIM_MISO_Pin GPIO_PIN_3
#define LF_STIM_MOSI_Pin GPIO_PIN_4
#define HF_STIM_RSTn_Pin GPIO_PIN_5
#define HF_STIM_TFLG_Pin GPIO_PIN_6
#define HF_STIM_ERR_Pin  GPIO_PIN_7
#define HF_STIM_PROG_Pin GPIO_PIN_8
#define HF_STIM_RAMP_Pin GPIO_PIN_9
#define HF_STIM_SYNC_Pin GPIO_PIN_10
#define HF_STIM_CONT_Pin GPIO_PIN_12
#define ICHG_Pin         GPIO_PIN_13
#define NFC_EN_Pin       GPIO_PIN_14
#define IMP_PDn_Pin      GPIO_PIN_15
/** @}*/

/** \addtogroup Port_E_Pin_Defintions
 *  @{
 */
#define LF_STIM_CONT_Pin GPIO_PIN_0
#define VRECT_MON_EN_Pin GPIO_PIN_1
#define HV_MEAS_EN_Pin   GPIO_PIN_3
#define HF_STIM_CLK_Pin  GPIO_PIN_4


/** @}*/
/** \addtogroup Port_H_Pin_Defintions
 *  @{
 */
#define HF_STIM_SEL_Pin GPIO_PIN_0
#define HF_STIM_RUN_Pin GPIO_PIN_1
#define BATTMON_EN_Pin  GPIO_PIN_3
/** @}*/


#ifndef All_Port_Defintions_Code_Folding
#    define IMP_OUT_P_Port    GPIOA
#    define IMP_OUT_N_Port    GPIOA
#    define VRECT_MON_Port    GPIOA
#    define HF_STIM_CSn_Port  GPIOA
#    define HF_STIM_SCK_Port  GPIOA
#    define HF_STIM_MISO_Port GPIOA
#    define HF_STIM_MOSI_Port GPIOA
#    define SAT_VHV_SW_Port   GPIOA
#    define HV_MEAS_BUF_Port  GPIOA
#    define LF_STIM_CLK_Port  GPIOA
#    define LF_STIM_TFLG_Port GPIOA
#    define LF_STIM_ERR_Port  GPIOA
#    define SWDAT_Port        GPIOA
#    define SWCLK_Port        GPIOA
#    define BATT_SW_EN_Port   GPIOA

#    define LF_STIM_PROG_Port GPIOB
#    define LF_STIM_RSTn_Port GPIOB
#    define SWO_Port          GPIOB
#    define LF_STIM_SEL_Port  GPIOB
#    define HF_STIM_STOP_Port GPIOB
#    define LF_STIM_SYNC_Port GPIOB
#    define LF_STIM_RUN_Port  GPIOB
#    define LF_STIM_STOP_Port GPIOB
#    define NFC_RXD_Port      GPIOB
#    define NFC_TXD_Port      GPIOB
#    define FRAM_CSn_Port     GPIOB
#    define FRAM_SCK_Port     GPIOB
#    define FRAM_MISO_Port    GPIOB
#    define FRAM_MOSI_Port    GPIOB

#    define ST_DEBUG_RX_2V5_Port GPIOC
#    define ST_DEBUG_TX_2V5_Port GPIOC
#    define BATT_LVL_Port        GPIOC
#    define SAT_BOOST_SW_Port    GPIOC
#    define FRAM_PWR_Port        GPIOC
#    define FRAM_WPn_Port        GPIOC
#    define AVDD_EN_Port         GPIOC
#    define DVDDIO_EN_Port       GPIOC
#    define CHG_EN_Port          GPIOC
#    define MAG_DET_Port         GPIOC
#    define LF_STIM_RAMP_Port    GPIOC
#    define IMP_ENn_Port         GPIOC
#    define CARRIER_DET_Port     GPIOC

#    define LF_STIM_CSn_Port  GPIOD
#    define LF_STIM_SCK_Port  GPIOD
#    define IMP_MUX_SEL_Port  GPIOD
#    define LF_STIM_MISO_Port GPIOD
#    define LF_STIM_MOSI_Port GPIOD
#    define HF_STIM_RSTn_Port GPIOD
#    define HF_STIM_TFLG_Port GPIOD
#    define HF_STIM_ERR_Port  GPIOD
#    define HF_STIM_PROG_Port GPIOD
#    define HF_STIM_RAMP_Port GPIOD
#    define HF_STIM_SYNC_Port GPIOD

#    define HF_STIM_CONT_Port GPIOD
#    define ICHG_Port         GPIOD
#    define NFC_EN_Port       GPIOD
#    define IMP_PDn_Port      GPIOD

#    define LF_STIM_CONT_Port GPIOE
#    define VRECT_MON_EN_Port GPIOE
#    define HV_MEAS_EN_Port   GPIOE
#    define HF_STIM_CLK_Port  GPIOE

#    define HF_STIM_SEL_Port GPIOH
#    define HF_STIM_RUN_Port GPIOH
#    define BATTMON_EN_Port  GPIOH


#endif

/** \addtogroup NVIC_Intrrupt_Defines
 *  @{
 */
/** @}*/
