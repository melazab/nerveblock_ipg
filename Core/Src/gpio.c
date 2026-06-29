/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"
#include "PortDefinitions.h"

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
    uint32_t alternate;             // For AF pins
    GPIO_PinState initial_state;    // For output pins
} gpio_config_t;

// Configuration table
static const gpio_config_t gpio_configs[] = {

    // Port,    Pin,                    Mode,           Pull,       Speed, AF, Initial State
    { GPIOA, IMP_OUT_P_Pin,       GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, IMP_OUT_N_Pin,       GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, VRECT_MON_Pin,       GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, HF_STIM_CSn_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOA, HF_STIM_SCK_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOA, HF_STIM_MISO_Pin,    GPIO_MODE_INPUT,     GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOA, HF_STIM_MOSI_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOA, SAT_VHV_SW_Pin,      GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOA, HV_MEAS_BUF_Pin,     GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, LF_STIM_CLK_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOA, LF_STIM_TFLG_Pin,    GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, LF_STIM_ERR_Pin,     GPIO_MODE_INPUT,     GPIO_NOPULL,   0,                         0, 0              },
    { GPIOA, BATT_SW_EN_Pin,      GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },

    { GPIOB, LF_STIM_PROG_Pin,    GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0,                         0, 0              },
    { GPIOB, LF_STIM_RSTn_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, SWO_Pin,             GPIO_MODE_AF_PP,     GPIO_NOPULL,   0,                         0, 0              },
    { GPIOB, LF_STIM_SEL_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, HF_STIM_STOP_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, LF_STIM_SYNC_Pin,    GPIO_MODE_INPUT,     GPIO_NOPULL,   0,                         0, 0              },
    { GPIOB, LF_STIM_RUN_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, LF_STIM_STOP_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, NFC_RXD_Pin,         GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0,                         0, 0              },
    { GPIOB, NFC_TXD_Pin,         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOB, FRAM_CSn_Pin,        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOB, FRAM_SCK_Pin,        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOB, FRAM_MISO_Pin,       GPIO_MODE_INPUT,     GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOB, FRAM_MOSI_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },

    { GPIOC, ST_DEBUG_RX_2V5_Pin, GPIO_MODE_INPUT,     GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOC, ST_DEBUG_TX_2V5_Pin, GPIO_MODE_OUTPUT_PP, 0,             GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOC, BATT_LVL_Pin,        GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOC, SAT_BOOST_SW_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOC, FRAM_PWR_Pin,        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },
    { GPIOC, FRAM_WPn_Pin,        GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOC, AVDD_EN_Pin,         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOC, DVDDIO_EN_Pin,       GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOC, CHG_EN_Pin,          GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },
    { GPIOC, MAG_DET_Pin,         GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0,                         0, 0              },
    { GPIOC, LF_STIM_RAMP_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOC, IMP_ENn_Pin,         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },
    { GPIOC, CARRIER_DET_Pin,     GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0,                         0, 0              },
    { GPIOC, OSC32_IN_Pin,        GPIO_MODE_AF_PP,     GPIO_NOPULL,   0,                         0, 0              },
    { GPIOC, OSC32_OUT_Pin,       GPIO_MODE_AF_PP,     GPIO_NOPULL,   0,                         0, 0              },

    { GPIOD, LF_STIM_CSn_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOD, LF_STIM_SCK_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOD, IMP_MUX_SEL_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOD, LF_STIM_MISO_Pin,    GPIO_MODE_INPUT,     GPIO_PULLDOWN, GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOD, LF_STIM_MOSI_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOD, HF_STIM_RSTn_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOD, HF_STIM_TFLG_Pin,    GPIO_MODE_ANALOG,    GPIO_NOPULL,   0,                         0, 0              },
    { GPIOD, HF_STIM_ERR_Pin,     GPIO_MODE_INPUT,     GPIO_NOPULL,   0,                         0, 0              },
    { GPIOD, HF_STIM_PROG_Pin,    GPIO_MODE_INPUT,     GPIO_PULLDOWN, 0,                         0, 0              },
    { GPIOD, HF_STIM_RAMP_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOD, HF_STIM_SYNC_Pin,    GPIO_MODE_INPUT,     GPIO_NOPULL,   GPIO_SPEED_FREQ_VERY_HIGH, 0, 0              },
    { GPIOD, HF_STIM_CONT_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOD, ICHG_Pin,            GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },
    { GPIOD, NFC_EN_Pin,          GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_SET   },
    { GPIOD, IMP_PDn_Pin,         GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },

    { GPIOE, LF_STIM_CONT_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOE, VRECT_MON_EN_Pin,    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOE, HV_MEAS_EN_Pin,      GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOE, HF_STIM_CLK_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },

    { GPIOH, HF_STIM_SEL_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOH, HF_STIM_RUN_Pin,     GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
    { GPIOH, BATTMON_EN_Pin,      GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,   0,                         0, GPIO_PIN_RESET },
};

void gpio_initialization( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // Configure each pin
    for ( size_t i = 0; i < sizeof( gpio_configs ) / sizeof( gpio_configs[ 0 ] ); i++ )
    {
        if ( gpio_configs[ i ].mode == GPIO_MODE_OUTPUT_PP
             || gpio_configs[ i ].mode == GPIO_MODE_OUTPUT_OD )
        {
            HAL_GPIO_WritePin( gpio_configs[ i ].port,
                               gpio_configs[ i ].pin,
                               gpio_configs[ i ].initial_state );
        }

        GPIO_InitStruct.Pin   = gpio_configs[ i ].pin;
        GPIO_InitStruct.Mode  = gpio_configs[ i ].mode;
        GPIO_InitStruct.Pull  = gpio_configs[ i ].pull;
        GPIO_InitStruct.Speed = gpio_configs[ i ].speed;

        if ( gpio_configs[ i ].mode == GPIO_MODE_AF_PP
             || gpio_configs[ i ].mode == GPIO_MODE_AF_OD )
        {
            GPIO_InitStruct.Alternate = gpio_configs[ i ].alternate;
        }

        HAL_GPIO_Init( gpio_configs[ i ].port, &GPIO_InitStruct );
    }
}
