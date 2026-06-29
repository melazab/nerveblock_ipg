/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"
#include "FRAM.hpp"
#include "PortDefinitions.h"
#include "Saturn2_spi.hpp"
#include "Saturn_spi.hpp"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

// SPI_HandleTypeDef hspi1;
// SPI_HandleTypeDef hspi2;
// SPI_HandleTypeDef hspi3;
/* SPI1 init function */

void HAL_SPI_MspInit( SPI_HandleTypeDef* spiHandle )
{
    if ( spiHandle == Saturn1_spi::get_handle_ptr() )
    {
        /* USER CODE BEGIN SPI1_MspInit 0 */

        /* USER CODE END SPI1_MspInit 0 */
        /* SPI1 clock enable */
        Saturn1_spi::gpio_init();

        /* USER CODE BEGIN SPI1_MspInit 1 */

        /* USER CODE END SPI1_MspInit 1 */
    }
    //    else if ( spiHandle == SD::get_handle_ptr() )
    //    {
    //        /* USER CODE BEGIN SPI2_MspInit 0 */
    //        SD::gpio_init();
    //        /* USER CODE END SPI2_MspInit 0 */
    //    }
    else if ( spiHandle == Saturn2_spi::get_handle_ptr() )
    {
        /* USER CODE BEGIN SPI2_MspInit 0 */
        Saturn2_spi::gpio_init();
        /* USER CODE END SPI2_MspInit 0 */
    }
    else if ( spiHandle == FRAM::get_handle_ptr() )
    {
        FRAM::gpio_init();
    }
}

void HAL_SPI_MspDeInit( SPI_HandleTypeDef* spiHandle )
{
    if ( spiHandle == Saturn1_spi::get_handle_ptr() )
    {
        Saturn1_spi::gpio_deinit();
    }
    else if ( spiHandle == Saturn2_spi::get_handle_ptr() )
    {
        /* USER CODE BEGIN SPI2_MspDeInit 0 */
        Saturn2_spi::gpio_deinit();
        /* USER CODE END SPI2_MspDeInit 0 */
    }
    else if ( spiHandle == FRAM::get_handle_ptr() )
    {
        FRAM::gpio_deinit();
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
