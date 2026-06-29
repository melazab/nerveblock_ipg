/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    adc.c
 * @brief   This file provides code for the configuration
 *          of the ADC instances.
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
#include "stm32wbxx.h"
#include <adc.h>

#include "PortDefinitions.h"
#include "tx_api.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

// #include "assertLocal.h"
#include "main.h"
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
TX_EVENT_FLAGS_GROUP data_ready;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

void MX_ADC1_Init( void )
{
    tx_event_flags_create( &data_ready, "Data Ready Flag" );
    tx_event_flags_set( &data_ready, 0, TX_AND );

    ADC_ChannelConfTypeDef sConfig = { 0 };

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;    // ENABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;    // DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 0;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;    // ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = DISABLE;    // DISABLE; // ADC_EOC_SINGLE_CONV;
    if ( HAL_ADC_Init( &hadc1 ) != HAL_OK )
    {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_5;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;    // ADC_SAMPLETIME_3CYCLES;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    sConfig.Channel      = ADC_CHANNEL_6;
    sConfig.Rank         = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }

    ADC1_VREF();
}

void ADC1_Init_IMP( void )
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    sConfig.Channel      = ADC_CHANNEL_5;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;    // ADC_SAMPLETIME_3CYCLES;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }
    sConfig.Channel      = ADC_CHANNEL_6;
    sConfig.Rank         = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }
}

void HAL_ADC_MspInit( ADC_HandleTypeDef* adcHandle )
{
    GPIO_InitTypeDef GPIO_InitStruct             = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    if ( adcHandle->Instance == ADC1 )
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        PeriphClkInitStruct.AdcClockSelection    = RCC_ADCCLKSOURCE_SYSCLK;

        __HAL_RCC_ADC_CLK_ENABLE();

        hdma_adc1.Instance                 = DMA2_Channel1;    // Check to see if this is correct
        hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_CIRCULAR;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;

        if ( HAL_DMA_Init( &hdma_adc1 ) != HAL_OK )
        {
            Error_Handler();
        }

        __HAL_LINKDMA( &hadc1, DMA_Handle, hdma_adc1 );

        // HAL_NVIC_DisableIRQ(ADC1_IRQn);
        HAL_NVIC_SetPriority( ADC1_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( ADC1_IRQn );
    }
}

void HAL_ADC_MspDeInit( ADC_HandleTypeDef* adcHandle )
{
    if ( adcHandle->Instance == ADC1 )
    {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_ADC_CLK_DISABLE();

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit( adcHandle->DMA_Handle );
        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
}

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef* hadc )
{
    tx_event_flags_set( &data_ready, ADC_READY_FLAG, TX_OR );
    HAL_NVIC_DisableIRQ( DMA2_Channel1_IRQn );
}
