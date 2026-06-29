/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"
#include "main.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef uartHandle1;
extern DMA_HandleTypeDef hdma_lpuart1_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init( void )
{
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    uartHandle1.Instance                    = USART1;
    uartHandle1.Init.BaudRate               = 115200;
    uartHandle1.Init.WordLength             = UART_WORDLENGTH_8B;
    uartHandle1.Init.StopBits               = UART_STOPBITS_1;
    uartHandle1.Init.Parity                 = UART_PARITY_NONE;
    uartHandle1.Init.Mode                   = UART_MODE_TX_RX;
    uartHandle1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    uartHandle1.Init.OverSampling           = UART_OVERSAMPLING_16;
    uartHandle1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    uartHandle1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    uartHandle1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if ( HAL_UART_Init( &uartHandle1 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetTxFifoThreshold( &uartHandle1, UART_TXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetRxFifoThreshold( &uartHandle1, UART_RXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_DisableFifoMode( &uartHandle1 ) != HAL_OK )
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

void HAL_UART_MspInit( UART_HandleTypeDef* uartHandle )
{
    GPIO_InitTypeDef GPIO_InitStruct             = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    HAL_DMA_MuxSyncConfigTypeDef pSyncConfig;
    
    if ( uartHandle->Instance == USART1 )
    {
        /* USER CODE BEGIN USART1_MspInit 0 */

        /* USER CODE END USART1_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
        if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
        {
            Error_Handler();
        }

        /* USART1 clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

        /* USER CODE BEGIN USART1_MspInit 1 */

        /* USER CODE END USART1_MspInit 1 */
    }
    else if ( uartHandle->Instance == LPUART1 )
    {
        /* USER CODE BEGIN LPUART1_MspInit 0 */

        /* USER CODE END LPUART1_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection  = RCC_PERIPHCLK_LPUART1;
        PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
        if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
        {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_LPUART1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        /**LPUART1 GPIO Configuration
        PA2     ------> LPUART1_TX
        PA3     ------> LPUART1_RX
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
        HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

        /* LPUART1 DMA Init */
        /* LPUART1_TX Init */
        hdma_lpuart1_tx.Instance                 = DMA1_Channel5;
        hdma_lpuart1_tx.Init.Request             = DMA_REQUEST_LPUART1_TX;
        hdma_lpuart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_lpuart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_lpuart1_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_lpuart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_lpuart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_lpuart1_tx.Init.Mode                = DMA_NORMAL;
        hdma_lpuart1_tx.Init.Priority            = DMA_PRIORITY_LOW;
        if ( HAL_DMA_Init( &hdma_lpuart1_tx ) != HAL_OK )
        {
            Error_Handler();
        }

        pSyncConfig.SyncSignalID  = HAL_DMAMUX1_SYNC_DMAMUX1_CH1_EVT;
        pSyncConfig.SyncPolarity  = HAL_DMAMUX_SYNC_NO_EVENT;
        pSyncConfig.SyncEnable    = DISABLE;
        pSyncConfig.EventEnable   = DISABLE;
        pSyncConfig.RequestNumber = 1;
        if ( HAL_DMAEx_ConfigMuxSync( &hdma_lpuart1_tx, &pSyncConfig ) != HAL_OK )
        {
            Error_Handler();
        }

        __HAL_LINKDMA( uartHandle, hdmatx, hdma_lpuart1_tx );

        /* LPUART1 interrupt Init */
        HAL_NVIC_SetPriority( LPUART1_IRQn, 0, 0 );
        HAL_NVIC_EnableIRQ( LPUART1_IRQn );
        /* USER CODE BEGIN LPUART1_MspInit 1 */

        /* USER CODE END LPUART1_MspInit 1 */
    }
}

void HAL_UART_MspDeInit( UART_HandleTypeDef* uartHandle )
{
    if ( uartHandle->Instance == USART1 )
    {
        /* USER CODE BEGIN USART1_MspDeInit 0 */

        /* USER CODE END USART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit( GPIOA, GPIO_PIN_9 | GPIO_PIN_10 );

        /* USER CODE BEGIN USART1_MspDeInit 1 */

        /* USER CODE END USART1_MspDeInit 1 */
    }
    else if ( uartHandle->Instance == LPUART1 )
    {
        /* USER CODE BEGIN LPUART1_MspDeInit 0 */

        /* USER CODE END LPUART1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_LPUART1_CLK_DISABLE();

        /**LPUART1 GPIO Configuration
        PA2     ------> LPUART1_TX
        PA3     ------> LPUART1_RX
        */
        HAL_GPIO_DeInit( GPIOA, GPIO_PIN_2 | GPIO_PIN_3 );

        /* LPUART1 DMA DeInit */
        HAL_DMA_DeInit( uartHandle->hdmatx );

        /* LPUART1 interrupt DeInit */
        HAL_NVIC_DisableIRQ( LPUART1_IRQn );
        /* USER CODE BEGIN LPUART1_MspDeInit 1 */

        /* USER CODE END LPUART1_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
