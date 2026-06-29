/// @file    USB.cpp
/// @brief   Handles the incoming byte stream from the UART interfaces. Sends
///          response packets to the UART interfaces.
/// @details Interfaces with the semaphore from the UART receive interrupt
/// 		 handler to know when data is ready in the receiver. Receives
///			 response data via a queue and applies the appropriate protocol
///			 wrapper around it before sending it to the UART interface.

#include "tx_api.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>


//#include "assertLocal.h"
#include "main.h"
#include "stm32wbxx.h"
#include "PortDefinitions.h"
#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"
#include "CommandHandler.hpp"
#include "SerialInterface.h"
#include "USB.h"


#define ArrayLen( x )                                                                              \
    ( sizeof( x ) / sizeof( ( x )[ 0 ] ) )    // TODO this needs to be in some kind of global H file

// Module Variable Definitions
uint32_t usbtick = 0;
static UART_HandleTypeDef huart2;
static DMA_HandleTypeDef hdma_uart2_rx;
static DMA_HandleTypeDef hdma_uart2_tx;

static TX_SEMAPHORE semBytesReady;

static uint8_t serRxBytesDma[ 32 ];


/*****************************************************************************/
void usbByteInThreadEntry( uint32_t treadInput )
/*****************************************************************************/
{
    uint32_t oldPos = 0;
    uint32_t curPos = 0;


    for ( ;; )
    {
        if ( TX_SUCCESS == tx_semaphore_get( &semBytesReady, 10 ) )
        {
            // HAL_GPIO_WritePin (GPIOD, BLE_LED_Pin, GPIO_PIN_RESET);
            usbtick = HAL_GetTick();
            // curPos = __HAL_DMA_GET_COUNTER(&hdmaRS422rx);
            curPos  = ArrayLen( serRxBytesDma ) - __HAL_DMA_GET_COUNTER( &hdma_uart2_rx );
            curPos &= ArrayLen( serRxBytesDma ) - 1;
            while ( oldPos != curPos )
            {
                crProtQueueByte( serRxBytesDma[ oldPos++ ] );
                oldPos &= ArrayLen( serRxBytesDma ) - 1;
            }
        }
        else{
            tx_thread_relinquish();
        }
    }
}

/*****************************************************************************/
void usbWrite( uint8_t* buf, uint16_t bufLen )
/*****************************************************************************/
{
    HAL_UART_Transmit_DMA( &huart2, buf, bufLen );
}

/*****************************************************************************/
void usbSendSerialDataBuf( SERIAL_DATA_OUT_BUF_T* serOutBuf )
/*****************************************************************************/
{
    HAL_UART_Transmit_DMA( &huart2, serOutBuf->outDataBuf, serOutBuf->bufDataLen );
    // TODO do we need a queue
    // osMessageQueuePut(queSerRespHndl, (const void *)(&serOutBuf), 0U, 0U);

    return;
}

/*****************************************************************************/
void usbInit( void )
/*****************************************************************************/
{
    tx_semaphore_create( &semBytesReady, "Bytes Ready Sem", 0 );

    // 2764800; // 1843200; // 921600; // Alternative high speed baud rates
    // 2000000  // 3000000  // 230400  // 460800
    huart2.Instance                    = USART1;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if ( HAL_UART_Init( &huart2 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetTxFifoThreshold( &huart2, UART_TXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_SetRxFifoThreshold( &huart2, UART_RXFIFO_THRESHOLD_1_8 ) != HAL_OK )
    {
        Error_Handler();
    }
    if ( HAL_UARTEx_DisableFifoMode( &huart2 ) != HAL_OK )
    {
        Error_Handler();
    }
    HAL_StatusTypeDef status

        = HAL_UARTEx_ReceiveToIdle_DMA( &huart2, serRxBytesDma, ArrayLen( serRxBytesDma ) );
}

/*****************************************************************************/
void usbGpioInit( void )
/*****************************************************************************/
{
    GPIO_InitTypeDef GPIO_InitStruct             = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    /* USER CODE END USART1_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection  = RCC_PERIPHCLK_LPUART1;
    PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    if ( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
    {
        Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitStruct.Pin       = ST_DEBUG_RX_2V5_Pin | ST_DEBUG_TX_2V5_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );


    hdma_uart2_rx.Instance                 = DMA1_Channel3;
    hdma_uart2_rx.Init.Request             = DMA_REQUEST_USART1_RX;
    hdma_uart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_uart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart2_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_uart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    if ( HAL_DMA_Init( &hdma_uart2_rx ) != HAL_OK )
    {
        Error_Handler();
    }

    __HAL_LINKDMA( &huart2, hdmarx, hdma_uart2_rx );


    hdma_uart2_tx.Instance                 = DMA1_Channel4;
    hdma_uart2_tx.Init.Request             = DMA_REQUEST_USART1_TX;
    hdma_uart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart2_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart2_tx.Init.Priority            = DMA_PRIORITY_LOW;
    if ( HAL_DMA_Init( &hdma_uart2_tx ) != HAL_OK )
    {
        Error_Handler();
    }

    __HAL_LINKDMA( &huart2, hdmatx, hdma_uart2_tx );


    HAL_NVIC_SetPriority( USART1_IRQn, 5, 0 );
    HAL_NVIC_EnableIRQ( USART1_IRQn );
}


/*****************************************************************************/
void usbGpioDeInit( void )
/*****************************************************************************/
{
    __HAL_RCC_USART1_CLK_DISABLE();

    HAL_GPIO_DeInit( GPIOC, ST_DEBUG_RX_2V5_Pin | ST_DEBUG_TX_2V5_Pin );
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/
//
///*****************************************************************************/
//void HAL_UART_RxHalfCpltCallback( UART_HandleTypeDef* huart )
///*****************************************************************************/
//{
//    // TODO I don't think these callbacks are used when using HAL_UARTEx_ReceiveToIdle_DMA
//    if ( huart == &huart2 )
//    {
//        tx_semaphore_put( &semBytesReady );
//    }
//}
//
//// HAL_UARTEx_RxEventCallback
//
////*****************************************************************************/
//void HAL_UART_RxCpltCallback( UART_HandleTypeDef* huart )
///*****************************************************************************/
//{
//    // TODO I don't think these callbacks are used when using HAL_UARTEx_ReceiveToIdle_DMA
//    if ( huart == &huart2 )
//    {
//        tx_semaphore_put( &semBytesReady );
//    }
//}

//*****************************************************************************/
void HAL_UARTEx_RxEventCallback( UART_HandleTypeDef* huart, uint16_t Size )
//*****************************************************************************/
{
    if ( huart->Instance == USART1 )    // &hStDbgUart3 )
    {
        tx_semaphore_put( &semBytesReady );
    }
}

/**
 * @brief This function handles LPUART1 global interrupt.
 */
void USART1_IRQHandler( void )
{
    HAL_UART_IRQHandler( &huart2 );
}

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void DMA1_Channel3_IRQHandler( void )
{
    HAL_DMA_IRQHandler( &hdma_uart2_rx );
}

/**
 * @brief This function handles DMA1 channel4 global interrupt.
 */
void DMA1_Channel4_IRQHandler( void )
{
    HAL_DMA_IRQHandler( &hdma_uart2_tx );
}

/** @}   */    // end group SerialInterface
