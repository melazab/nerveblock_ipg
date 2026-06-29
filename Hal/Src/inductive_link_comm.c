#include "CmdRespProtocol.h"
#include "PortDefinitions.h"
#include "main.h"
#include "stm32wbxx.h"
#include "tim.h"
#include <stdint.h>
#include <stdio.h>


TIM_OC_InitTypeDef sConfigOC = { 0 };
static uint8_t curByte;
static uint8_t sendbit = 0;
static uint16_t PW     = 0;
static uint16_t high_array[ 50 ];
static uint16_t low_array[ 50 ];
static uint16_t cmdLength;
uint32_t end_pulse;
bool curr_match       = 0;
static bool first_bit = true;

static uint8_t rxBuffer[ 12 ] = { 0 };
static uint32_t IC_Val;
static uint8_t state;
static uint8_t edges            = 0;
static uint16_t dbg_array[ 50 ] = { 0 };
static uint32_t tim2_start      = 0;

typedef enum
{
    B_DECODE_INVALID        = 0,    // Invalid state
    B_DECODE_WAIT_FOR_START = 1,
    B_DECODE_PROCESS_BYTE   = 2     // Start byte found, process data bits
} BYTE_DECODER_STATES_T;

#define START_BIT      3000 * ( 1 )
#define ONE_BIT        2000 * ( 1 )
#define ZERO_BIT       1000 * ( 1 )
#define WAIT_BIT       5000 * ( 1 )
#define WAIT_BYTE      11000 * ( 1 )

void nfc_rx_falling_edge( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin              = NFC_RXD_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull             = GPIO_PULLDOWN;
    HAL_GPIO_Init( NFC_RXD_Port, &GPIO_InitStruct );

    HAL_NVIC_SetPriority( EXTI15_10_IRQn, 2, 0 );
    HAL_NVIC_EnableIRQ( EXTI15_10_IRQn );
    SET_BIT( EXTI->PR1, EXTI_FTSR1_FT10 );
}

void timIntSendSerialDataBuf( SERIAL_DATA_OUT_BUF_T* serOutBuf )
{
    curByte = 0;
    sendbit = 0;

    for ( int i = 0; i < serOutBuf->bufDataLen; i++ )
    {
        curByte          = serOutBuf->outDataBuf[ i ];
        uint8_t curBit   = 0;
        uint8_t bitValue = 0;

        while ( curBit < 9 )
        {
            if ( curBit == 0 )
            {
                high_array[ i * 9 ] = START_BIT;
                low_array[ i * 9 ]  = WAIT_BIT - START_BIT;
                curBit              = 1;
            }

            bitValue = curByte & 1;

            if ( bitValue == 0 )
            {
                high_array[ i * 9 + curBit ] = ZERO_BIT;
                low_array[ i * 9 + curBit ]  = WAIT_BIT - ZERO_BIT;
            }
            else
            {
                high_array[ i * 9 + curBit ] = ONE_BIT;
                low_array[ i * 9 + curBit ]  = WAIT_BIT - ONE_BIT;
            }

            curByte = curByte >> 1;
            curBit++;
        }
        low_array[ i * 9 + ( curBit - 1 ) ] = WAIT_BYTE;
    }

    cmdLength = serOutBuf->bufDataLen;
    HAL_TIM_Base_Start( &htim2 );
    TIM2_ConfigureOutputCompare_CH4();
}

// Function to configure TIM2 Channel 4 for Output Compare Toggle mode
void TIM2_ConfigureOutputCompare_CH4( void )
{
    // Disable channel 4 while configuring
    // TIM2->CCER &= ~TIM_CCER_CC4E;

    // Clear channel 3 configuration if it was enabled
    TIM2->CCER &= ~TIM_CCER_CC3E;
    TIM2->DIER &= ~TIM_DIER_CC3IE;    // Disable CH3 interrupt

    // Clear update flag
    TIM2->SR &= ~TIM_SR_UIF;

    // Configure Channel 4 as Output Compare
    // CCMR2 register: CC4S[1:0] = 00 (CC4 channel configured as output)
    TIM2->CCMR2 &= ~TIM_CCMR2_CC4S;    // Clear CC4S bits (00 = output)

    // Set Output Compare mode to Toggle (OC4M = 011)
    TIM2->CCMR2 &= ~TIM_CCMR2_OC4M;                  // Clear OC4M bits
    TIM2->CCMR2 |= ( 0x3 << TIM_CCMR2_OC4M_Pos );    // Set OC4M = 011 (Toggle)

    // Optional: Enable preload for CCR4
    TIM2->CCMR2 |= TIM_CCMR2_OC4PE;

    // Set initial compare value
    TIM2->CNT  = 0;
    TIM2->CCR4 = START_BIT;
    curr_match = 0;
    first_bit  = true;

    // Force an update event to ensure the compare value is loaded
    TIM2->EGR |= TIM_EGR_CC4G;    // Generate a capture/compare 4 event

    // Enable capture/compare 4 interrupt
    TIM2->DIER |= TIM_DIER_CC4IE;

    // Clear any pending interrupt flag
    TIM2->SR &= ~TIM_SR_CC4IF;

    // Enable channel 4 output
    TIM2->CCER |= TIM_CCER_CC4E;
}

void HAL_TIM_OC_DelayElapsedCallback( TIM_HandleTypeDef* htim )
{
    if ( htim->Instance == TIM2 )
    {
        uint8_t state  = HAL_GPIO_ReadPin( GPIOB, NFC_TXD_Pin );
        TIM2->CCER    |= TIM_CCER_CC4E;

        if ( ( state == 1 ) && ( !first_bit ) )
        {
            // curr_match = 0;
            PW = low_array[ sendbit ];
            sendbit++;

            if ( sendbit < ( 9 * cmdLength ) + 1)
            {
                TIM2->DIER &= ~TIM_DIER_UIE;
                // Generate update event to reset counter
                TIM2->EGR |= TIM_EGR_UG;
                // Clear the update flag that was set
                TIM2->SR &= ~TIM_SR_UIF;
                // Re-enable update interrupt if needed
                TIM2->DIER |= TIM_DIER_UIE;
                TIM2->CCR4  = PW;
            }
            else
            {
                // PW                = WAIT_BYTE;
                // TIM2->DIER       &= ~TIM_DIER_UIE;
                // TIM2->EGR        |= TIM_EGR_UG;
                // TIM2->SR         &= ~TIM_SR_UIF;
                // TIM2->DIER       |= TIM_DIER_UIE;
                // TIM2->CCR4        = PW;
                
                TIM2->DIER       &= ~TIM_DIER_CC4IE;    // Disable interrupt
                sConfigOC.OCMode  = TIM_OCMODE_FORCED_INACTIVE;
                HAL_TIM_OC_ConfigChannel( &htim2, &sConfigOC, TIM_CHANNEL_4 );
            }
        }
        else if ( ( state == 1 ) && ( first_bit ) )
        {
            PW          = low_array[ sendbit ];
            TIM2->DIER &= ~TIM_DIER_UIE;
            TIM2->EGR  |= TIM_EGR_UG;
            TIM2->SR   &= ~TIM_SR_UIF;
            TIM2->DIER |= TIM_DIER_UIE;
            TIM2->CCR4  = PW;
            first_bit   = false;
        }
        else if ( ( state == 0 ) && ( first_bit ) )
        {
            PW          = START_BIT;
            TIM2->DIER &= ~TIM_DIER_UIE;
            TIM2->EGR  |= TIM_EGR_UG;
            TIM2->SR   &= ~TIM_SR_UIF;
            TIM2->DIER |= TIM_DIER_UIE;
            TIM2->CCR4  = PW;
            first_bit   = true;
        }
        else
        {
            if ( sendbit < ( 9 * cmdLength ) )
            {
                // curr_match  = 1;
                PW          = high_array[ sendbit ];
                TIM2->DIER &= ~TIM_DIER_UIE;
                TIM2->EGR  |= TIM_EGR_UG;
                TIM2->SR   &= ~TIM_SR_UIF;
                TIM2->DIER |= TIM_DIER_UIE;
                TIM2->CCR4  = PW;
                first_bit   = false;
            }
            else
            {
                // PW                = WAIT_BYTE;
                // TIM2->DIER       &= ~TIM_DIER_UIE;
                // TIM2->EGR        |= TIM_EGR_UG;
                // TIM2->SR         &= ~TIM_SR_UIF;
                // TIM2->DIER       |= TIM_DIER_UIE;
                // TIM2->CCR4        = PW;

                TIM2->DIER       &= ~TIM_DIER_CC4IE;    // Disable interrupt
                sConfigOC.OCMode  = TIM_OCMODE_FORCED_INACTIVE;
                HAL_TIM_OC_ConfigChannel( &htim2, &sConfigOC, TIM_CHANNEL_4 );
            }
        }
    }
}

