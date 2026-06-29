/// @file    Impedance.cpp
/// @brief   This module handles the I2C interface to the
///          external impedance muxes.
/// @details Switches on the impedance muxes close when measuring
///          the impedance of two channels

#include "Impedance.hpp"
#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"
#include "CommandHandler.hpp"
#include "PortDefinitions.h"
#include "Saturn2_spi.hpp"
#include "Saturn_spi.hpp"
#include "dio_stm32.hpp"
#include "main.h"
#include "stm32wbxx_hal.h"
#include <adc.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define XferSize             21320
#define NumberOfMeasChannels 2
#define PrintOutLen          22
#define DIFFAMPOFFSET        1650
uint8_t rbuff;
bool measuringimp            = false;
uint16_t samples_in_duration = 2667;

/// Module Variable Definitions
namespace Impedance
{

static constexpr uint8_t slave1 = 0x4C;
static constexpr uint8_t slave2 = 0x4F;

volatile uint16_t uhADCxConvertedValue[ XferSize * NumberOfMeasChannels ] = { 0 };
static uint16_t impMeasBytes                                              = 0;
static bool imp_mux_value                                                 = 1;
// static const uint16_t ByteRatePerChPerMs                                  = 2132;
#define uS_per_meas_cycle 0.9375f    // 30 ADC cycles * (1 / ADC_CLK)
bool impedance_ongoing = false;

uint8_t channeltoswitch[ 27 ] = { 0,  13, 12, 11, 10, 9,  1,  2, 3, 4, 5, 6, 7, 8,
                                  16, 15, 14, 13, 12, 11, 10, 9, 1, 2, 3, 4, 5 };


}    // namespace Impedance

// Powers up the impedance measurement front-end:
//   IMP_ENn low  = enable analog supply to impedance circuitry
//   IMP_PDn high = release from power-down
//   IMP_MUX_SEL  = select which differential input pair (ch1 or ch3/4)
void Impedance::start_up_seq( void )
{
    ADC1_Init_IMP();
    HAL_GPIO_WritePin( IMP_ENn_Port, IMP_ENn_Pin, GPIO_PIN_RESET );
    HAL_Delay( 100 );
    HAL_GPIO_WritePin( IMP_PDn_Port, IMP_PDn_Pin, GPIO_PIN_SET );
    HAL_Delay( 100 );
    imp_mux_value ? HAL_GPIO_WritePin( IMP_MUX_SEL_Port, IMP_MUX_SEL_Pin, GPIO_PIN_SET )
                  : HAL_GPIO_WritePin( IMP_MUX_SEL_Port, IMP_MUX_SEL_Pin, GPIO_PIN_RESET );

    HAL_NVIC_EnableIRQ( DMA2_Channel1_IRQn );
}

void Impedance::pwr_dwn_seq( void )
{
    HAL_GPIO_WritePin( IMP_MUX_SEL_Port, IMP_MUX_SEL_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( IMP_PDn_Port, IMP_PDn_Pin, GPIO_PIN_RESET );
    HAL_Delay( 100 );
    HAL_GPIO_WritePin( IMP_ENn_Port, IMP_ENn_Pin, GPIO_PIN_SET );
    HAL_Delay( 100 );
}

void Impedance::CloseSwitches( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == mapping[ 25 ] )    // Electrode is mapped to ch1
    {
        imp_mux_value = 1;
    }
    else    // Assume electrode is mapped to ch3 and ch4
    {
        imp_mux_value = 0;
    }
}

void Impedance::StartMeasurement( CommandData_T* cmd )
{
    ULONG actual_flags;

    if ( !impedance_ongoing )
    {
        Impedance::start_up_seq();

        impedance_ongoing = true;

        samples_in_duration = duration2 * 1000 / uS_per_meas_cycle;

        memset( (void*)uhADCxConvertedValue, 0x00, sizeof( uhADCxConvertedValue ) );

        if ( HAL_ADC_Start_DMA( &hadc1,
                                (uint32_t*)&uhADCxConvertedValue,
                                ( samples_in_duration * 2 ) )
             != HAL_OK )
        {
            Error_Handler();
        }

        //_tx_thread_sleep( 1 );

        Saturn2_spi::StartContinuousStim();

        tx_event_flags_get( &data_ready,
                            ADC_READY_FLAG,
                            TX_OR_CLEAR,
                            &actual_flags,
                            TX_WAIT_FOREVER );

        if ( HAL_ADC_Stop_DMA( &hadc1 ) != HAL_OK )
        {
            Error_Handler();
        }
        //_tx_thread_sleep( duration2 );

        Saturn2_spi::StopContinuousStim();

        Impedance::StopMeasurement();
    }
}

void Impedance::StopMeasurement( void )
{
    impMeasBytes = samples_in_duration * 2;

    Impedance::impMeasStreamDataTask();
}

// Packetises and streams ADC impedance data back to the host as CT_ImpedanceData
// responses.  Each 130-byte packet carries:
//   bytes 0-1: packet index (MSB set on the last packet to signal end-of-stream)
//   bytes 2-129: 64 signed int16 differential voltage samples in mV
//
// Differential conversion: (ADC_A - ADC_B) * gain * Vref/counts * 1000
//   where gain=4.5, Vref=3.3V, counts=4095  =>  ~3.623 mV per ADC LSB
// ADC channels A and B are the two outputs of the differential amplifier
// connected across the impedance-under-test.
static void Impedance::impMeasStreamDataTask( void )
{
    static int8_t telmBuffer[ 130 ];
    uint16_t nPcksToSend = ( Impedance::impMeasBytes / 128 ) + 1;

    for ( uint16_t pckCnt = 0; pckCnt < nPcksToSend; pckCnt++ )
    {
        if ( pckCnt == ( nPcksToSend - 1 ) )
        {
            *( (uint16_t*)telmBuffer ) = pckCnt | 0x8000;    // Mark last packet
        }
        else
        {
            *( (uint16_t*)telmBuffer ) = pckCnt;
        }
        int16_t* samplePtr = (int16_t*)( &( telmBuffer[ 2 ] ) );
        int16_t* dataPtr   = (int16_t*)( &( Impedance::uhADCxConvertedValue[ pckCnt * 128 ] ) );
        for ( int sample = 0; sample < 64; sample++ )
        {
            uint16_t impA = *dataPtr;
            dataPtr++;
            uint16_t impB = *dataPtr;
            dataPtr++;
            // Convert raw ADC counts to mV: differential * gain * Vref/counts * 1000
            *samplePtr = ( impA - impB ) * ( 4.5 ) * ( 3.3 / 4095 ) * ( 1000 );
            samplePtr++;
        }

        SendCmdResponse( CT_ImpedanceData, CMD_RESP_ACK, telmBuffer, 130 );
        _tx_thread_sleep( 10 );
    }
    impedance_ongoing = false;
    //Impedance::pwr_dwn_seq();
}
