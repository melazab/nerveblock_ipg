#include "CommandHandler.hpp"
#include "PortDefinitions.h"
#include "inductive_link_comm.h"
#include "stm32wbxx.h"
#include "tx_api.h"
#include <adc.h>


#define XferSize    6
#define VREFINT_CAL ( *( (uint16_t*)VREFINT_CAL_ADDR ) )
volatile uint16_t AD_VMEAS[ XferSize * 2 ] = { 0 };
volatile float actual_vdda                 = 0.0f;

void batt_charging_entry( ULONG treadInput )
{
    static CommandData_T cmd;
    static SERIAL_DATA_OUT_BUF_T serOutBuf;
    float vbat, vrect;
    static bool charging = false;

    for ( ;; )
    {
        meas_voltages( &vbat, &vrect );

        if ( vrect >= 2.7f && vbat < 4.2f )
        {
            HAL_GPIO_WritePin( CHG_EN_Port, CHG_EN_Pin, GPIO_PIN_SET );
            charging = true;
        }
        else
        {
            HAL_GPIO_WritePin( CHG_EN_Port, CHG_EN_Pin, GPIO_PIN_RESET );
            charging = false;
        }

        uint8_t v_bat
            = (uint8_t)( ( vbat - 3.0f ) * 50.0f + 0.5f ); /* 1/0.02 = 50, +0.5 for rounding */
        uint8_t v_rect = (uint8_t)( ( vrect - 1.2f ) * ( 1 / 0.6f ) + 0.5f );

        cmd.cmdDataLen      = 2;
        cmd.cmdDataBuf[ 0 ] = 0;
        cmd.cmdDataBuf[ 1 ] = 0;

        cmd.cmdDataBuf[ 0 ] |= ( v_rect << 5 );
        cmd.cmdDataBuf[ 0 ] |= ( 2 << 2 );
        cmd.cmdDataBuf[ 0 ] |= 3;

        cmd.cmdDataBuf[ 1 ] |= ( charging << 6 );
        cmd.cmdDataBuf[ 1 ] |= ( v_bat );

        build_status_command( cmd.cmdDataBuf, &serOutBuf );
        timIntSendSerialDataBuf( &serOutBuf );

        tx_thread_sleep( 100 );
    }
}

void ADC1_VREF( void )
{
    uint32_t vref = 0;
    ULONG actual_flags;

    ADC_ChannelConfTypeDef sConfig = { 0 };

    sConfig.Channel      = ADC_CHANNEL_VREFINT;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }

    HAL_NVIC_EnableIRQ( DMA2_Channel1_IRQn );

    memset( (void*)AD_VMEAS, 0x00, sizeof( AD_VMEAS ) );

    HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED );

    if ( HAL_ADC_Start_DMA( &hadc1, (uint32_t*)&AD_VMEAS, XferSize ) != HAL_OK )
    {
        Error_Handler();
    }

    tx_event_flags_get( &data_ready, ADC_READY_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER );

    if ( HAL_ADC_Stop_DMA( &hadc1 ) != HAL_OK )
    {
        Error_Handler();
    }

    actual_vdda = 3.6f * VREFINT_CAL / AD_VMEAS[ 0 ];
}

void ADC1_Init_BATT( void )
{
    HAL_NVIC_EnableIRQ( DMA2_Channel1_IRQn );

    HAL_GPIO_WritePin( VRECT_MON_EN_Port, VRECT_MON_EN_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( BATTMON_EN_Port, BATTMON_EN_Pin, GPIO_PIN_SET );

    tx_thread_sleep( 50 );

    ADC_ChannelConfTypeDef sConfig = { 0 };

    sConfig.Channel      = ADC_CHANNEL_3;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_8;
    sConfig.Rank         = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    if ( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK )
    {
        Error_Handler();
    }
}

void meas_voltages( float* vbat_ptr, float* vrect_ptr )
{
    uint32_t v_avg_rec = 0;
    uint32_t v_avg_bat = 0;
    ULONG actual_flags;

    ADC1_Init_BATT();

    memset( (void*)AD_VMEAS, 0x00, sizeof( AD_VMEAS ) );

    HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED );

    if ( HAL_ADC_Start_DMA( &hadc1, (uint32_t*)&AD_VMEAS, XferSize ) != HAL_OK )
    {
        Error_Handler();
    }

    tx_event_flags_get( &data_ready, ADC_READY_FLAG, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER );

    if ( HAL_ADC_Stop_DMA( &hadc1 ) != HAL_OK )
    {
        Error_Handler();
    }

    for ( int i = 0; i < 6; i+=2 )
    {
        v_avg_bat += ( AD_VMEAS[ i ] );
        v_avg_rec += ( AD_VMEAS[ i + 1 ] );
    }

    v_avg_bat /= 3;
    v_avg_rec /= 3;

    *vbat_ptr  = ( (uint16_t)v_avg_bat * actual_vdda * 2.0f ) / ( 4095.0f );
    *vrect_ptr = ( (uint16_t)v_avg_rec * actual_vdda * 2.0f ) / ( 4095.0f );

    HAL_GPIO_WritePin( VRECT_MON_EN_Port, VRECT_MON_EN_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( BATTMON_EN_Port, BATTMON_EN_Pin, GPIO_PIN_RESET );
}
