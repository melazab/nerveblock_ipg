/// @file    FRAM.cpp
/// @brief   This module handles the SPI interface to the External FRAM.
/// @details Calibration values of Saturn 1 and Saturn 2 are stored in
/// 		 the external FRAM and are read upon power up.

#include "FRAM.hpp"
#include "stm32wbxx_hal.h"
#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "PortDefinitions.h"
#include "dio_stm32.hpp"
#include "spi.h"


uint8_t tempbuff[ 5 ];
unsigned int calvals[ 1024 ] = { 0 };

namespace FRAM
{
static SPI_HandleTypeDef handle;

// The chip select line. Active low.
static dio_stm32 chip_select { (std::uint32_t)FRAM_CSn_Port,
                               FRAM_CSn_Pin,
                               dio_stm32::Negative_Logic };
static dio_stm32 write_protect { (std::uint32_t)FRAM_WPn_Port,
                                 FRAM_WPn_Pin,
                                 dio_stm32::Negative_Logic };

static void peripheral_init( void );
}    // namespace FRAM

SPI_HandleTypeDef* FRAM::get_handle_ptr( void )
{
    return &FRAM::handle;
}

void FRAM::gpio_init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin       = FRAM_SCK_Pin | FRAM_MISO_Pin | FRAM_MOSI_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    FRAM::write_protect.deactivate();
    FRAM::chip_select.deactivate();

    return;
}

void FRAM::gpio_deinit( void )
{
    __HAL_RCC_SPI2_CLK_DISABLE();
    HAL_GPIO_DeInit( GPIOB, FRAM_SCK_Pin | FRAM_MISO_Pin | FRAM_MOSI_Pin );
    return;
}

static void FRAM::peripheral_init( void )
{
    FRAM::handle.Instance               = SPI2;
    FRAM::handle.Init.Mode              = SPI_MODE_MASTER;
    FRAM::handle.Init.Direction         = SPI_DIRECTION_2LINES;
    FRAM::handle.Init.DataSize          = SPI_DATASIZE_8BIT;
    FRAM::handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    FRAM::handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    FRAM::handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    FRAM::handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    FRAM::handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    FRAM::handle.Init.CRCPolynomial     = 10;
    FRAM::handle.Init.CRCPolynomial     = 7;
    FRAM::handle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    FRAM::handle.Init.NSS               = SPI_NSS_SOFT;
    FRAM::handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    FRAM::handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if ( HAL_SPI_Init( &FRAM::handle ) != HAL_OK )
    {
        Error_Handler();
    }

    return;
}

void FRAM::enable( void )
{
    FRAM::peripheral_init();
    return;
}

void FRAM::disable( void )
{
    FRAM::gpio_deinit();
    return;
}

// FRAM SPI opcode 0x06 = WREN (Write Enable Latch).  Must be sent before every
// write_single() call because the FRAM resets the WEL bit after each write.
void FRAM::write_enable( void )
{
    uint8_t opcode[ 1 ];
    opcode[ 0 ] = 0x06;
    FRAM::chip_select.activate();
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit( &FRAM::handle, (uint8_t*)opcode, 1, 500 );
    FRAM::chip_select.deactivate();
}

// FRAM SPI write frame (opcode 0x02 = WRITE): [0x02][addr_hi][addr_mid][addr_lo][data].
// 24-bit address space.  Caller must call write_enable() first.
void FRAM::write_single( unsigned int address, unsigned int data )
{
    uint8_t array[ 5 ];
    array[ 0 ] = 0x02;
    array[ 1 ] = ( address & 0xFF0000 ) >> 16;
    array[ 2 ] = ( address & 0xFF00 ) >> 8;
    array[ 3 ] = ( address & 0xFF );
    array[ 4 ] = data;

    FRAM::chip_select.activate();
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit( &FRAM::handle, (uint8_t*)array, 5, 500 );
    FRAM::chip_select.deactivate();

    FRAM::read_single( address );
}

// FRAM SPI read frame (opcode 0x03 = READ): TX [0x03][addr_hi][addr_mid][addr_lo],
// then receive 1 data byte.  The result is stored in calvals[address] for later
// use by Calibration().  tempbuff[4] holds the received byte.
void FRAM::read_single( unsigned int address )
{
    uint8_t array[ 4 ];
    array[ 0 ] = 0x03;
    array[ 1 ] = ( address & 0xFF0000 ) >> 16;
    array[ 2 ] = ( address & 0xFF00 ) >> 8;
    array[ 3 ] = ( address & 0xFF );

    FRAM::chip_select.activate();
    volatile HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive( &FRAM::handle,
                                                            (uint8_t*)array,
                                                            (uint8_t*)( tempbuff ),
                                                            5,
                                                            20000 );
    calvals[ address ]           = tempbuff[ 4 ];
    FRAM::chip_select.deactivate();
}

// Reads calibration values for both Saturn ASICs from FRAM into calvals[].
// Address layout convention: Saturn1 cal data lives at 0x1xx, Saturn2 at 0x2xx,
// where the lower byte mirrors the target Saturn register address.
void FRAM::ReadCalValues( void )
{
    read_single( 0x122 );    // Saturn1 reg 0x022 — current source trim
    read_single( 0x134 );    // Saturn1 reg 0x034 — compliance voltage trim
    read_single( 0x137 );    // Saturn1 reg 0x037 — oscillator trim low
    read_single( 0x138 );    // Saturn1 reg 0x038 — oscillator trim high

    read_single( 0x222 );    // Saturn2 reg 0x022 — current source trim
    read_single( 0x234 );    // Saturn2 reg 0x034 — compliance voltage trim
    read_single( 0x237 );    // Saturn2 reg 0x037 — oscillator trim low
    read_single( 0x238 );    // Saturn2 reg 0x038 — oscillator trim high
}
