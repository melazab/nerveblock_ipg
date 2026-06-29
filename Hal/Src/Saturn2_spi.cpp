/// @file    Saturn2_spi.cpp
/// @brief   This module handles the SPI interface to the
///          "Saturn 2" or "Low Frequency" ASIC.
/// @details Handles the power up sequence of the Saturn chip,
/// 		 contains the functions to control stim parameters.

#include "Saturn2_spi.hpp"
#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"
#include "CommandHandler.hpp"
#include "FRAM.hpp"
#include "Impedance.hpp"
#include "PortDefinitions.h"
#include "Saturn_spi.hpp"
// #include "assertLocal.h"
#include "main.h"
#include "stm32wbxx_hal.h"
#include <bit>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unordered_map>
#include <vector>



volatile unsigned int count2                             = 0;    // General Counter
unsigned int ARG7_2                                      = 0;
unsigned int ARG6_2                                      = 0;
unsigned int STIM_UP2                                    = 0;    // Status of STIM USCI interface
uint8_t mapping2[ 27 ] __attribute__( ( aligned( 4 ) ) ) = { 0 };
unsigned int startdelay2                                 = 0;
uint32_t PW2                                             = 10;
uint32_t PW2phase2                                       = 10;
uint16_t f2                                              = 2;
uint32_t delay1reg2                                      = 0;
int64_t starttick2                                       = -2;
uint32_t duration2                                       = 0;
uint32_t offduration2                                    = 0;
uint32_t stoppoint2                                      = 0;
int64_t ogtick2                                          = 0;
uint16_t chargelimits2[ 5 ]                              = { 0, 25, 25, 25, 25 };
uint16_t AMP2ph1[ 27 ]                                   = { 0 };
uint16_t AMP2ph2[ 27 ]                                   = { 0 };

static constexpr uint16_t seq0a_2     = 0x80;     // bank 2 	(0x01)
static constexpr uint16_t seq0b_2     = 0xC0;     // bank 3	(0x02)
static constexpr uint16_t seq0delay_2 = 0x100;    // bank 4	(0x03)
static constexpr uint16_t seq1a_2     = 0x200;    // bank 6	(0x05)
static constexpr uint16_t seq1b_2     = 0x240;    // bank 7	(0x06)
static constexpr uint16_t seq1delay_2 = 0x280;    // bank 8	(0x07)

namespace Saturn2_spi
{
static SPI_HandleTypeDef handle;

// The chip select line. Active low.
static dio_stm32 chip_select { (std::uint32_t)LF_STIM_CSn_Port,
                               LF_STIM_CSn_Pin,
                               dio_stm32::Negative_Logic };
static dio_stm32 dvddio_en { (std::uint32_t)DVDDIO_EN_Port,
                             DVDDIO_EN_Pin,
                             dio_stm32::Positive_Logic };
static dio_stm32 avdd_en { (std::uint32_t)AVDD_EN_Port, AVDD_EN_Pin, dio_stm32::Positive_Logic };
static dio_stm32 hvdd_en { (std::uint32_t)SAT_VHV_SW_Port,
                           SAT_VHV_SW_Pin,
                           dio_stm32::Positive_Logic };
static dio_stm32 sat_boost_en { (std::uint32_t)SAT_BOOST_SW_Port,
                                SAT_BOOST_SW_Pin,
                                dio_stm32::Positive_Logic };
}    // namespace Saturn2_spi

SPI_HandleTypeDef* Saturn2_spi::get_handle_ptr( void )
{
    return &Saturn2_spi::handle;
}

void Saturn2_spi::enable( void )
{
    static bool spi_is_initialized = false;

    // We only need to init the SPI peripheral one time
    if ( !spi_is_initialized )
    {
        Saturn2_spi::peripheral_init();
        spi_is_initialized = true;
    }
    Saturn2_spi::gpio_init();
    Saturn2_spi::chip_select.deactivate();
}

void Saturn2_spi::peripheral_init()
{
    Saturn2_spi::handle.Instance               = SPI2;
    Saturn2_spi::handle.Init.Mode              = SPI_MODE_MASTER;
    Saturn2_spi::handle.Init.Direction         = SPI_DIRECTION_2LINES;
    Saturn2_spi::handle.Init.DataSize          = SPI_DATASIZE_8BIT;
    Saturn2_spi::handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    Saturn2_spi::handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    Saturn2_spi::handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    Saturn2_spi::handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    Saturn2_spi::handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    Saturn2_spi::handle.Init.CRCPolynomial     = 10;
    Saturn2_spi::handle.Init.CRCPolynomial     = 7;
    Saturn2_spi::handle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    Saturn2_spi::handle.Init.NSS               = SPI_NSS_SOFT;
    Saturn2_spi::handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    Saturn2_spi::handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    // 96 Mhz Main Clock / 1 / 8 = 12.0 Mhz Clock
    // Max SPI clock is 50 Mhz so we could go a little higher
    if ( HAL_SPI_Init( &Saturn2_spi::handle ) != HAL_OK )
    {
        Error_Handler();
    }

    return;
}

void Saturn2_spi::gpio_init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin       = LF_STIM_SCK_Pin | LF_STIM_MISO_Pin | LF_STIM_MOSI_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );

    Saturn2_spi::chip_select.deactivate();

    return;
}

void Saturn2_spi::gpio_deinit( void )
{
    __HAL_RCC_SPI2_CLK_DISABLE();

    HAL_GPIO_DeInit( GPIOD, LF_STIM_SCK_Pin | LF_STIM_MISO_Pin | LF_STIM_MOSI_Pin );

    return;
}

void Saturn2_spi::STIM_POWER_UP( void )    // Bring up power to Saturn
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin   = DVDDIO_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init( DVDDIO_EN_Port, &GPIO_InitStruct );

    Saturn2_spi::dvddio_en.activate();

    STIM_UP2 = 1;    // Store status of STIM SPI interface
}

void Saturn2_spi::PULSE_STIM_CLK( unsigned int pulses )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_GPIOE_CLK_ENABLE();
    count2 = 0;
    while ( count2 < pulses )    // Do Pulse Train
    {
        GPIO_InitStruct.Pin   = LF_STIM_CLK_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init( LF_STIM_CLK_Port, &GPIO_InitStruct );

        HAL_GPIO_WritePin( LF_STIM_CLK_Port, LF_STIM_CLK_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin( LF_STIM_CLK_Port, LF_STIM_CLK_Pin, GPIO_PIN_RESET );

        count2++;
    }
}

void Saturn2_spi::PULSE_STIM_SCK( unsigned int pulses )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if ( STIM_UP2 > 0 )
    {
        GPIO_InitStruct.Pin   = LF_STIM_SCK_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );
        HAL_GPIO_WritePin( GPIOD, LF_STIM_SCK_Pin, GPIO_PIN_RESET );
    }
    count2 = 0;
    while ( count2 < pulses )    // Do Pulse Train
    {
        HAL_GPIO_WritePin( GPIOD, LF_STIM_SCK_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin( GPIOD, LF_STIM_SCK_Pin, GPIO_PIN_RESET );

        count2++;
    }
    if ( STIM_UP2 > 0 )
    {
        GPIO_InitStruct.Pin       = LF_STIM_SCK_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init( LF_STIM_SCK_Port, &GPIO_InitStruct );
    }
}

void Saturn2_spi::SATURN_WRITE_SINGLE( unsigned int address, unsigned int data )
{
    unsigned int highaddr = ( address & 0xFF00 ) >> 8;
    unsigned int lowaddr  = ( address & 0xFF );
    HAL_StatusTypeDef hal_status;

    uint8_t array[ 3 ];
    array[ 0 ] = highaddr;
    array[ 1 ] = lowaddr;
    array[ 2 ] = data;

    Saturn2_spi::gpio_init();
    Saturn2_spi::chip_select.activate();
    hal_status = HAL_SPI_Transmit( &Saturn2_spi::handle, (uint8_t*)array, 3, 500 );
    Saturn2_spi::chip_select.deactivate();
    Saturn2_spi::gpio_deinit();
}

void Saturn2_spi::SATURN_READ_SINGLE( unsigned int address )
{
    unsigned int highaddr  = ( address & 0xFF00 ) >> 8;
    unsigned int lowaddr   = ( address & 0xFF );
    highaddr              |= 0x80;
    HAL_StatusTypeDef hal_status;

    uint8_t array[ 2 ];
    array[ 0 ] = highaddr;
    array[ 1 ] = lowaddr;

    Saturn2_spi::gpio_init();
    Saturn2_spi::chip_select.activate();
    hal_status = HAL_SPI_Transmit( &Saturn2_spi::handle, (uint8_t*)array, 2, 500 );

    if ( hal_status == HAL_OK )
    {
        hal_status = HAL_SPI_Receive( &Saturn2_spi::handle, (uint8_t*)( &ARG7_2 ), 4, 500 );
        ARG7_2     = ( ARG7_2 & 0xFF00 ) >> 8;
    }
    Saturn2_spi::chip_select.deactivate();
    Saturn2_spi::gpio_deinit();
}

CmdHandlerReturnCode_T Saturn2_spi::ReadReturn( CommandData_T* cmd )
{
    uint16_t address = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 );
    Saturn2_spi::SATURN_READ_SINGLE( address );
    cmd->cmdDataBuf[ 0 ] = ( address & 0xFF00 ) >> 8;
    cmd->cmdDataBuf[ 1 ] = ( address & 0xFF );
    cmd->cmdDataBuf[ 2 ] = ARG7_2;
    cmd->cmdDataLen      = 3;
}

void Saturn2_spi::SATURN_WRITE_MASK( unsigned int address, unsigned int data, unsigned int mask )
{
    SATURN_READ_SINGLE( address );    // Fetch existing register to ARG7_2
    SATURN_WRITE_SINGLE(
        address,
        ( ( ARG7_2 &= ~mask ) | ( data & mask ) ) );    // Apply masked update to register
}

void Saturn2_spi::SATURN_DIG_STARTUP( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    HAL_GPIO_WritePin( LF_STIM_RSTn_Port, LF_STIM_RSTn_Pin, GPIO_PIN_RESET );    // Set RST_N low

    GPIO_InitStruct.Pin  = LF_STIM_STOP_Pin | LF_STIM_RUN_Pin | LF_STIM_SEL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = LF_STIM_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( LF_STIM_CONT_Port, &GPIO_InitStruct );

    HAL_Delay( 50 );    // Wait during pull-up interval
    STIM_POWER_UP();    // Apply DVDD_STIM

    HAL_Delay( 3 );
    Saturn2_spi::enable();    // Init SPI lines
    PULSE_STIM_SCK( 4 );      // Toggle SCK 4x

    HAL_Delay( 3 );
    Saturn2_spi::avdd_en.activate();    // Set AVDD high
    HAL_Delay( 3 );

    GPIO_InitStruct.Pin  = LF_STIM_STOP_Pin | LF_STIM_RUN_Pin | LF_STIM_SEL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = LF_STIM_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( LF_STIM_CONT_Port, &GPIO_InitStruct );

    HAL_GPIO_WritePin( LF_STIM_RSTn_Port, LF_STIM_RSTn_Pin, GPIO_PIN_SET );    // Set RST_N high

    PULSE_STIM_CLK( 8 );    // 8 CLK cycles to exit reset
    PULSE_STIM_SCK( 4 );    // Toggle SCK 4x again

    HAL_Delay( 3 );
    SATURN_WRITE_SINGLE( 0x06, 0xFF );     // WE5 register
    SATURN_WRITE_SINGLE( 0x01, 0x87 );     // ENABLE register
    SATURN_WRITE_SINGLE( 0x033, 0xC9 );    // STIM_OSC_CFG
    PULSE_STIM_CLK( 72 );                  // Pulse train

    HAL_Delay( 3 );
    SATURN_WRITE_SINGLE( 0x02, 0xFF );     // WE1
    SATURN_WRITE_SINGLE( 0x03, 0xFF );     // WE2
    SATURN_WRITE_SINGLE( 0x04, 0xFF );     // WE3
    SATURN_WRITE_SINGLE( 0x05, 0xFF );     // WE4
    SATURN_WRITE_SINGLE( 0x06, 0xFF );     // WE5

    SATURN_WRITE_SINGLE( 0x1F, 0x18 );     // VG_CTRL <- Enable RAMS
    SATURN_WRITE_SINGLE( 0x800, 0x00 );    // Write 0x00 to RAM1[0]
    SATURN_READ_SINGLE( 0x800 );           // RAM1[0] -> ARG7_2
    SATURN_WRITE_SINGLE( 0x900, 0x00 );    // Write 0x00 to RAM2[0]
    SATURN_READ_SINGLE( 0x900 );           // RAM2[0] -> ARG7_2
}

void Saturn2_spi::SATURN_BBC_STARTUP_R1( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    int START_COUNT = 0;    // BBC Startup Loop Counter

    GPIO_InitStruct.Pin   = SAT_VHV_SW_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    Saturn2_spi::hvdd_en.activate();    // VHV_SW enabled
    Saturn2_spi::sat_boost_en.activate();

    HAL_Delay( 10 );                      //~10mS settling delay
    SATURN_WRITE_SINGLE( 0x24, 0x81 );    // CG_CTRL <- 0x88 (Enable Current Sources)

    // SATURN_WRITE_MASK (0x01,0x03,0x03);				//
    // ENABLE <- Bits 0x02 & 0x01 (Enable Bandgap, BC)
    HAL_Delay( 2 );                       //~2mS settling delay
    SATURN_WRITE_SINGLE( 0x20, 0x0C );    // BC_CTRL0 <- 0x0C
    SATURN_WRITE_SINGLE( 0x21, 0x0B );    // BC_CTRL1 <- 0x0B

    HAL_Delay( 1 );                       //~1mS settling delay
    SATURN_WRITE_SINGLE( 0x21, 0x1B );    // BC_CTRL1 <- 0x1B

    HAL_Delay( 10 );                      // Wait 10mS
    ARG7_2 = 0;

    while ( ( START_COUNT < 100 ) &&          // Loop until we've started up, or tried
            ( !( ARG7_2 && 0x80 ) ) )         // 	too many times:
    {
        SATURN_READ_SINGLE( 0x21 );           // Get BC_CTRL1 > ARG7_2
        HAL_Delay( 1 );                       // Wait 1mS
        START_COUNT += 1;                     // Increment Loop Ctr
    }
    ARG6_2 = START_COUNT;                     // Report Loop Ctr for Monitoring
    if ( ( ARG7_2 && 0x80 ) )                 // Is BCG set?
    {
        SATURN_WRITE_SINGLE( 0x1F, 0x02 );    // VG_CTRL <- 0x02 (HVDDL Enable)
        HAL_Delay( 2 );                       // ~2mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x06 );    // VG_CTRL <- Bit 0x04 (Assert LV Reset)
        HAL_Delay( 1 );                       // ~1mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x02 );    // VG_CTRL <- Bit 0x04 (Clear LV Reset)
        HAL_Delay( 1 );                       // ~1mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x03 );    // VG_CTRL <- Bit 0x01 (CAS Enable)
        HAL_Delay( 2 );                       // ~2mS settling time
        ARG7_2 = 0;                           // Return Success
    }
}

void Saturn2_spi::read_registers( CommandData_T* cmd )
{
    //std::ofstream outFile("output.txt");

    for ( int address = 0; address < 0x640; address++ )
    {
        Saturn2_spi::SATURN_READ_SINGLE( address );
        cmd->cmdDataBuf[ 0 ] = ( address & 0xFF00 ) >> 8;
        cmd->cmdDataBuf[ 1 ] = ( address & 0xFF );
        cmd->cmdDataBuf[ 2 ] = ARG7_2;
        cmd->cmdDataLen      = 3;

        //outFile << "%d, " << address << "%d" << ARG7 << std::endl;
        //printf("%d, %d\n", address, ARG7);
        SendUCmdResponse( CT_ReadRegisters, CMD_RESP_ACK, cmd->cmdDataBuf, cmd->cmdDataLen );
        HAL_Delay( 25 );
    }
       //outFile.close();
}


void Saturn2_spi::DEFAULT_STIM( void )
{
    mapping2[12] = 1;
    mapping2[9] = 1;
    mapping2[5] = 2;
    mapping2[3] = 2;
    SATURN_WRITE_SINGLE( 0x0006, 0xFF );
    SATURN_WRITE_SINGLE( 0x0001, 0x87 );
    SATURN_WRITE_SINGLE( 0x0002, 0xFF );
    SATURN_WRITE_SINGLE( 0x0003, 0xFF );
    SATURN_WRITE_SINGLE( 0x0004, 0xFF );
    SATURN_WRITE_SINGLE( 0x0005, 0xFF );
    SATURN_WRITE_SINGLE( 0x0006, 0xFF );
    SATURN_WRITE_SINGLE( 0x0007, 0x00 );
    SATURN_WRITE_SINGLE( 0x0008, 0x00 );
    SATURN_WRITE_SINGLE( 0x0009, 0x00 );
    SATURN_WRITE_SINGLE( 0x000A, 0x00 );
    SATURN_WRITE_SINGLE( 0x000B, 0x00 );
    SATURN_WRITE_SINGLE( 0x000C, 0x01 );    // SEQ0A
    SATURN_WRITE_SINGLE( 0x000D, 0x02 );    // SEQ0B
    SATURN_WRITE_SINGLE( 0x000E, 0x07 );    // SEQ1A
    SATURN_WRITE_SINGLE( 0x000F, 0x08 );    // SEQ1B
    SATURN_WRITE_SINGLE( 0x0010, 0xFF );
    SATURN_WRITE_SINGLE( 0x0011, 0x00 );
    SATURN_WRITE_SINGLE( 0x0012, 0x00 );
    SATURN_WRITE_SINGLE( 0x0013, 0x00 );
    SATURN_WRITE_SINGLE( 0x0014, 0x00 );
    SATURN_WRITE_SINGLE( 0x0015, 0x00 );
    SATURN_WRITE_SINGLE( 0x0020, 0x3F );
    SATURN_WRITE_SINGLE( 0x0021, 0x1C );
    SATURN_WRITE_SINGLE( 0x0025, 0x0F );
    SATURN_WRITE_SINGLE( 0x0036, 0xFF );
    SATURN_WRITE_SINGLE( 0x0023, 0x00 );
    SATURN_WRITE_SINGLE( 0x0034, 0x7F );
    SATURN_WRITE_SINGLE( 0x0035, 0x00 );
    SATURN_WRITE_SINGLE( 0x0040, 0x00 );
    SATURN_WRITE_SINGLE( 0x0041, 0x10 );
    SATURN_WRITE_SINGLE( 0x0042, 0x99 );
    SATURN_WRITE_SINGLE( 0x0043, 0x99 );
    SATURN_WRITE_SINGLE( 0x0044, 0x19 );
    SATURN_WRITE_SINGLE( 0x0045, 0x00 );
    SATURN_WRITE_SINGLE( 0x0046, 0x0A );    // DLYL
    SATURN_WRITE_SINGLE( 0x0047, 0x00 );    // DLYU
    SATURN_WRITE_SINGLE( 0x0048, 0x00 );
    SATURN_WRITE_SINGLE( 0x0049, 0x00 );
    SATURN_WRITE_SINGLE( 0x004A, 0x04 );
    SATURN_WRITE_SINGLE( 0x004B, 0x00 );
    SATURN_WRITE_SINGLE( 0x004C, 0x04 );
    SATURN_WRITE_SINGLE( 0x004D, 0x00 );
    SATURN_WRITE_SINGLE( 0x004E, 0x04 );
    SATURN_WRITE_SINGLE( 0x004F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0050, 0x04 );
    SATURN_WRITE_SINGLE( 0x0051, 0x00 );
    SATURN_WRITE_SINGLE( 0x0052, 0x04 );
    SATURN_WRITE_SINGLE( 0x0053, 0x00 );
    SATURN_WRITE_SINGLE( 0x0054, 0x04 );
    SATURN_WRITE_SINGLE( 0x0055, 0x00 );
    SATURN_WRITE_SINGLE( 0x0056, 0x04 );
    SATURN_WRITE_SINGLE( 0x0057, 0x00 );
    SATURN_WRITE_SINGLE( 0x0058, 0x04 );
    SATURN_WRITE_SINGLE( 0x0059, 0x00 );
    SATURN_WRITE_SINGLE( 0x005A, 0x04 );
    SATURN_WRITE_SINGLE( 0x005B, 0x00 );
    SATURN_WRITE_SINGLE( 0x005C, 0x04 );
    SATURN_WRITE_SINGLE( 0x005D, 0x00 );
    SATURN_WRITE_SINGLE( 0x005E, 0x04 );
    SATURN_WRITE_SINGLE( 0x005F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0060, 0x04 );
    SATURN_WRITE_SINGLE( 0x0061, 0x00 );
    SATURN_WRITE_SINGLE( 0x0062, 0x04 );
    SATURN_WRITE_SINGLE( 0x0063, 0x00 );
    SATURN_WRITE_SINGLE( 0x0064, 0x04 );
    SATURN_WRITE_SINGLE( 0x0065, 0x00 );
    SATURN_WRITE_SINGLE( 0x0066, 0x04 );
    SATURN_WRITE_SINGLE( 0x0067, 0x00 );
    SATURN_WRITE_SINGLE( 0x0068, 0x04 );
    SATURN_WRITE_SINGLE( 0x0069, 0x00 );
    SATURN_WRITE_SINGLE( 0x006A, 0x04 );
    SATURN_WRITE_SINGLE( 0x006B, 0x00 );
    SATURN_WRITE_SINGLE( 0x006C, 0x04 );
    SATURN_WRITE_SINGLE( 0x006D, 0x00 );
    SATURN_WRITE_SINGLE( 0x006E, 0x04 );
    SATURN_WRITE_SINGLE( 0x006F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0070, 0x04 );
    SATURN_WRITE_SINGLE( 0x0071, 0x00 );
    SATURN_WRITE_SINGLE( 0x0072, 0x04 );
    SATURN_WRITE_SINGLE( 0x0073, 0x00 );
    SATURN_WRITE_SINGLE( 0x0074, 0x04 );
    SATURN_WRITE_SINGLE( 0x0075, 0x00 );
    SATURN_WRITE_SINGLE( 0x0076, 0x04 );
    SATURN_WRITE_SINGLE( 0x0077, 0x00 );
    SATURN_WRITE_SINGLE( 0x0078, 0x04 );
    SATURN_WRITE_SINGLE( 0x0079, 0x00 );
    SATURN_WRITE_SINGLE( 0x007A, 0x04 );
    SATURN_WRITE_SINGLE( 0x007B, 0x00 );
    SATURN_WRITE_SINGLE( 0x007C, 0x04 );
    SATURN_WRITE_SINGLE( 0x007D, 0x00 );
    SATURN_WRITE_SINGLE( 0x0080, 0x00 );
    SATURN_WRITE_SINGLE( 0x0081, 0x00 );
    SATURN_WRITE_SINGLE( 0x0082, 0x99 );
    SATURN_WRITE_SINGLE( 0x0083, 0x99 );
    SATURN_WRITE_SINGLE( 0x0084, 0x19 );
    SATURN_WRITE_SINGLE( 0x0085, 0x00 );
    SATURN_WRITE_SINGLE( 0x0086, 0x00 );    // DLYL
    SATURN_WRITE_SINGLE( 0x0087, 0x00 );    // DLYU
    SATURN_WRITE_SINGLE( 0x0088, 0x00 );
    SATURN_WRITE_SINGLE( 0x0089, 0x00 );
    SATURN_WRITE_SINGLE( 0x008A, 0x04 );
    SATURN_WRITE_SINGLE( 0x008B, 0x00 );
    SATURN_WRITE_SINGLE( 0x008C, 0x04 );
    SATURN_WRITE_SINGLE( 0x008D, 0x00 );
    SATURN_WRITE_SINGLE( 0x008E, 0x04 );
    SATURN_WRITE_SINGLE( 0x008F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0090, 0x04 );
    SATURN_WRITE_SINGLE( 0x0091, 0x00 );
    SATURN_WRITE_SINGLE( 0x0092, 0x04 );
    SATURN_WRITE_SINGLE( 0x0093, 0x00 );
    SATURN_WRITE_SINGLE( 0x0094, 0x04 );
    SATURN_WRITE_SINGLE( 0x0095, 0x00 );
    SATURN_WRITE_SINGLE( 0x0096, 0x04 );
    SATURN_WRITE_SINGLE( 0x0097, 0x00 );
    SATURN_WRITE_SINGLE( 0x0098, 0x04 );
    SATURN_WRITE_SINGLE( 0x0099, 0x00 );
    SATURN_WRITE_SINGLE( 0x009A, 0x04 );
    SATURN_WRITE_SINGLE( 0x009B, 0x00 );
    SATURN_WRITE_SINGLE( 0x009C, 0x04 );
    SATURN_WRITE_SINGLE( 0x009D, 0x00 );
    SATURN_WRITE_SINGLE( 0x009E, 0x04 );
    SATURN_WRITE_SINGLE( 0x009F, 0x00 );
    SATURN_WRITE_SINGLE( 0x00A0, 0x04 );
    SATURN_WRITE_SINGLE( 0x00A1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00A2, 0x04 );
    SATURN_WRITE_SINGLE( 0x00A3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00A4, 0x04 );
    SATURN_WRITE_SINGLE( 0x00A5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00A6, 0x04 );
    SATURN_WRITE_SINGLE( 0x00A7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00A8, 0x04 );
    SATURN_WRITE_SINGLE( 0x00A9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00AA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00AB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00AC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00AD, 0x00 );
    SATURN_WRITE_SINGLE( 0x00AE, 0x04 );
    SATURN_WRITE_SINGLE( 0x00AF, 0x00 );
    SATURN_WRITE_SINGLE( 0x00B0, 0x04 );
    SATURN_WRITE_SINGLE( 0x00B1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00B2, 0x04 );
    SATURN_WRITE_SINGLE( 0x00B3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00B4, 0x04 );
    SATURN_WRITE_SINGLE( 0x00B5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00B6, 0x04 );
    SATURN_WRITE_SINGLE( 0x00B7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00B8, 0x04 );
    SATURN_WRITE_SINGLE( 0x00B9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00BA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00BB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00BC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00BD, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C0, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C1, 0x10 );
    SATURN_WRITE_SINGLE( 0x00C2, 0xCC );
    SATURN_WRITE_SINGLE( 0x00C3, 0xCC );
    SATURN_WRITE_SINGLE( 0x00C4, 0x0C );
    SATURN_WRITE_SINGLE( 0x00C5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C6, 0x0A );
    SATURN_WRITE_SINGLE( 0x00C7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C8, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00CA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00CB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00CC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00CD, 0x00 );
    SATURN_WRITE_SINGLE( 0x00CE, 0x04 );
    SATURN_WRITE_SINGLE( 0x00CF, 0x00 );
    SATURN_WRITE_SINGLE( 0x00D0, 0x04 );
    SATURN_WRITE_SINGLE( 0x00D1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00D2, 0x04 );
    SATURN_WRITE_SINGLE( 0x00D3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00D4, 0x04 );
    SATURN_WRITE_SINGLE( 0x00D5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00D6, 0x04 );
    SATURN_WRITE_SINGLE( 0x00D7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00D8, 0x04 );
    SATURN_WRITE_SINGLE( 0x00D9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00DA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00DB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00DC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00DD, 0x00 );
    SATURN_WRITE_SINGLE( 0x00DE, 0x04 );
    SATURN_WRITE_SINGLE( 0x00DF, 0x00 );
    SATURN_WRITE_SINGLE( 0x00E0, 0x04 );
    SATURN_WRITE_SINGLE( 0x00E1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00E2, 0x04 );
    SATURN_WRITE_SINGLE( 0x00E3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00E4, 0x04 );
    SATURN_WRITE_SINGLE( 0x00E5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00E6, 0x04 );
    SATURN_WRITE_SINGLE( 0x00E7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00E8, 0x04 );
    SATURN_WRITE_SINGLE( 0x00E9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00EA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00EB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00EC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00ED, 0x00 );
    SATURN_WRITE_SINGLE( 0x00EE, 0x04 );
    SATURN_WRITE_SINGLE( 0x00EF, 0x00 );
    SATURN_WRITE_SINGLE( 0x00F0, 0x04 );
    SATURN_WRITE_SINGLE( 0x00F1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00F2, 0x04 );
    SATURN_WRITE_SINGLE( 0x00F3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00F4, 0x04 );
    SATURN_WRITE_SINGLE( 0x00F5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00F6, 0x04 );
    SATURN_WRITE_SINGLE( 0x00F7, 0x00 );
    SATURN_WRITE_SINGLE( 0x00F8, 0x04 );
    SATURN_WRITE_SINGLE( 0x00F9, 0x00 );
    SATURN_WRITE_SINGLE( 0x00FA, 0x04 );
    SATURN_WRITE_SINGLE( 0x00FB, 0x00 );
    SATURN_WRITE_SINGLE( 0x00FC, 0x04 );
    SATURN_WRITE_SINGLE( 0x00FD, 0x00 );
    SATURN_WRITE_SINGLE( 0x0100, 0x00 );
    SATURN_WRITE_SINGLE( 0x0101, 0x00 );
    SATURN_WRITE_SINGLE( 0x0102, 0xCC );
    SATURN_WRITE_SINGLE( 0x0103, 0xCC );
    SATURN_WRITE_SINGLE( 0x0104, 0x0C );
    SATURN_WRITE_SINGLE( 0x0105, 0x00 );
    SATURN_WRITE_SINGLE( 0x0106, 0x06 );
    SATURN_WRITE_SINGLE( 0x0107, 0x27 );
    SATURN_WRITE_SINGLE( 0x0108, 0x00 );
    SATURN_WRITE_SINGLE( 0x0109, 0x00 );
    SATURN_WRITE_SINGLE( 0x010A, 0x04 );
    SATURN_WRITE_SINGLE( 0x010B, 0x00 );
    SATURN_WRITE_SINGLE( 0x010C, 0x04 );
    SATURN_WRITE_SINGLE( 0x010D, 0x00 );
    SATURN_WRITE_SINGLE( 0x010E, 0x04 );
    SATURN_WRITE_SINGLE( 0x010F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0110, 0x04 );
    SATURN_WRITE_SINGLE( 0x0111, 0x00 );
    SATURN_WRITE_SINGLE( 0x0112, 0x04 );
    SATURN_WRITE_SINGLE( 0x0113, 0x00 );
    SATURN_WRITE_SINGLE( 0x0114, 0x04 );
    SATURN_WRITE_SINGLE( 0x0115, 0x00 );
    SATURN_WRITE_SINGLE( 0x0116, 0x04 );
    SATURN_WRITE_SINGLE( 0x0117, 0x00 );
    SATURN_WRITE_SINGLE( 0x0118, 0x04 );
    SATURN_WRITE_SINGLE( 0x0119, 0x00 );
    SATURN_WRITE_SINGLE( 0x011A, 0x04 );
    SATURN_WRITE_SINGLE( 0x011B, 0x00 );
    SATURN_WRITE_SINGLE( 0x011C, 0x04 );
    SATURN_WRITE_SINGLE( 0x011D, 0x00 );
    SATURN_WRITE_SINGLE( 0x011E, 0x04 );
    SATURN_WRITE_SINGLE( 0x011F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0120, 0x04 );
    SATURN_WRITE_SINGLE( 0x0121, 0x00 );
    SATURN_WRITE_SINGLE( 0x0122, 0x04 );
    SATURN_WRITE_SINGLE( 0x0123, 0x00 );
    SATURN_WRITE_SINGLE( 0x0124, 0x04 );
    SATURN_WRITE_SINGLE( 0x0125, 0x00 );
    SATURN_WRITE_SINGLE( 0x0126, 0x04 );
    SATURN_WRITE_SINGLE( 0x0127, 0x00 );
    SATURN_WRITE_SINGLE( 0x0128, 0x04 );
    SATURN_WRITE_SINGLE( 0x0129, 0x00 );
    SATURN_WRITE_SINGLE( 0x012A, 0x04 );
    SATURN_WRITE_SINGLE( 0x012B, 0x00 );
    SATURN_WRITE_SINGLE( 0x012C, 0x04 );
    SATURN_WRITE_SINGLE( 0x012D, 0x00 );
    SATURN_WRITE_SINGLE( 0x012E, 0x04 );
    SATURN_WRITE_SINGLE( 0x012F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0130, 0x04 );
    SATURN_WRITE_SINGLE( 0x0131, 0x00 );
    SATURN_WRITE_SINGLE( 0x0132, 0x04 );
    SATURN_WRITE_SINGLE( 0x0133, 0x00 );
    SATURN_WRITE_SINGLE( 0x0134, 0x04 );
    SATURN_WRITE_SINGLE( 0x0135, 0x00 );
    SATURN_WRITE_SINGLE( 0x0136, 0x04 );
    SATURN_WRITE_SINGLE( 0x0137, 0x00 );
    SATURN_WRITE_SINGLE( 0x0138, 0x04 );
    SATURN_WRITE_SINGLE( 0x0139, 0x00 );
    SATURN_WRITE_SINGLE( 0x013A, 0x04 );
    SATURN_WRITE_SINGLE( 0x013B, 0x00 );
    SATURN_WRITE_SINGLE( 0x013C, 0x04 );
    SATURN_WRITE_SINGLE( 0x013D, 0x00 );
    SATURN_WRITE_SINGLE( 0x0140, 0x00 );
    SATURN_WRITE_SINGLE( 0x0141, 0x10 );
    SATURN_WRITE_SINGLE( 0x0142, 0xCC );
    SATURN_WRITE_SINGLE( 0x0143, 0xCC );
    SATURN_WRITE_SINGLE( 0x0144, 0x0C );
    SATURN_WRITE_SINGLE( 0x0145, 0x00 );
    SATURN_WRITE_SINGLE( 0x0146, 0x0A );
    SATURN_WRITE_SINGLE( 0x0147, 0x00 );
    SATURN_WRITE_SINGLE( 0x0148, 0x00 );
    SATURN_WRITE_SINGLE( 0x0149, 0x00 );
    SATURN_WRITE_SINGLE( 0x014A, 0x04 );
    SATURN_WRITE_SINGLE( 0x014B, 0x00 );
    SATURN_WRITE_SINGLE( 0x014C, 0x04 );
    SATURN_WRITE_SINGLE( 0x014D, 0x00 );
    SATURN_WRITE_SINGLE( 0x014E, 0x04 );
    SATURN_WRITE_SINGLE( 0x014F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0150, 0x04 );
    SATURN_WRITE_SINGLE( 0x0151, 0x00 );
    SATURN_WRITE_SINGLE( 0x0152, 0x04 );
    SATURN_WRITE_SINGLE( 0x0153, 0x00 );
    SATURN_WRITE_SINGLE( 0x0154, 0x04 );
    SATURN_WRITE_SINGLE( 0x0155, 0x00 );
    SATURN_WRITE_SINGLE( 0x0156, 0x04 );
    SATURN_WRITE_SINGLE( 0x0157, 0x00 );
    SATURN_WRITE_SINGLE( 0x0158, 0x04 );
    SATURN_WRITE_SINGLE( 0x0159, 0x00 );
    SATURN_WRITE_SINGLE( 0x015A, 0x04 );
    SATURN_WRITE_SINGLE( 0x015B, 0x00 );
    SATURN_WRITE_SINGLE( 0x015C, 0x04 );
    SATURN_WRITE_SINGLE( 0x015D, 0x00 );
    SATURN_WRITE_SINGLE( 0x015E, 0x04 );
    SATURN_WRITE_SINGLE( 0x015F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0160, 0x04 );
    SATURN_WRITE_SINGLE( 0x0161, 0x00 );
    SATURN_WRITE_SINGLE( 0x0162, 0x04 );
    SATURN_WRITE_SINGLE( 0x0163, 0x00 );
    SATURN_WRITE_SINGLE( 0x0164, 0x04 );
    SATURN_WRITE_SINGLE( 0x0165, 0x00 );
    SATURN_WRITE_SINGLE( 0x0166, 0x04 );
    SATURN_WRITE_SINGLE( 0x0167, 0x00 );
    SATURN_WRITE_SINGLE( 0x0168, 0x04 );
    SATURN_WRITE_SINGLE( 0x0169, 0x00 );
    SATURN_WRITE_SINGLE( 0x016A, 0x04 );
    SATURN_WRITE_SINGLE( 0x016B, 0x00 );
    SATURN_WRITE_SINGLE( 0x016C, 0x04 );
    SATURN_WRITE_SINGLE( 0x016D, 0x00 );
    SATURN_WRITE_SINGLE( 0x016E, 0x04 );
    SATURN_WRITE_SINGLE( 0x016F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0170, 0x04 );
    SATURN_WRITE_SINGLE( 0x0171, 0x00 );
    SATURN_WRITE_SINGLE( 0x0172, 0x04 );
    SATURN_WRITE_SINGLE( 0x0173, 0x00 );
    SATURN_WRITE_SINGLE( 0x0174, 0x04 );
    SATURN_WRITE_SINGLE( 0x0175, 0x00 );
    SATURN_WRITE_SINGLE( 0x0176, 0x04 );
    SATURN_WRITE_SINGLE( 0x0177, 0x00 );
    SATURN_WRITE_SINGLE( 0x0178, 0x04 );
    SATURN_WRITE_SINGLE( 0x0179, 0x00 );
    SATURN_WRITE_SINGLE( 0x017A, 0x04 );
    SATURN_WRITE_SINGLE( 0x017B, 0x00 );
    SATURN_WRITE_SINGLE( 0x017C, 0x04 );
    SATURN_WRITE_SINGLE( 0x017D, 0x00 );
    SATURN_WRITE_SINGLE( 0x0180, 0x00 );
    SATURN_WRITE_SINGLE( 0x0181, 0x00 );
    SATURN_WRITE_SINGLE( 0x0182, 0xCC );
    SATURN_WRITE_SINGLE( 0x0183, 0xCC );
    SATURN_WRITE_SINGLE( 0x0184, 0x0C );
    SATURN_WRITE_SINGLE( 0x0185, 0x00 );
    SATURN_WRITE_SINGLE( 0x0186, 0x00 );
    SATURN_WRITE_SINGLE( 0x0187, 0x00 );
    SATURN_WRITE_SINGLE( 0x0188, 0x00 );
    SATURN_WRITE_SINGLE( 0x0189, 0x00 );
    SATURN_WRITE_SINGLE( 0x018A, 0x04 );
    SATURN_WRITE_SINGLE( 0x018B, 0x00 );
    SATURN_WRITE_SINGLE( 0x018C, 0x04 );
    SATURN_WRITE_SINGLE( 0x018D, 0x00 );
    SATURN_WRITE_SINGLE( 0x018E, 0x04 );
    SATURN_WRITE_SINGLE( 0x018F, 0x00 );
    SATURN_WRITE_SINGLE( 0x0190, 0x04 );
    SATURN_WRITE_SINGLE( 0x0191, 0x00 );
    SATURN_WRITE_SINGLE( 0x0192, 0x04 );
    SATURN_WRITE_SINGLE( 0x0193, 0x00 );
    SATURN_WRITE_SINGLE( 0x0194, 0x04 );
    SATURN_WRITE_SINGLE( 0x0195, 0x00 );
    SATURN_WRITE_SINGLE( 0x0196, 0x04 );
    SATURN_WRITE_SINGLE( 0x0197, 0x00 );
    SATURN_WRITE_SINGLE( 0x0198, 0x04 );
    SATURN_WRITE_SINGLE( 0x0199, 0x00 );
    SATURN_WRITE_SINGLE( 0x019A, 0x04 );
    SATURN_WRITE_SINGLE( 0x019B, 0x00 );
    SATURN_WRITE_SINGLE( 0x019C, 0x04 );
    SATURN_WRITE_SINGLE( 0x019D, 0x00 );
    SATURN_WRITE_SINGLE( 0x019E, 0x04 );
    SATURN_WRITE_SINGLE( 0x019F, 0x00 );
    SATURN_WRITE_SINGLE( 0x01A0, 0x04 );
    SATURN_WRITE_SINGLE( 0x01A1, 0x00 );
    SATURN_WRITE_SINGLE( 0x01A2, 0x04 );
    SATURN_WRITE_SINGLE( 0x01A3, 0x00 );
    SATURN_WRITE_SINGLE( 0x01A4, 0x04 );
    SATURN_WRITE_SINGLE( 0x01A5, 0x00 );
    SATURN_WRITE_SINGLE( 0x01A6, 0x04 );
    SATURN_WRITE_SINGLE( 0x01A7, 0x00 );
    SATURN_WRITE_SINGLE( 0x01A8, 0x04 );
    SATURN_WRITE_SINGLE( 0x01A9, 0x00 );
    SATURN_WRITE_SINGLE( 0x01AA, 0x04 );
    SATURN_WRITE_SINGLE( 0x01AB, 0x00 );
    SATURN_WRITE_SINGLE( 0x01AC, 0x04 );
    SATURN_WRITE_SINGLE( 0x01AD, 0x00 );
    SATURN_WRITE_SINGLE( 0x01AE, 0x04 );
    SATURN_WRITE_SINGLE( 0x01AF, 0x00 );
    SATURN_WRITE_SINGLE( 0x01B0, 0x04 );
    SATURN_WRITE_SINGLE( 0x01B1, 0x00 );
    SATURN_WRITE_SINGLE( 0x01B2, 0x04 );
    SATURN_WRITE_SINGLE( 0x01B3, 0x00 );
    SATURN_WRITE_SINGLE( 0x01B4, 0x04 );
    SATURN_WRITE_SINGLE( 0x01B5, 0x00 );
    SATURN_WRITE_SINGLE( 0x01B6, 0x04 );
    SATURN_WRITE_SINGLE( 0x01B7, 0x00 );
    SATURN_WRITE_SINGLE( 0x01B8, 0x04 );
    SATURN_WRITE_SINGLE( 0x01B9, 0x00 );
    SATURN_WRITE_SINGLE( 0x01BA, 0x04 );
    SATURN_WRITE_SINGLE( 0x01BB, 0x00 );
    SATURN_WRITE_SINGLE( 0x01BC, 0x04 );
    SATURN_WRITE_SINGLE( 0x01BD, 0x00 );

    default_chunk_write( 0x01C0 );
    default_chunk_write( 0x0200 );
    default_chunk_write( 0x0240 );
    default_chunk_write( 0x0280 );
    default_chunk_write( 0x02C0 );
    default_chunk_write( 0x0300 );
    default_chunk_write( 0x0340 );
    default_chunk_write( 0x0380 );
    default_chunk_write( 0x03C0 );
    default_chunk_write( 0x0400 );
    default_chunk_write( 0x0440 );
    default_chunk_write( 0x0480 );
    default_chunk_write( 0x04C0 );
    default_chunk_write( 0x0500 );
    default_chunk_write( 0x0500 );
    default_chunk_write( 0x0540 );
    default_chunk_write( 0x0580 );
    default_chunk_write( 0x05C0 );
    default_chunk_write( 0x0600 );

    HAL_GPIO_WritePin( LF_STIM_RUN_Port, LF_STIM_RUN_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LF_STIM_CONT_Port, LF_STIM_CONT_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin, GPIO_PIN_RESET );
}

void Saturn2_spi::default_chunk_write( unsigned int start_address )
{
    const uint8_t config[] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

    for ( unsigned int i = 0; i < sizeof( config ); i++ )
    {
        SATURN_WRITE_SINGLE( start_address + i, config[ i ] );
    }

    for ( unsigned int i = start_address + 0x0A; i <= start_address + 0x3C; i += 2 )
    {
        SATURN_WRITE_SINGLE( i, 0x04 );
        SATURN_WRITE_SINGLE( i + 1, 0x00 );
    }
}

void Saturn2_spi::ElectrodeEnable( CommandData_T* cmd )
{
    for ( int i = 0; i < 27; i++ )
    {
        if ( mapping2[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a_2 + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b_2 + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a_2 + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b_2 + 8;

            SATURN_WRITE_MASK( phase1, 16, 16 );
            SATURN_WRITE_MASK( phase2, 16, 16 );
            SATURN_WRITE_MASK( phase3, 16, 16 );
            SATURN_WRITE_MASK( phase4, 16, 16 );
        }
    }
}

void Saturn2_spi::DisableElectrode( CommandData_T* cmd )
{
    for ( int i = 0; i < 27; i++ )
    {
        if ( mapping2[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a_2 + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b_2 + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a_2 + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b_2 + 8;

            SATURN_WRITE_MASK( phase1, 0, 16 );
            SATURN_WRITE_MASK( phase2, 0, 16 );
            SATURN_WRITE_MASK( phase3, 0, 16 );
            SATURN_WRITE_MASK( phase4, 0, 16 );
        }
    }
}

void Saturn2_spi::ElectrodetoChannel( CommandData_T* cmd )
{
    std::unordered_map< int, int > hashMap;
    hashMap[ 1 ] = 12;
    hashMap[ 2 ] = 9;
    hashMap[ 3 ] = 5;
    hashMap[ 4 ] = 3;

    uint16_t phase1
        = 2 * ( hashMap[ cmd->cmdDataBuf[ 0 ] ] ) + seq0a_2 + 8;    // Phase 1 Address XA
    uint16_t phase2
        = 2 * ( hashMap[ cmd->cmdDataBuf[ 0 ] ] ) + seq0b_2 + 8;    // Phase 2 Address XA
    uint16_t phase3 = 2 * ( hashMap[ cmd->cmdDataBuf[ 0 ] ] ) + seq1a_2 + 8;
    uint16_t phase4 = 2 * ( hashMap[ cmd->cmdDataBuf[ 0 ] ] ) + seq1b_2 + 8;

    if ( cmd->cmdDataBuf[ 1 ] == 0x01 )       // Source first, sink second
    {
        SATURN_WRITE_MASK( phase1, 0, 4 );    // 0 to Bit 2 for source
        SATURN_WRITE_MASK( phase2, 4, 4 );    // 1 to Bit 2 for sink
        SATURN_WRITE_MASK( phase3, 0, 4 );    // 0 to Bit 2 for source
        SATURN_WRITE_MASK( phase4, 4, 4 );
    }
    else if ( cmd->cmdDataBuf[ 1 ] == 0x02 )    // Sink first, source second
    {
        SATURN_WRITE_MASK( phase1, 4, 4 );      // 1 to Bit 2 for sink
        SATURN_WRITE_MASK( phase2, 0, 4 );      // 0 to Bit 2 for source
        SATURN_WRITE_MASK( phase3, 4, 4 );      // 1 to Bit 2 for sink
        SATURN_WRITE_MASK( phase4, 0, 4 );
    }

    mapping2[ hashMap[ cmd->cmdDataBuf[ 0 ] ] ] = cmd->cmdDataBuf[ 2 ];
}

void Saturn2_spi::Starttick( void )
{
    starttick2 = HAL_GetTick();
    ogtick2    = starttick2;
}

void Saturn2_spi::StartContinuousStim( void )
{
    //	HAL_GPIO_TogglePin(GPIOB, SD_CS_Pin);		//used for debugging
    // timing before board calibration
    SATURN_WRITE_SINGLE( 0x0007, 0xFF );
    SATURN_WRITE_SINGLE( 0x0008, 0xFF );
    SATURN_WRITE_SINGLE( 0x0009, 0xFF );
    SATURN_WRITE_SINGLE( 0x000A, 0xFF );
    SATURN_WRITE_SINGLE( 0x000B, 0x03 );

    HAL_GPIO_WritePin( LF_STIM_RUN_Port, LF_STIM_RUN_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LF_STIM_STOP_Port, LF_STIM_STOP_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LF_STIM_CONT_Port, LF_STIM_CONT_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( LF_STIM_RUN_Port, LF_STIM_RUN_Pin, GPIO_PIN_SET );
}

void Saturn2_spi::Amplitude( CommandData_T* cmd )
{
    for ( int i = 0; i < 27; i++ )
    {
        if ( mapping2[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a_2 + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b_2 + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a_2 + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b_2 + 8;

            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMP          = AMP / 15;

                uint8_t AMPL = AMP & 0xFF;
                SATURN_WRITE_SINGLE( phase3 - 1, AMPL );
                SATURN_WRITE_SINGLE( phase4 - 1, AMPL );

                uint8_t AMPU = ( AMP >> 8 ) & 0x03;
                SATURN_WRITE_MASK( phase3, AMPU, 3 );
                SATURN_WRITE_MASK( phase4, AMPU, 3 );
            }

            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMP2ph1[ i ] = AMP;
                AMP2ph2[ i ] = AMP;
                AMP          = AMP / 15;

                uint8_t AMPL = AMP & 0xFF;
                SATURN_WRITE_SINGLE( phase1 - 1, AMPL );    // Channel 1, phase 1
                SATURN_WRITE_SINGLE( phase2 - 1, AMPL );    // Channel 1, phase 2

                uint8_t AMPU = ( AMP >> 8 ) & 0x03;
                SATURN_WRITE_MASK( phase1, AMPU, 3 );    // Channel 1, phase 1
                SATURN_WRITE_MASK( phase2, AMPU, 3 );    // Channel 1, phase 2
            }
            HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );

            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMP          = AMP / 15;

                uint8_t AMPL = AMP & 0xFF;
                SATURN_WRITE_SINGLE( phase3 - 1, AMPL );
                SATURN_WRITE_SINGLE( phase4 - 1, AMPL );

                uint8_t AMPU = ( AMP >> 8 ) & 0x03;
                SATURN_WRITE_MASK( phase3, AMPU, 3 );
                SATURN_WRITE_MASK( phase4, AMPU, 3 );
            }

            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMP2ph1[ i ] = AMP;
                AMP2ph2[ i ] = AMP;
                AMP          = AMP / 15;

                uint8_t AMPL = AMP & 0xFF;
                SATURN_WRITE_SINGLE( phase1 - 1, AMPL );    // Channel 1, phase 1
                SATURN_WRITE_SINGLE( phase2 - 1, AMPL );    // Channel 1, phase 2

                uint8_t AMPU = ( AMP >> 8 ) & 0x03;
                SATURN_WRITE_MASK( phase1, AMPU, 3 );    // Channel 1, phase 1
                SATURN_WRITE_MASK( phase2, AMPU, 3 );    // Channel 1, phase 2
            }
        }
    }
    Saturn2_spi::chargecheck( cmd );
}

CmdHandlerReturnCode_T Saturn2_spi::PulseWidth( CommandData_T* cmd )
{
    // cmd->cmdDataBuf[ 0 ] is electrode number
    if ( ( ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
           | ( cmd->cmdDataBuf[ 3 ] << 16 ) )
         > 0x989680 )
    {
        return CmdHndlRetPWOutRange;
    }
    else
    {
        if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
        {
            PW2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
                | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW2 = pow( 2.0, 24 ) / PW2;
            SATURN_WRITE_SINGLE( seq1a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1a_2 + 2, ( PW2 & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq1b_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1b_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1b_2 + 2, ( PW2 & 0x0000FF ) );
        }
        else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
        {
            PW2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
                | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW2 = pow( 2.0, 24 ) / PW2;
            SATURN_WRITE_SINGLE( seq0a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0a_2 + 2, ( PW2 & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq0b_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0b_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0b_2 + 2, ( PW2 & 0x0000FF ) );
        }
        HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
        if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
        {
            PW2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
                | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW2 = pow( 2.0, 24 ) / PW2;
            SATURN_WRITE_SINGLE( seq1a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1a_2 + 2, ( PW2 & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq1b_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1b_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1b_2 + 2, ( PW2 & 0x0000FF ) );
        }
        else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
        {
            PW2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
                | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW2 = pow( 2.0, 24 ) / PW2;
            SATURN_WRITE_SINGLE( seq0a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0a_2 + 2, ( PW2 & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq0b_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0b_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0b_2 + 2, ( PW2 & 0x0000FF ) );
        }
        PW2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
            | ( cmd->cmdDataBuf[ 3 ] << 16 );
        PW2phase2 = PW2;
        Saturn2_spi::chargecheck( cmd );
        return Saturn2_spi::update_delay( cmd );
    }
}

void Saturn2_spi::StartDelay( CommandData_T* cmd )
{
    // SATURN_WRITE_SINGLE(0x000C,	0x00);	//SEQ0A
    // SATURN_WRITE_SINGLE(0x000E,	0x04);	//SEQ1A
    // cmd->cmdDataBuf[ 0 ] is electrode number

    startdelay2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
    /*
    if (HAL_GPIO_ReadPin(GPIOB, LF_STIM_SEL_Pin)==1){
            SATURN_WRITE_SINGLE(0x0046, (startdelay2 & 0xFF));
    //DLYL SATURN_WRITE_SINGLE(0x0047,	(startdelay2 & 0xFF00)>>8);
    //DLYU
    }
    else if (HAL_GPIO_ReadPin(GPIOB, LF_STIM_SEL_Pin)==0){
            SATURN_WRITE_SINGLE(0x146,	(startdelay2 & 0xFF));
            SATURN_WRITE_SINGLE(0x147,	(startdelay2 & 0xFF00)>>8);
    }
    HAL_GPIO_TogglePin(GPIOB, LF_STIM_SEL_Pin);

    if (HAL_GPIO_ReadPin(GPIOB, LF_STIM_SEL_Pin)==1){
            SATURN_WRITE_SINGLE(0x0046, (startdelay2 & 0xFF));
    //DLYL SATURN_WRITE_SINGLE(0x0047,	(startdelay2 & 0xFF00)>>8);
    //DLYU
    }
    else if (HAL_GPIO_ReadPin(GPIOB, LF_STIM_SEL_Pin)==0){
            SATURN_WRITE_SINGLE(0x146,	(startdelay2 & 0xFF));
            SATURN_WRITE_SINGLE(0x147,	(startdelay2 & 0xFF00)>>8);

    }*/
}

CmdHandlerReturnCode_T Saturn2_spi::Frequency( CommandData_T* cmd )
{
    if ( ( ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) ) == 0 )
    {
        f2 = 2;
        return CmdHndlRetFreqZero;
    }
    else
    {
        f2 = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
        return Saturn2_spi::update_delay( cmd );
    }
}

CmdHandlerReturnCode_T Saturn2_spi::InterphaseDelay( CommandData_T* cmd )
{
    uint32_t IDrequest
        = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) | ( cmd->cmdDataBuf[ 3 ] << 16 );

    if ( IDrequest > 0x989680 )
    {
        return CmdHndlIDOutRange;
    }
    else
    {
        if ( IDrequest > 1 )
        {                                  // If they send a delay that is 2us or greater
            delay1reg2 = IDrequest - 2;    // Account for the built in delay
                                           // IDIRL = IDrequest;
        }
        else if ( IDrequest <= 1 )
        {                      // If the delay is 0us or 1us
            delay1reg2 = 0;    // Set the delay to 0us
                               // IDIRL = 2;
        }

        if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
        {
            SATURN_WRITE_SINGLE( seq0a_2 + 6, ( delay1reg2 & 0xFF ) );           // DLYL
            SATURN_WRITE_SINGLE( seq0a_2 + 7, ( delay1reg2 & 0xFF00 ) >> 8 );    // DLYU
        }

        else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
        {
            SATURN_WRITE_SINGLE( seq1a_2 + 6, ( delay1reg2 & 0xFF ) );
            SATURN_WRITE_SINGLE( seq1a_2 + 7, ( delay1reg2 & 0xFF00 ) >> 8 );
        }
        HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
        if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
        {
            SATURN_WRITE_SINGLE( seq0a_2 + 6, ( delay1reg2 & 0xFF ) );           // DLYL
            SATURN_WRITE_SINGLE( seq0a_2 + 7, ( delay1reg2 & 0xFF00 ) >> 8 );    // DLYU
        }
        else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
        {
            SATURN_WRITE_SINGLE( seq1a_2 + 6, ( delay1reg2 & 0xFF ) );
            SATURN_WRITE_SINGLE( seq1a_2 + 7, ( delay1reg2 & 0xFF00 ) >> 8 );
        }
        return Saturn2_spi::update_delay( cmd );
    }
}

CmdHandlerReturnCode_T Saturn2_spi::update_delay( CommandData_T* cmd )
{
    uint32_t delay2reg;
    if ( f2 != 0 )
    {
        delay2reg = ( 1000000 / f2 ) - PW2 - PW2phase2 - delay1reg2;
    }
    else
    {
        return CmdHndlRetFreqZero;
    }
    if ( PW2 + PW2phase2 + delay1reg2 > ( 1000000 / f2 ) )
    {
        return CmdHndlRetFreqTooSmall;
    }
    else
    {
        if ( delay2reg > 1 )
        {                      // If they send a delay that is 2us or greater
            delay2reg -= 2;    // Account for the built in delay
                               // IDIRL = IDrequest;
        }
        else if ( delay2reg <= 1 )
        {                     // If the delay is 0us or 1us
            delay2reg = 0;    // Set the delay to 0us
                              // IDIRL = 2;
        }

        if ( delay2reg < 0xFFFF )
        {
            SATURN_WRITE_SINGLE( 0x000D, ( seq0b_2 / 64 ) - 1 );    // SEQ0B
            SATURN_WRITE_SINGLE( 0x000F, ( seq1b_2 / 64 ) - 1 );    // SEQ1B
            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b_2 + 6, ( delay2reg & 0xFF ) );           // DLYL
                SATURN_WRITE_SINGLE( seq0b_2 + 7, ( delay2reg & 0xFF00 ) >> 8 );    // DLYU
            }
            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b_2 + 6, ( delay2reg & 0xFF ) );
                SATURN_WRITE_SINGLE( seq1b_2 + 7, ( delay2reg & 0xFF00 ) >> 8 );
            }

            HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b_2 + 6, ( delay2reg & 0xFF ) );           // DLYL
                SATURN_WRITE_SINGLE( seq0b_2 + 7, ( delay2reg & 0xFF00 ) >> 8 );    // DLYU
            }
            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b_2 + 6, ( delay2reg & 0xFF ) );
                SATURN_WRITE_SINGLE( seq1b_2 + 7, ( delay2reg & 0xFF00 ) >> 8 );
            }
        }
        else
        {
            Saturn2_spi::AddDelay( delay2reg );
        }

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

void Saturn2_spi::AddDelay( unsigned int delay2 )
{
    SATURN_WRITE_SINGLE( 0x000D, ( seq0delay_2 / 64 ) - 1 );    // SEQ0B
    SATURN_WRITE_SINGLE( 0x000F, ( seq1delay_2 / 64 ) - 1 );    // SEQ1B
    uint32_t remainder = delay2 - 0xFFFF - 2;
    uint32_t extrapw   = 2;

    if ( remainder > 0xFFFF )
    {
        extrapw   = remainder - 0xFFFF + 2;
        remainder = 0xFFFF;
    }
    extrapw = pow( 2.0, 24 ) / extrapw;

    if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
    {
        SATURN_WRITE_SINGLE( seq0b_2 + 6, 0xFF );                        // DLYL Phase 2
        SATURN_WRITE_SINGLE( seq0b_2 + 7, 0xFF );                        // DLYU Phase 2
        SATURN_WRITE_SINGLE( seq0delay_2 + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay_2 + 7,
                             ( remainder & 0xFF00 ) >> 8 );              // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay_2 + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq0delay_2 + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq0delay_2 + 2, ( extrapw & 0x0000FF ) );
    }
    else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
    {
        SATURN_WRITE_SINGLE( seq1b_2 + 6, 0xFF );
        SATURN_WRITE_SINGLE( seq1b_2 + 7, 0xFF );
        SATURN_WRITE_SINGLE( seq1delay_2 + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay_2 + 7,
                             ( remainder & 0xFF00 ) >> 8 );              // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay_2 + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq1delay_2 + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq1delay_2 + 2, ( extrapw & 0x0000FF ) );
    }
    HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );

    if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
    {
        SATURN_WRITE_SINGLE( seq0b_2 + 6, 0xFF );                        // DLYL Phase 2
        SATURN_WRITE_SINGLE( seq0b_2 + 7, 0xFF );                        // DLYU Phase 2
        SATURN_WRITE_SINGLE( seq0delay_2 + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay_2 + 7,
                             ( remainder & 0xFF00 ) >> 8 );              // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay_2 + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq0delay_2 + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq0delay_2 + 2, ( extrapw & 0x0000FF ) );
    }
    else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
    {
        SATURN_WRITE_SINGLE( seq1b_2 + 6, 0xFF );
        SATURN_WRITE_SINGLE( seq1b_2 + 7, 0xFF );
        SATURN_WRITE_SINGLE( seq1delay_2 + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay_2 + 7,
                             ( remainder & 0xFF00 ) >> 8 );              // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay_2 + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq1delay_2 + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq1delay_2 + 2, ( extrapw & 0x0000FF ) );
    }
}

void Saturn2_spi::StopContinuousStim( void )
{
    //	HAL_GPIO_TogglePin(GPIOB, SD_CS_Pin);		//used for debugging
    // timing before board calibration
    HAL_GPIO_WritePin( LF_STIM_CONT_Port, LF_STIM_CONT_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( LF_STIM_RUN_Port, LF_STIM_RUN_Pin, GPIO_PIN_RESET );
    // if ( measuringimp )
    // {
    //     measuringimp = false;
    //     Impedance::StopMeasurement();
    // }
}

void Saturn2_spi::CurrentTick( uint32_t tick )
{
    if ( ( tick == starttick2 + 1 )
         || ( ( tick == starttick2 + offduration2 + duration2 + 1 ) && stoppoint2 != 0 ) )
    {
        if ( ( stoppoint2 != 0 ) && ( tick < ( ogtick2 + stoppoint2 ) ) )
        {
            starttick2 = tick - 1;
            Saturn2_spi::StartContinuousStim();
        }
        else if ( stoppoint2 == 0 && startdelay2 == 0 )
        {
            Saturn2_spi::StartContinuousStim();
        }
        else if ( ( stoppoint2 != 0 ) && ( tick >= ogtick2 + stoppoint2 ) )
        {
        }
    }
    else if ( ( tick == starttick2 + duration2 + 1 ) && ( duration2 != 0 ) )
    {
        Saturn2_spi::StopContinuousStim();
    }
    else if ( tick == starttick2 + startdelay2 + 1 )
    {
        Saturn2_spi::StartContinuousStim();
    }
    else
    {
    }
}

void Saturn2_spi::StimDuration( CommandData_T* cmd )
{
    duration2
        = ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) | ( cmd->cmdDataBuf[ 2 ] << 16 );
}

void Saturn2_spi::OffDuration( CommandData_T* cmd )
{
    offduration2
        = ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) | ( cmd->cmdDataBuf[ 2 ] << 16 );
}

void Saturn2_spi::NCycles( CommandData_T* cmd )
{
    stoppoint2 = ( duration2 + offduration2 )
               * ( ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) );
}

void Saturn2_spi::Calibration( void )
{
    SATURN_WRITE_SINGLE( 0x022, calvals[ 0x222 ] );
    SATURN_WRITE_SINGLE( 0x034, calvals[ 0x234 ] );
    SATURN_WRITE_SINGLE( 0x037, calvals[ 0x237 ] );
    SATURN_WRITE_SINGLE( 0x038, calvals[ 0x238 ] );
}

void Saturn2_spi::RAM1( CommandData_T* cmd )
{
    SATURN_WRITE_MASK( 0x00A, 0x0, 0xC0 );
    SATURN_WRITE_MASK( 0x025, 0x30, 0x30 );
    SATURN_WRITE_SINGLE( 2048, 0 );

    for ( int i = 0; i < 256; i++ )
    {
        SATURN_WRITE_SINGLE( ( 2048 + i ), cmd->cmdDataBuf[ i ] );
    }
}

void Saturn2_spi::RAM2( CommandData_T* cmd )
{
    SATURN_WRITE_MASK( 0x00A, 0x0, 0xC0 );
    SATURN_WRITE_MASK( 0x025, 0x30, 0x30 );
    SATURN_WRITE_SINGLE( 2304, 0 );

    for ( int i = 0; i < 256; i++ )
    {
        SATURN_WRITE_SINGLE( ( 2304 + i ), cmd->cmdDataBuf[ i ] );
    }
}

void Saturn2_spi::SelectShape( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 2 ] != 0 && cmd->cmdDataBuf[ 1 ] == 1 )
    {
        SATURN_WRITE_SINGLE( seq0a_2, 4 );    // Ram1
        SATURN_WRITE_SINGLE( seq1a_2, 4 );    // Ram1
    }
    else if ( cmd->cmdDataBuf[ 2 ] != 0 && cmd->cmdDataBuf[ 1 ] == 2 )
    {
        SATURN_WRITE_SINGLE( seq0b_2, 5 );    // Ram2
        SATURN_WRITE_SINGLE( seq1b_2, 5 );    // Ram2
    }
    else if ( cmd->cmdDataBuf[ 2 ] == 0 && cmd->cmdDataBuf[ 1 ] == 1 )
    {
        SATURN_WRITE_SINGLE( seq0a_2, 0 );    // Rect1
        SATURN_WRITE_SINGLE( seq1a_2, 0 );    // Rect1
        SATURN_WRITE_SINGLE( 0x10, 0xFF );
    }
    else if ( cmd->cmdDataBuf[ 2 ] == 0 && cmd->cmdDataBuf[ 1 ] == 2 )
    {
        SATURN_WRITE_SINGLE( seq0b_2, 1 );    // Rect2
        SATURN_WRITE_SINGLE( seq1b_2, 1 );    // Rect2
        SATURN_WRITE_SINGLE( 0x11, 0xFF );
    }
}

CmdHandlerReturnCode_T Saturn2_spi::PerPhasePW( CommandData_T* cmd )
{
    for ( int j = 0; j < 2; j++ )
    {
        if ( cmd->cmdDataBuf[ 1 ] == 1 )
        {
            PW2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                | ( cmd->cmdDataBuf[ 4 ] << 16 );
            PW2 = pow( 2.0, 24 ) / PW2;

            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq1a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1a_2 + 2, ( PW2 & 0x0000FF ) );
            }
            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0a_2 + 4, ( PW2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq0a_2 + 3, ( PW2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq0a_2 + 2, ( PW2 & 0x0000FF ) );
            }
            HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
            PW2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                | ( cmd->cmdDataBuf[ 4 ] << 16 );
        }

        else if ( cmd->cmdDataBuf[ 1 ] == 2 )
        {
            PW2phase2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                      | ( cmd->cmdDataBuf[ 4 ] << 16 );
            PW2phase2 = pow( 2.0, 24 ) / PW2phase2;

            if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b_2 + 4, ( PW2phase2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq1b_2 + 3, ( PW2phase2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1b_2 + 2, ( PW2phase2 & 0x0000FF ) );
            }
            else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b_2 + 4, ( PW2phase2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq0b_2 + 3, ( PW2phase2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq0b_2 + 2, ( PW2phase2 & 0x0000FF ) );
            }
            HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
            PW2phase2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                      | ( cmd->cmdDataBuf[ 4 ] << 16 );
        }
    }
    Saturn2_spi::chargecheck( cmd );
    return Saturn2_spi::update_delay( cmd );
}

void Saturn2_spi::PerPhaseAmp( CommandData_T* cmd )
{
    for ( int i = 0; i < 27; i++ )
    {
        if ( mapping2[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a_2 + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b_2 + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a_2 + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b_2 + 8;

            for ( int j = 0; j < 2; j++ )
            {
                if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 0 )
                {
                    uint16_t AMP = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 );
                    AMP          = AMP / 15;
                    uint8_t AMPL = AMP & 0xFF;
                    uint8_t AMPU = ( AMP >> 8 ) & 0x03;

                    if ( cmd->cmdDataBuf[ 1 ] == 1 )
                    {
                        SATURN_WRITE_SINGLE( phase3 - 1, AMPL );
                        SATURN_WRITE_MASK( phase3, AMPU, 3 );
                    }
                    else if ( cmd->cmdDataBuf[ 1 ] == 2 )
                    {
                        SATURN_WRITE_SINGLE( phase4 - 1, AMPL );
                        SATURN_WRITE_MASK( phase4, AMPU, 3 );
                    }
                }

                else if ( HAL_GPIO_ReadPin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin ) == 1 )
                {
                    uint16_t AMP = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 );
                    AMP          = AMP / 15;
                    uint8_t AMPL = AMP & 0xFF;
                    uint8_t AMPU = ( AMP >> 8 ) & 0x03;

                    if ( cmd->cmdDataBuf[ 1 ] == 1 )
                    {
                        SATURN_WRITE_SINGLE( phase1 - 1, AMPL );
                        SATURN_WRITE_MASK( phase1, AMPU, 3 );
                        AMP2ph1[ i ] = AMP * 15;
                    }
                    else if ( cmd->cmdDataBuf[ 1 ] == 2 )
                    {
                        SATURN_WRITE_SINGLE( phase2 - 1, AMPL );
                        SATURN_WRITE_MASK( phase2, AMPU, 3 );
                        AMP2ph2[ i ] = AMP * 15;
                    }
                }
                HAL_GPIO_TogglePin( LF_STIM_SEL_Port, LF_STIM_SEL_Pin );
            }
        }
    }
    Saturn2_spi::chargecheck( cmd );
}

void Saturn2_spi::EditChargeLimit( CommandData_T* cmd )
{
    for ( int i = 0; i < 27; i++ )
    {
        if ( mapping2[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            chargelimits2[ cmd->cmdDataBuf[ 0 ] ] = cmd->cmdDataBuf[ 1 ];
        }
    }
    if ( cmd->cmdDataBuf[ 0 ] == 0 )
    {
        memset( chargelimits2, 0, 5 * sizeof( uint16_t ) );
    }
}

void Saturn2_spi::chargecheck( CommandData_T* cmd )
{
    bool error = false;
    static uint8_t outbuff[ 27 ];

    for ( int i = 0; i < 27; i++ )
    {
        if ( ( ( AMP2ph1[ i ] * PW2 ) / 10000 > chargelimits2[ mapping2[ i ] ]
               || ( AMP2ph2[ i ] * PW2phase2 ) / 10000 > chargelimits2[ mapping2[ i ] ] )
             && chargelimits2[ mapping2[ i ] ] != 0 )
        {
            outbuff[ i ] = mapping2[ i ];
            error        = true;
        }
        else
        {
            outbuff[ i ] = 0;
        }
    }
    if ( error )
    {
        SendUCmdResponse( CT_ChargeLimit, CMD_RESP_E0A, outbuff, 27 );
    }
}
