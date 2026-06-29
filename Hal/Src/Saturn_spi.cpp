/// @file    Saturn_spi.cpp
/// @brief   This module handles the SPI interface to the
///          "Saturn 1" or "High Frequency" ASIC.
/// @details Handles the power up sequence of the Saturn chip,
/// 		 contains the functions to control stim parameters.

#include "Saturn_spi.hpp"
#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"
#include "CommandHandler.hpp"
#include "FRAM.hpp"
#include "Impedance.hpp"
#include "PortDefinitions.h"
#include "Saturn2_spi.hpp"
// #include "assertLocal.h"
#include "app_entry.h"
#include "main.h"
#include "stm32wb55xx.h"
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


volatile unsigned int count                            = 0;
unsigned int SPEED                                     = 1;    // Current MCLK Rate (MHz)
unsigned int ARG7                                      = 0;    // Scratch register holding the last SPI read result
unsigned int ARG6                                      = 0;    // BBC startup loop counter (diagnostic)
unsigned int STIM_UP                                   = 0;    // Status of STIM USCI interface
// mapping[i] = electrode group assigned to physical Saturn channel i (0 = unassigned)
uint8_t mapping[ 27 ] __attribute__( ( aligned( 4 ) ) ) = { 0 };
unsigned int startdelay                                = 0;    // ms before first pulse after Starttick()
uint32_t PW                                            = 10;   // Phase-1 pulse width in µs (raw command value)
uint32_t PWphase2                                      = 10;   // Phase-2 pulse width in µs (raw command value)
uint16_t frequency                                     = 2;    // Stimulation frequency in Hz
uint32_t delay1reg                                     = 0;    // Interphase delay register value (µs, 0-based)
uint32_t delay2reg                                     = 0;    // Inter-pulse delay register value (µs, 0-based)
int64_t starttick                                      = -2;   // HAL tick when the current on-cycle started (-2 = idle)
uint32_t duration                                      = 0;    // Stim-on duration in ms (0 = continuous)
uint32_t offduration                                   = 0;    // Stim-off duration in ms between on-cycles
uint32_t stoppoint                                     = 0;    // Total elapsed ticks after which cycling stops (0 = run forever)
int64_t ogtick                                         = 0;    // HAL tick captured at Starttick() for stoppoint math
uint16_t chargelimits[ 5 ]                             = { 0, 25, 25, 25, 25 };  // Per-electrode charge limit in nC (0 = disabled)
uint16_t AMPph1[ 27 ]                                  = { 0 };  // Last written phase-1 amplitude per channel (raw µA)
uint16_t AMPph2[ 27 ]                                  = { 0 };  // Last written phase-2 amplitude per channel (raw µA)

// Sequencer bank base addresses in the Saturn register map.
// Each bank encodes one phase of a biphasic pulse.  Period 1 (fundamental)
// uses banks 0x80/0xC0/0x100 and 0x200/0x240/0x280.  Period 2 (2nd harmonic)
// is switched to a different address set by UpdateSequencerAddresses(2).
uint16_t seq0a     = 0x80;     // Phase-A sequencer bank 0 base (anodic phase, period 1)
uint16_t seq0b     = 0xC0;     // Phase-B sequencer bank 0 base (cathodic phase, period 1)
uint16_t seq0delay = 0x100;    // Inter-pulse delay bank 0 base (period 1)
uint16_t seq1a     = 0x200;    // Phase-A sequencer bank 1 base (anodic phase, period 1)
uint16_t seq1b     = 0x240;    // Phase-B sequencer bank 1 base (cathodic phase, period 1)
uint16_t seq1delay = 0x280;    // Inter-pulse delay bank 1 base (period 1)

struct ramping
{
    static volatile uint8_t electrode, duty_cycle, rep_freq;
    static volatile uint16_t min_amplitude, max_amplitude, periods_in_total, step_size;
    static volatile uint32_t ramp_off_duration;
    static bool ramping_en;
};

volatile uint8_t ramping::electrode          = 0;
volatile uint8_t ramping::duty_cycle         = 0;
volatile uint8_t ramping::rep_freq           = 0;
volatile uint16_t ramping::min_amplitude     = 0;
volatile uint16_t ramping::max_amplitude     = 0;
volatile uint32_t ramping::ramp_off_duration = 0;
volatile uint16_t ramping::periods_in_total  = 0;
volatile uint16_t ramping::step_size         = 0;
bool ramping::ramping_en                     = false;

#define total_period_mS( x ) ( 1000 / x )
#define num_channels         sizeof( mapping ) / sizeof( mapping[ 0 ] )

namespace Saturn1_spi
{

static SPI_HandleTypeDef handle;

// The chip select line. Active low.
static dio_stm32 chip_select { (std::uint32_t)HF_STIM_CSn_Port,
                               HF_STIM_CSn_Pin,
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
}    // namespace Saturn1_spi

SPI_HandleTypeDef* Saturn1_spi::get_handle_ptr( void )
{
    return &Saturn1_spi::handle;
}

void Saturn1_spi::enable( void )
{
    Saturn1_spi::peripheral_init();

    return;
}

void Saturn1_spi::peripheral_init( void )
{
    Saturn1_spi::handle.Instance               = SPI1;
    Saturn1_spi::handle.Init.Mode              = SPI_MODE_MASTER;
    Saturn1_spi::handle.Init.Direction         = SPI_DIRECTION_2LINES;
    Saturn1_spi::handle.Init.DataSize          = SPI_DATASIZE_8BIT;
    Saturn1_spi::handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    Saturn1_spi::handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    Saturn1_spi::handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    Saturn1_spi::handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    Saturn1_spi::handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    Saturn1_spi::handle.Init.CRCPolynomial     = 10;
    Saturn1_spi::handle.Init.CRCPolynomial     = 7;
    Saturn1_spi::handle.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    Saturn1_spi::handle.Init.NSS               = SPI_NSS_SOFT;
    Saturn1_spi::handle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    Saturn1_spi::handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;

    if ( HAL_SPI_Init( &Saturn1_spi::handle ) != HAL_OK )
    {
        Error_Handler();
    }

    return;
}

void Saturn1_spi::gpio_init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin       = HF_STIM_SCK_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( HF_STIM_SCK_Port, &GPIO_InitStruct );

    GPIO_InitStruct.Pin       = HF_STIM_MISO_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( HF_STIM_MISO_Port, &GPIO_InitStruct );

    GPIO_InitStruct.Pin       = HF_STIM_MOSI_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init( HF_STIM_MOSI_Port, &GPIO_InitStruct );

    Saturn1_spi::chip_select.deactivate();

    return;
}

void Saturn1_spi::gpio_deinit( void )
{
    __HAL_RCC_SPI1_CLK_DISABLE();

    HAL_GPIO_DeInit( HF_STIM_MOSI_Port, HF_STIM_MOSI_Pin );
    HAL_GPIO_DeInit( HF_STIM_MISO_Port, HF_STIM_MISO_Pin );
    HAL_GPIO_DeInit( HF_STIM_SCK_Port, HF_STIM_SCK_Pin );

    return;
}

void Saturn1_spi::STIM_POWER_UP( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    GPIO_InitStruct.Pin   = DVDDIO_EN_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

    Saturn1_spi::dvddio_en.activate();

    STIM_UP = 1;
}

void Saturn1_spi::PULSE_STIM_CLK( unsigned int pulses )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_GPIOE_CLK_ENABLE();
    count = 0;
    while ( count < pulses )    // Do Pulse Train
    {
        GPIO_InitStruct.Pin   = HF_STIM_CLK_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init( HF_STIM_CLK_Port, &GPIO_InitStruct );

        HAL_GPIO_WritePin( HF_STIM_CLK_Port, HF_STIM_CLK_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin( HF_STIM_CLK_Port, HF_STIM_CLK_Pin, GPIO_PIN_RESET );

        count++;
    }
}

void Saturn1_spi::PULSE_STIM_SCK( unsigned int pulses )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    if ( STIM_UP > 0 )
    {
        GPIO_InitStruct.Pin   = HF_STIM_SCK_Pin;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init( HF_STIM_SCK_Port, &GPIO_InitStruct );
        HAL_GPIO_WritePin( HF_STIM_SCK_Port, HF_STIM_SCK_Pin, GPIO_PIN_RESET );
    }
    count = 0;
    while ( count < pulses )    // Do Pulse Train
    {
        HAL_GPIO_WritePin( HF_STIM_SCK_Port, HF_STIM_SCK_Pin, GPIO_PIN_SET );
        HAL_GPIO_WritePin( HF_STIM_SCK_Port, HF_STIM_SCK_Pin, GPIO_PIN_RESET );

        count++;
    }
    if ( STIM_UP > 0 )
    {
        GPIO_InitStruct.Pin       = HF_STIM_SCK_Pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init( HF_STIM_SCK_Port, &GPIO_InitStruct );
    }
}

// Saturn SPI write frame: [addr_hi][addr_lo][data] (3 bytes, CS active-low).
// gpio_init/deinit is called on every transaction because SPI2 bus is shared with FRAM.
void Saturn1_spi::SATURN_WRITE_SINGLE( unsigned int address, unsigned int data )
{
    unsigned int highaddr = ( address & 0xFF00 ) >> 8;
    unsigned int lowaddr  = ( address & 0xFF );
    HAL_StatusTypeDef hal_status;

    uint8_t array[ 3 ];
    array[ 0 ] = highaddr;
    array[ 1 ] = lowaddr;
    array[ 2 ] = data;

    Saturn1_spi::gpio_init();
    Saturn1_spi::chip_select.activate();
    hal_status = HAL_SPI_Transmit( &Saturn1_spi::handle, (uint8_t*)array, 3, 500 );
    Saturn1_spi::chip_select.deactivate();
    Saturn1_spi::gpio_deinit();
}

// Saturn SPI read frame: [addr_hi | 0x80][addr_lo] TX, then 4 bytes RX.
// The read bit is the MSB of the high address byte.  Result lands in ARG7.
unsigned int Saturn1_spi::SATURN_READ_SINGLE( unsigned int address )
{
    unsigned int highaddr  = ( address & 0xFF00 ) >> 8;
    unsigned int lowaddr   = ( address & 0xFF );
    highaddr              |= 0x80;    // Set read bit
    HAL_StatusTypeDef hal_status;

    uint8_t array[ 2 ];
    array[ 0 ] = highaddr;
    array[ 1 ] = lowaddr;

    Saturn1_spi::gpio_init();
    Saturn1_spi::chip_select.activate();
    hal_status = HAL_SPI_Transmit( &Saturn1_spi::handle, (uint8_t*)array, 2, 500 );

    if ( hal_status == HAL_OK )
    {
        hal_status = HAL_SPI_Receive( &Saturn1_spi::handle, (uint8_t*)( &ARG7 ), 4, 500 );
        ARG7       = ( ARG7 & 0xFF00 ) >> 8;
    }
    Saturn1_spi::chip_select.deactivate();
    Saturn1_spi::gpio_deinit();
    return ARG7;
}

CmdHandlerReturnCode_T Saturn1_spi::ReadReturn( CommandData_T* cmd )
{
    //	uint16_t address=(cmd->cmdDataBuf[2]) | (cmd->cmdDataBuf[3]<<8);
    //	Saturn1_spi::SATURN_READ_SINGLE (address);
    //	cmd->cmdDataBuf[0]= (address & 0xFF00) >> 8;
    //	cmd->cmdDataBuf[1]=(address & 0xFF);
    //	cmd->cmdDataBuf[2]=ARG7;
    //	cmd->cmdDataLen=3;
}

void Saturn1_spi::SATURN_WRITE_MASK( unsigned int address, unsigned int data, unsigned int mask )
{
    SATURN_READ_SINGLE( address );    // Fetch existing register to ARG7
    SATURN_WRITE_SINGLE(
        address,
        ( ( ARG7 &= ~mask ) | ( data & mask ) ) );    // Apply masked update to register
}

void Saturn1_spi::SATURN_DIG_STARTUP( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    HAL_GPIO_WritePin( HF_STIM_RSTn_Port, HF_STIM_RSTn_Pin, GPIO_PIN_RESET );    // Set RST_N low

    GPIO_InitStruct.Pin  = HF_STIM_SEL_Pin | HF_STIM_RUN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( GPIOH, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = HF_STIM_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = HF_STIM_STOP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( HF_STIM_STOP_Port, &GPIO_InitStruct );

    HAL_Delay( 50 );    // Wait during pull-up interval
    STIM_POWER_UP();    // Apply DVDD_STIM

    HAL_Delay( 3 );
    Saturn1_spi::enable();    // Init SPI lines
    PULSE_STIM_SCK( 4 );      // Toggle SCK 4x

    HAL_Delay( 3 );
    Saturn1_spi::avdd_en.activate();    // Set AVDD high
    HAL_Delay( 3 );

    GPIO_InitStruct.Pin  = HF_STIM_SEL_Pin | HF_STIM_RUN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOH, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = HF_STIM_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOD, &GPIO_InitStruct );

    GPIO_InitStruct.Pin  = HF_STIM_STOP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init( HF_STIM_STOP_Port, &GPIO_InitStruct );

    HAL_GPIO_WritePin( HF_STIM_RSTn_Port, HF_STIM_RSTn_Pin, GPIO_PIN_SET );    // Set RST_N high

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
    SATURN_READ_SINGLE( 0x800 );           // RAM1[0] -> ARG7
    SATURN_WRITE_SINGLE( 0x900, 0x00 );    // Write 0x00 to RAM2[0]
    SATURN_READ_SINGLE( 0x900 );           // RAM2[0] -> ARG7
}

void Saturn1_spi::SATURN_BBC_STARTUP_R1( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    int START_COUNT = 0;    // BBC Startup Loop Counter

    GPIO_InitStruct.Pin   = SAT_VHV_SW_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

    Saturn1_spi::hvdd_en.activate();    // VHV_SW enabled
    Saturn1_spi::sat_boost_en.activate();

    HAL_Delay( 10 );                      //~10mS settling delay
    SATURN_WRITE_SINGLE( 0x24, 0x81 );    // CG_CTRL <- 0x81 (Enable Current Sources)

    // SATURN_WRITE_MASK (0x01,0x03,0x03);	//ENABLE <- Bits 0x02 & 0x01
    // (Enable Bandgap, BC)
    HAL_Delay( 2 );                       //~2mS settling delay
    SATURN_WRITE_SINGLE( 0x20, 0x0C );    // BC_CTRL0 <- 0x0C
    SATURN_WRITE_SINGLE( 0x21, 0x0B );    // BC_CTRL1 <- 0x0B

    HAL_Delay( 1 );                       //~1mS settling delay
    SATURN_WRITE_SINGLE( 0x21, 0x1B );    // BC_CTRL1 <- 0x1B

    HAL_Delay( 10 );                      // Wait 10mS
    ARG7 = 0;

    while ( ( START_COUNT < 100 ) &&          // Loop until we've started up, or tried
            ( !( ARG7 && 0x80 ) ) )           // 	too many times:
    {
        SATURN_READ_SINGLE( 0x21 );           // Get BC_CTRL1 > ARG7
        HAL_Delay( 1 );                       // Wait 1mS
        START_COUNT += 1;                     // Increment Loop Ctr
    }
    ARG6 = START_COUNT;                       // Report Loop Ctr for Monitoring
    if ( ( ARG7 && 0x80 ) )                   // Is BCG set?
    {
        SATURN_WRITE_SINGLE( 0x1F, 0x02 );    // VG_CTRL <- 0x02 (HVDDL Enable)
        HAL_Delay( 2 );                       // ~2mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x06 );    // VG_CTRL <- Bit 0x04 (Assert LV Reset)
        HAL_Delay( 1 );                       // ~1mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x02 );    // VG_CTRL <- Bit 0x04 (Clear LV Reset)
        HAL_Delay( 1 );                       // ~1mS settling time
        SATURN_WRITE_SINGLE( 0x1F, 0x03 );    // VG_CTRL <- Bit 0x01 (CAS Enable)
        HAL_Delay( 2 );                       // ~2mS settling time
        ARG7 = 0;                             // Return Success
    }
}

void Saturn1_spi::read_registers( CommandData_T* cmd )
{
    //std::ofstream outFile("output.txt");

    for ( int address = 0; address < 0x640; address++ )
    {
        Saturn1_spi::SATURN_READ_SINGLE( address );
        cmd->cmdDataBuf[ 0 ] = ( address & 0xFF00 ) >> 8;
        cmd->cmdDataBuf[ 1 ] = ( address & 0xFF );
        cmd->cmdDataBuf[ 2 ] = ARG7;
        cmd->cmdDataLen      = 3;

        //outFile << "%d, " << address << "%d" << ARG7 << std::endl;
        //printf("%d, %d\n", address, ARG7);
        SendUCmdResponse( CT_ReadRegisters, CMD_RESP_ACK, cmd->cmdDataBuf, cmd->cmdDataLen );
        HAL_Delay( 25 );
    }
       //outFile.close();
}

void Saturn1_spi::DEFAULT_STIM( void )
{
    mapping[25] = 1;
    mapping[22] = 1;
    mapping[18] = 2;
    mapping[16] = 2;
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
    SATURN_WRITE_SINGLE( 0x0011, 0xFF );
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
    SATURN_WRITE_SINGLE( 0x0046, 0x0A );
    SATURN_WRITE_SINGLE( 0x0047, 0x00 );
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
    SATURN_WRITE_SINGLE( 0x00C1, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C2, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C3, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C4, 0x01 );
    SATURN_WRITE_SINGLE( 0x00C5, 0x00 );
    SATURN_WRITE_SINGLE( 0x00C6, 0x00 );
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
    SATURN_WRITE_SINGLE( 0x0102, 0x00 );
    SATURN_WRITE_SINGLE( 0x0103, 0x00 );
    SATURN_WRITE_SINGLE( 0x0104, 0x01 );
    SATURN_WRITE_SINGLE( 0x0105, 0x00 );
    SATURN_WRITE_SINGLE( 0x0106, 0x00 );
    SATURN_WRITE_SINGLE( 0x0107, 0x00 );
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
    SATURN_WRITE_SINGLE( 0x0141, 0x00 );
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

    HAL_GPIO_WritePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_CONT_Port, HF_STIM_CONT_Pin, GPIO_PIN_RESET );
}

void Saturn1_spi::default_chunk_write( unsigned int start_address )
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

// Each sequencer bank entry has an enable bit (bit 4 = 0x10).
// Channels are stored as pairs of bytes in the bank starting at offset +8.
void Saturn1_spi::ElectrodeEnable( CommandData_T* cmd )
{
    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b + 8;

            SATURN_WRITE_MASK( phase1, 16, 16 );
            SATURN_WRITE_MASK( phase2, 16, 16 );
            SATURN_WRITE_MASK( phase3, 16, 16 );
            SATURN_WRITE_MASK( phase4, 16, 16 );
        }
    }
}

void Saturn1_spi::DisableElectrode( CommandData_T* cmd )
{
    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b + 8;

            SATURN_WRITE_MASK( phase1, 0, 16 );
            SATURN_WRITE_MASK( phase2, 0, 16 );
            SATURN_WRITE_MASK( phase3, 0, 16 );
            SATURN_WRITE_MASK( phase4, 0, 16 );
        }
    }
}

void Saturn1_spi::ElectrodetoChannel( CommandData_T* cmd )
{
    std::unordered_map< int, int > hashMap;
    hashMap[ 1 ] = 25;
    hashMap[ 2 ] = 22;
    hashMap[ 3 ] = 18;
    hashMap[ 4 ] = 16;

    uint16_t phase1 = 2 * ( hashMap[cmd->cmdDataBuf[ 0 ]] ) + seq0a + 8;    // Phase 1 Address XA
    uint16_t phase2 = 2 * ( hashMap[cmd->cmdDataBuf[ 0 ]] ) + seq0b + 8;    // Phase 2 Address XA
    uint16_t phase3 = 2 * ( hashMap[cmd->cmdDataBuf[ 0 ]] ) + seq1a + 8;
    uint16_t phase4 = 2 * ( hashMap[cmd->cmdDataBuf[ 0 ]] ) + seq1b + 8;

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

    mapping[ hashMap[cmd->cmdDataBuf[ 0 ]] ] = cmd->cmdDataBuf[ 2 ];
}

// Captures the current HAL tick so CurrentTick() can compute elapsed time.
// Called from cmdHndlStartContinuousStim — must be called before stimulation begins.
void Saturn1_spi::Starttick( void )
{
    starttick = HAL_GetTick();
    ogtick    = starttick;
}

void Saturn1_spi::StartContinuousStim( void )
{
    SATURN_WRITE_SINGLE( 0x0007, 0xFF );
    SATURN_WRITE_SINGLE( 0x0008, 0xFF );
    SATURN_WRITE_SINGLE( 0x0009, 0xFF );
    SATURN_WRITE_SINGLE( 0x000A, 0xFF );
    SATURN_WRITE_SINGLE( 0x000B, 0x03 );

    if ( harmonics )
    {
        SATURN_WRITE_SINGLE( 0x000C, 0x01 );    // SEQ0A
        SATURN_WRITE_SINGLE( 0x000D, 0x06 );    // SEQ0B
        SATURN_WRITE_SINGLE( 0x000E, 0x07 );    // SEQ1A
        SATURN_WRITE_SINGLE( 0x000F, 0x0C );    // SEQ1B
    }

    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_STOP_Port, HF_STIM_STOP_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_CONT_Port, HF_STIM_CONT_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_SET );
}

void Saturn1_spi::Amplitude( CommandData_T* cmd )
{
    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b + 8;

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
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

            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMPph1[ i ]  = AMP;
                AMPph2[ i ]  = AMP;
                AMP          = AMP / 15;

                uint8_t AMPL = AMP & 0xFF;
                SATURN_WRITE_SINGLE( phase1 - 1, AMPL );    // Channel 1, phase 1
                SATURN_WRITE_SINGLE( phase2 - 1, AMPL );    // Channel 1, phase 2

                uint8_t AMPU = ( AMP >> 8 ) & 0x03;
                SATURN_WRITE_MASK( phase1, AMPU, 3 );    // Channel 1, phase 1
                SATURN_WRITE_MASK( phase2, AMPU, 3 );    // Channel 1, phase 2
            }
            HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
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

            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                uint16_t AMP = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
                AMPph1[ i ]  = AMP;
                AMPph2[ i ]  = AMP;
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
    Saturn1_spi::chargecheck( cmd );
}

// PulseWidth converts the commanded period (µs, max 10 000 000) to the Saturn
// register value using: register = 2^24 / period_µs.
// The function writes to both SEL=0 and SEL=1 sequencer banks by toggling
// HF_STIM_SEL between the two writes — this ensures both physical ASIC
// sequencer sets have identical timing.
CmdHandlerReturnCode_T Saturn1_spi::PulseWidth( CommandData_T* cmd )
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
        if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
        {
            PW = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
               | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW = pow( 2.0, 24 ) / PW;    // Convert µs period to ASIC clock divisor
            SATURN_WRITE_SINGLE( seq1a + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1a + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1a + 2, ( PW & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq1b + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1b + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1b + 2, ( PW & 0x0000FF ) );
        }

        else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
        {
            PW = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
               | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW = pow( 2.0, 24 ) / PW;
            SATURN_WRITE_SINGLE( seq0a + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0a + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0a + 2, ( PW & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq0b + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0b + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0b + 2, ( PW & 0x0000FF ) );
        }
        HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );
        if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
        {
            PW = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
               | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW = pow( 2.0, 24 ) / PW;
            SATURN_WRITE_SINGLE( seq1a + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1a + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1a + 2, ( PW & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq1b + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq1b + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq1b + 2, ( PW & 0x0000FF ) );
        }

        else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
        {
            PW = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
               | ( cmd->cmdDataBuf[ 3 ] << 16 );
            PW = pow( 2.0, 24 ) / PW;
            SATURN_WRITE_SINGLE( seq0a + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0a + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0a + 2, ( PW & 0x0000FF ) );

            SATURN_WRITE_SINGLE( seq0b + 4, ( PW & 0xFF0000 ) >> 16 );
            SATURN_WRITE_SINGLE( seq0b + 3, ( PW & 0x00FF00 ) >> 8 );
            SATURN_WRITE_SINGLE( seq0b + 2, ( PW & 0x0000FF ) );
        }

        PW = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 )
           | ( cmd->cmdDataBuf[ 3 ] << 16 );
        PWphase2 = PW;
        Saturn1_spi::chargecheck( cmd );
        return Saturn1_spi::update_delay( cmd );
    }
}

void Saturn1_spi::StartDelay( CommandData_T* cmd )
{
    startdelay = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
}

CmdHandlerReturnCode_T Saturn1_spi::Frequency( CommandData_T* cmd )
{
    if ( ( ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) ) == 0 )
    {
        frequency = 2;
        return CmdHndlRetFreqZero;
    }
    else
    {
        frequency = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
        return Saturn1_spi::update_delay( cmd );
    }
}

CmdHandlerReturnCode_T Saturn1_spi::InterphaseDelay( CommandData_T* cmd )
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
        {                                 // If they send a delay that is 2us or greater
            delay1reg = IDrequest - 2;    // Account for the built in delay
                                          // IDIRL = IDrequest;
        }
        else if ( IDrequest <= 1 )
        {                     // If the delay is 0us or 1us
            delay1reg = 0;    // Set the delay to 0us
                              // IDIRL = 2;
        }

        if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
        {
            SATURN_WRITE_SINGLE( seq0a + 6, ( delay1reg & 0xFF ) );           // DLYL
            SATURN_WRITE_SINGLE( seq0a + 7, ( delay1reg & 0xFF00 ) >> 8 );    // DLYU
        }

        else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
        {
            SATURN_WRITE_SINGLE( seq1a + 6, ( delay1reg & 0xFF ) );
            SATURN_WRITE_SINGLE( seq1a + 7, ( delay1reg & 0xFF00 ) >> 8 );
        }
        HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );

        if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
        {
            SATURN_WRITE_SINGLE( seq0a + 6, ( delay1reg & 0xFF ) );           // DLYL
            SATURN_WRITE_SINGLE( seq0a + 7, ( delay1reg & 0xFF00 ) >> 8 );    // DLYU
        }

        else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
        {
            SATURN_WRITE_SINGLE( seq1a + 6, ( delay1reg & 0xFF ) );
            SATURN_WRITE_SINGLE( seq1a + 7, ( delay1reg & 0xFF00 ) >> 8 );
        }

        return Saturn1_spi::update_delay( cmd );
    }
}

// Recalculates the inter-pulse dead-time register after any change to frequency,
// pulse width, or interphase delay.
// Formula: dead_time_µs = (1 000 000 / Hz) - PW_ph1 - PW_ph2 - interphase_delay
// If the sum of pulse widths exceeds one period, returns CmdHndlRetFreqTooSmall.
// Delays longer than 0xFFFF µs overflow to AddDelay() which uses a third
// sequencer phase to extend the dead time.
CmdHandlerReturnCode_T Saturn1_spi::update_delay( CommandData_T* cmd )
{
    if ( frequency != 0 )
    {
        delay2reg = ( 1000000 / frequency ) - PW - PWphase2 - delay1reg;
    }
    else
    {
        return CmdHndlRetFreqZero;
    }
    if ( PW + PWphase2 + delay1reg > ( 1000000 / frequency ) )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetFreqTooSmall;
    }
    else
    {
        if ( delay2reg > 1 )
        {                      // If the delay is 2us or greater
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
            SATURN_WRITE_SINGLE( 0x000D, ( seq0delay / 64 ) - 1 );    // SEQ0B
            SATURN_WRITE_SINGLE( 0x000F, ( seq1delay / 64 ) - 1 );    // SEQ1B

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b + 6, ( delay2reg & 0xFF ) );           // DLYL
                SATURN_WRITE_SINGLE( seq0b + 7, ( delay2reg & 0xFF00 ) >> 8 );    // DLYU
                SATURN_WRITE_SINGLE( seq0delay + 6, 0 );                          // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq0delay + 7, 0 );                          // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq0delay + 4, 0xFF );
                SATURN_WRITE_SINGLE( seq0delay + 3, 0xFF );
                SATURN_WRITE_SINGLE( seq0delay + 2, 0xFF );
            }
            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b + 6, ( delay2reg & 0xFF ) );
                SATURN_WRITE_SINGLE( seq1b + 7, ( delay2reg & 0xFF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1delay + 6, 0 );    // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq1delay + 7, 0 );    // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq1delay + 4, 0xFF );
                SATURN_WRITE_SINGLE( seq1delay + 3, 0xFF );
                SATURN_WRITE_SINGLE( seq1delay + 2, 0xFF );
            }
            HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b + 6, ( delay2reg & 0xFF ) );           // DLYL
                SATURN_WRITE_SINGLE( seq0b + 7, ( delay2reg & 0xFF00 ) >> 8 );    // DLYU
                SATURN_WRITE_SINGLE( seq0delay + 6, 0 );                          // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq0delay + 7, 0 );                          // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq0delay + 4, 0xFF );
                SATURN_WRITE_SINGLE( seq0delay + 3, 0xFF );
                SATURN_WRITE_SINGLE( seq0delay + 2, 0xFF );
            }
            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b + 6, ( delay2reg & 0xFF ) );
                SATURN_WRITE_SINGLE( seq1b + 7, ( delay2reg & 0xFF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1delay + 6, 0 );    // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq1delay + 7, 0 );    // DLYL Phase 3
                SATURN_WRITE_SINGLE( seq1delay + 4, 0xFF );
                SATURN_WRITE_SINGLE( seq1delay + 3, 0xFF );
                SATURN_WRITE_SINGLE( seq1delay + 2, 0xFF );
            }
        }
        else
        {
            Saturn1_spi::AddDelay( delay2reg );
        }

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

void Saturn1_spi::AddDelay( unsigned int delay2 )
{
    SATURN_WRITE_SINGLE( 0x000D, ( seq0delay / 64 ) - 1 );    // SEQ0B
    SATURN_WRITE_SINGLE( 0x000F, ( seq1delay / 64 ) - 1 );    // SEQ1B
    uint32_t remainder = delay2 - 0xFFFF - 2;
    uint32_t extrapw   = 2;

    if ( remainder > 0xFFFF )
    {
        extrapw   = remainder - 0xFFFF + 2;
        remainder = 0xFFFF;
    }
    extrapw = pow( 2.0, 24 ) / extrapw;

    if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
    {
        SATURN_WRITE_SINGLE( seq0b + 6, 0xFF );                        // DLYL Phase 2
        SATURN_WRITE_SINGLE( seq0b + 7, 0xFF );                        // DLYU Phase 2
        SATURN_WRITE_SINGLE( seq0delay + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay + 7,
                             ( remainder & 0xFF00 ) >> 8 );            // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq0delay + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq0delay + 2, ( extrapw & 0x0000FF ) );
    }
    else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
    {
        SATURN_WRITE_SINGLE( seq1b + 6, 0xFF );
        SATURN_WRITE_SINGLE( seq1b + 7, 0xFF );
        SATURN_WRITE_SINGLE( seq1delay + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay + 7,
                             ( remainder & 0xFF00 ) >> 8 );            // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq1delay + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq1delay + 2, ( extrapw & 0x0000FF ) );
    }
    HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );

    if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
    {
        SATURN_WRITE_SINGLE( seq0b + 6, 0xFF );                        // DLYL Phase 2
        SATURN_WRITE_SINGLE( seq0b + 7, 0xFF );                        // DLYU Phase 2
        SATURN_WRITE_SINGLE( seq0delay + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay + 7,
                             ( remainder & 0xFF00 ) >> 8 );            // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq0delay + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq0delay + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq0delay + 2, ( extrapw & 0x0000FF ) );
    }
    else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
    {
        SATURN_WRITE_SINGLE( seq1b + 6, 0xFF );
        SATURN_WRITE_SINGLE( seq1b + 7, 0xFF );
        SATURN_WRITE_SINGLE( seq1delay + 6, ( remainder & 0xFF ) );    // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay + 7,
                             ( remainder & 0xFF00 ) >> 8 );            // DLYL Phase 3
        SATURN_WRITE_SINGLE( seq1delay + 4, ( extrapw & 0xFF0000 ) >> 16 );
        SATURN_WRITE_SINGLE( seq1delay + 3, ( extrapw & 0x00FF00 ) >> 8 );
        SATURN_WRITE_SINGLE( seq1delay + 2, ( extrapw & 0x0000FF ) );
    }
}

void Saturn1_spi::StopContinuousStim( void )
{
    // HAL_GPIO_TogglePin( HF_STIM_RUN_Port, SD_CS_Pin );    // used for debugging timing before
    // board calibration
    HAL_GPIO_WritePin( HF_STIM_CONT_Port, HF_STIM_CONT_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_RESET );
    if ( measuringimp )
    {
        Impedance::StopMeasurement();
        measuringimp = false;
    }
}

// Called from the TIM17 1-ms ISR.  Implements the stim on/off duty cycle:
//   - Starts stim 1 tick after Starttick() (or after startdelay ms if set)
//   - Stops stim after 'duration' ms if duration != 0
//   - Restarts stim after 'offduration' ms (cycling), up to 'stoppoint' total ticks
void Saturn1_spi::CurrentTick( uint32_t tick )
{
    if ( ( tick == starttick + 1 )
         || ( ( tick == starttick + offduration + duration + 1 ) && stoppoint != 0 ) )
    {
        if ( ( stoppoint != 0 ) && ( tick < ( ogtick + stoppoint ) ) )
        {
            starttick = tick - 1;
            Saturn1_spi::StartContinuousStim();
        }
        else if ( stoppoint == 0 && startdelay == 0 )
        {
            Saturn1_spi::StartContinuousStim();
        }
        else if ( ( stoppoint != 0 ) && ( tick >= ogtick + stoppoint ) )
        {
        }
    }
    else if ( ( tick == starttick + duration + 1 ) && ( duration != 0 ) )
    {
        Saturn1_spi::StopContinuousStim();
    }
    else if ( tick == starttick + startdelay + 1 )
    {
        Saturn1_spi::StartContinuousStim();
    }
}

void Saturn1_spi::StimDuration( CommandData_T* cmd )
{
    duration
        = ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) | ( cmd->cmdDataBuf[ 2 ] << 16 );
}

void Saturn1_spi::OffDuration( CommandData_T* cmd )
{
    offduration
        = ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) | ( cmd->cmdDataBuf[ 2 ] << 16 );
}

void Saturn1_spi::NCycles( CommandData_T* cmd )
{
    stoppoint
        = ( duration + offduration ) * ( ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 ) );
}

// Writes Saturn1 calibration values loaded from FRAM at boot.
// FRAM address layout (mirrors Saturn register map, offset by 0x100):
//   0x122 -> Saturn reg 0x022  (current source trim)
//   0x134 -> Saturn reg 0x034  (compliance voltage trim)
//   0x137 -> Saturn reg 0x037  (oscillator trim)
//   0x138 -> Saturn reg 0x038  (oscillator trim high byte)
void Saturn1_spi::Calibration( void )
{
    SATURN_WRITE_SINGLE( 0x022, calvals[ 0x122 ] );
    SATURN_READ_SINGLE( 0x022 );
    SATURN_WRITE_SINGLE( 0x034, calvals[ 0x134 ] );
    SATURN_READ_SINGLE( 0x034 );
    SATURN_WRITE_SINGLE( 0x037, calvals[ 0x137 ] );
    SATURN_WRITE_SINGLE( 0x038, calvals[ 0x138 ] );
}

void Saturn1_spi::RAM1( CommandData_T* cmd )
{
    SATURN_WRITE_MASK( 0x00A, 0x0, 0xC0 );
    SATURN_WRITE_MASK( 0x025, 0x30, 0x30 );
    SATURN_WRITE_SINGLE( 2048, 0 );

    for ( int i = 0; i < 256; i++ )
    {
        SATURN_WRITE_SINGLE( ( 2048 + i ), cmd->cmdDataBuf[ i ] );
    }
}

void Saturn1_spi::RAM2( CommandData_T* cmd )
{
    SATURN_WRITE_MASK( 0x00A, 0x0, 0xC0 );
    SATURN_WRITE_MASK( 0x025, 0x30, 0x30 );
    SATURN_WRITE_SINGLE( 2304, 0 );

    for ( int i = 0; i < 256; i++ )
    {
        SATURN_WRITE_SINGLE( ( 2304 + i ), cmd->cmdDataBuf[ i ] );
    }
}

void Saturn1_spi::SelectShape( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 2 ] != 0 && cmd->cmdDataBuf[ 1 ] == 1 )
    {
        SATURN_WRITE_SINGLE( seq0a, 4 );    // Ram1
        SATURN_WRITE_SINGLE( seq1a, 4 );    // Ram1
    }
    else if ( cmd->cmdDataBuf[ 2 ] != 0 && cmd->cmdDataBuf[ 1 ] == 2 )
    {
        SATURN_WRITE_SINGLE( seq0b, 5 );    // Ram2
        SATURN_WRITE_SINGLE( seq1b, 5 );    // Ram2
    }
    else if ( cmd->cmdDataBuf[ 2 ] == 0 && cmd->cmdDataBuf[ 1 ] == 1 )
    {
        SATURN_WRITE_SINGLE( seq0a, 0 );    // Rect1
        SATURN_WRITE_SINGLE( seq1a, 0 );    // Rect1
        SATURN_WRITE_SINGLE( 0x10, 0xFF );
    }
    else if ( cmd->cmdDataBuf[ 2 ] == 0 && cmd->cmdDataBuf[ 1 ] == 2 )
    {
        SATURN_WRITE_SINGLE( seq0b, 1 );    // Rect2
        SATURN_WRITE_SINGLE( seq1b, 1 );    // Rect2
        SATURN_WRITE_SINGLE( 0x11, 0xFF );
    }
}

CmdHandlerReturnCode_T Saturn1_spi::PerPhasePW( CommandData_T* cmd )
{
    for ( int j = 0; j < 2; j++ )
    {
        if ( cmd->cmdDataBuf[ 1 ] == 1 )
        {
            PW = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
               | ( cmd->cmdDataBuf[ 4 ] << 16 );
            PW = pow( 2.0, 24 ) / PW;

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1a + 4, ( PW & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq1a + 3, ( PW & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1a + 2, ( PW & 0x0000FF ) );
            }
            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0a + 4, ( PW & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq0a + 3, ( PW & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq0a + 2, ( PW & 0x0000FF ) );
            }
            HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );
            PW = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
               | ( cmd->cmdDataBuf[ 4 ] << 16 );
        }

        else if ( cmd->cmdDataBuf[ 1 ] == 2 )
        {
            PWphase2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                     | ( cmd->cmdDataBuf[ 4 ] << 16 );
            PWphase2 = pow( 2.0, 24 ) / PWphase2;

            if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
            {
                SATURN_WRITE_SINGLE( seq1b + 4, ( PWphase2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq1b + 3, ( PWphase2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq1b + 2, ( PWphase2 & 0x0000FF ) );
            }
            else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
            {
                SATURN_WRITE_SINGLE( seq0b + 4, ( PWphase2 & 0xFF0000 ) >> 16 );
                SATURN_WRITE_SINGLE( seq0b + 3, ( PWphase2 & 0x00FF00 ) >> 8 );
                SATURN_WRITE_SINGLE( seq0b + 2, ( PWphase2 & 0x0000FF ) );
            }
            HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );
            PWphase2 = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 )
                     | ( cmd->cmdDataBuf[ 4 ] << 16 );
        }
    }
    Saturn1_spi::chargecheck( cmd );
    return Saturn1_spi::update_delay( cmd );
}

void Saturn1_spi::PerPhaseAmp( CommandData_T* cmd )
{
    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b + 8;
            uint16_t phase3 = ( 2 * i ) + seq1a + 8;
            uint16_t phase4 = ( 2 * i ) + seq1b + 8;

            for ( int j = 0; j < 2; j++ )
            {
                if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 0 )
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

                else if ( HAL_GPIO_ReadPin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin ) == 1 )
                {
                    uint16_t AMP = ( cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 );
                    AMP          = AMP / 15;
                    uint8_t AMPL = AMP & 0xFF;
                    uint8_t AMPU = ( AMP >> 8 ) & 0x03;

                    if ( cmd->cmdDataBuf[ 1 ] == 1 )
                    {
                        SATURN_WRITE_SINGLE( phase1 - 1, AMPL );
                        SATURN_WRITE_MASK( phase1, AMPU, 3 );
                        AMPph1[ i ] = AMP * 15;
                    }

                    else if ( cmd->cmdDataBuf[ 1 ] == 2 )
                    {
                        SATURN_WRITE_SINGLE( phase2 - 1, AMPL );
                        SATURN_WRITE_MASK( phase2, AMPU, 3 );
                        AMPph2[ i ] = AMP * 15;
                    }
                }
                HAL_GPIO_TogglePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin );
            }
        }
    }
    Saturn1_spi::chargecheck( cmd );
}

void Saturn1_spi::Min_Amplitude_Ramp( CommandData_T* cmd )
{
    ramping::electrode     = cmd->cmdDataBuf[ 0 ];
    ramping::min_amplitude = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );
}

void Saturn1_spi::Max_Amplitude_Ramp( CommandData_T* cmd )
{
    ramping::max_amplitude = ( cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 );

    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            uint16_t phase1 = ( 2 * i ) + seq0a + 8;
            uint16_t phase2 = ( 2 * i ) + seq0b + 8;

            uint16_t AMP = ( ramping::max_amplitude ) / 15;

            uint8_t AMPL = AMP & 0xFF;
            SATURN_WRITE_SINGLE( phase1 - 1, AMPL );    // Channel 1, phase 1
            SATURN_WRITE_SINGLE( phase2 - 1, AMPL );    // Channel 1, phase 2

            uint8_t AMPU = ( AMP >> 8 ) & 0x03;
            SATURN_WRITE_MASK( phase1, AMPU, 3 );    // Channel 1, phase 1
            SATURN_WRITE_MASK( phase2, AMPU, 3 );    // Channel 1, phase 2
        }
    }
}

void Saturn1_spi::Duty_Cycle( CommandData_T* cmd )
{
    ramping::duty_cycle = cmd->cmdDataBuf[ 0 ];
}

void Saturn1_spi::Rep_Freq( CommandData_T* cmd )
{
    ramping::rep_freq = ( cmd->cmdDataBuf[ 0 ] );

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin              = HF_STIM_SYNC_Pin;
    GPIO_InitStruct.Mode             = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( HF_STIM_SYNC_Port, &GPIO_InitStruct );

    HAL_NVIC_SetPriority( EXTI9_5_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( EXTI9_5_IRQn );
}

void Saturn1_spi::Finite_Ramp_Duration( CommandData_T* cmd )
{
    uint32_t finite_total_duration = ( cmd->cmdDataBuf[ 0 ] ) | ( cmd->cmdDataBuf[ 1 ] << 8 );
    float ramp_duration = ( total_period_mS( ramping::rep_freq ) * ( ramping::duty_cycle ) ) / 100;
    ramping::ramp_off_duration = total_period_mS( ramping::rep_freq ) - ramp_duration + 0.5;
    ramping::periods_in_total
        = ( finite_total_duration * 1000 ) / total_period_mS( ramping::rep_freq );
}

uint32_t get_total_duration( void )
{
    return ( ramping::periods_in_total * ramping::ramp_off_duration );
}

void Saturn1_spi::ramp_setup( void )
{
    float ramp_duration = ( total_period_mS( ramping::rep_freq ) * ( ramping::duty_cycle ) ) / 100;
    ramping::ramp_off_duration = total_period_mS( ramping::rep_freq ) - ramp_duration + 0.5;

    uint8_t num_ramp_steps = ramp_duration / ( 1000.0 / frequency );
    ramping::step_size
        = ( ramping::max_amplitude - ramping::min_amplitude ) / ( num_ramp_steps - 1 );

    HAL_GPIO_WritePin( HF_STIM_SEL_Port, HF_STIM_SEL_Pin, GPIO_PIN_RESET );

    SATURN_WRITE_SINGLE( 0x0007, 0xFF );
    SATURN_WRITE_SINGLE( 0x0008, 0xFF );
    SATURN_WRITE_SINGLE( 0x0009, 0xFF );
    SATURN_WRITE_SINGLE( 0x000A, 0x0F );
    SATURN_WRITE_SINGLE( 0x000B, 0x03 );

    uint8_t last_ph = SATURN_READ_SINGLE( 0x000D );
    SATURN_WRITE_MASK( ( 64 * last_ph ) + 65, 16, 16 );

    HAL_GPIO_WritePin( HF_STIM_CONT_Port, HF_STIM_CONT_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( HF_STIM_STOP_Port, HF_STIM_STOP_Pin, GPIO_PIN_RESET );
}

// Called once per HF_STIM_SYNC rising edge (via thread resume in app_entry.c).
// Steps amplitude from min to max in equal increments over the ramp duration,
// then returns the off-duration so the caller (rampStimThreadEntry) can sleep
// before beginning the next ramp cycle.  Returns 0 while still mid-ramp so
// the caller loops immediately back for the next step.
uint32_t start_ramping( void )
{
    static uint32_t amp = ramping::min_amplitude;

    if ( amp == ramping::min_amplitude )
    {
        Saturn1_spi::ramp_setup();
    }

    // Saturn reg 0x10 accepts a 0–255 scale value; normalize against max amplitude.
    uint16_t curr_amplitude = ( 255 * amp ) / ramping::max_amplitude;

    Saturn1_spi::SATURN_WRITE_SINGLE( 0x10, curr_amplitude );
    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_SET );

    pause_ramp();    // Suspends this thread until the next SYNC edge resumes it

    HAL_GPIO_WritePin( HF_STIM_RUN_Port, HF_STIM_RUN_Pin, GPIO_PIN_RESET );

    amp += ramping::step_size;

    if ( amp > ramping::max_amplitude )
    {
        amp = ramping::min_amplitude;
        return ramping::ramp_off_duration;    // Signal caller to sleep before next ramp
    }

    return 0;
}

void Saturn1_spi::EditChargeLimit( CommandData_T* cmd )
{
    for ( int i = 0; i < num_channels; i++ )
    {
        if ( mapping[ i ] == cmd->cmdDataBuf[ 0 ] )
        {
            chargelimits[ cmd->cmdDataBuf[ 0 ] ]
                = cmd->cmdDataBuf[ 1 ] | ( cmd->cmdDataBuf[ 2 ] << 8 );
        }
    }
    if ( cmd->cmdDataBuf[ 0 ] == 0 )
    {
        memset( chargelimits, 0, 5 * sizeof( uint16_t ) );
    }
}

// Checks all channels for charge-per-phase safety violations after any amplitude
// or pulse-width change.  Charge (nC) = amplitude_µA * PW_µs / 1 000 000, scaled
// here as integer: amplitude * PW / 10000 (units: 0.1 nC).  If any channel
// exceeds its chargelimit, a CT_ChargeLimit unsolicited response is sent listing
// the offending electrode numbers.
void Saturn1_spi::chargecheck( CommandData_T* cmd )
{
    bool error = false;
    static uint8_t outbuff[ num_channels ];

    for ( int i = 0; i < num_channels; i++ )
    {
        if ( ( ( AMPph1[ i ] * PW ) / 10000 > chargelimits[ mapping[ i ] ]
               || ( AMPph2[ i ] * PWphase2 ) / 10000 > chargelimits[ mapping[ i ] ] )
             && chargelimits[ mapping[ i ] ] != 0 )
        {
            outbuff[ i ] = mapping[ i ];
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

// Switches the active sequencer bank set used by all parameter-write functions.
// period=1: fundamental frequency register banks (default).
// period=2: second-harmonic register banks (used in harmonics mode to program
//           a separate frequency on the same ASIC simultaneously).
// Called from CommandHandler before and after writing harmonic parameters to
// ensure both banks are programmed without changing the active stimulation.
void Saturn1_spi::UpdateSequencerAddresses( unsigned int period )
{
    if ( period == 1 || period == 0 )
    {
        seq0a     = 0x80;
        seq0b     = 0xC0;
        seq0delay = 0x100;
        seq1a     = 0x200;
        seq1b     = 0x240;
        seq1delay = 0x280;
    }
    else if ( period == 2 )
    {
        seq0a     = 0x140;
        seq0b     = 0x180;
        seq0delay = 0x1C0;
        seq1a     = 0x2C0;
        seq1b     = 0x300;
        seq1delay = 0x340;
    }
}
