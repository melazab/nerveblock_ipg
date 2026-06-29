/// @file    ext_flash_spi.hpp
/// @brief   This module handles the SPI interface to the External Flash.
/// @details Configures and initializes the SPI peripheral.
/// 			Handles enabling and disabling the GPIO pins for the SPI.
#pragma once
#include "CmdProtocolDefinitions.h"
#include "stm32wbxx_hal.h"


extern unsigned int calvals[1024];

namespace FRAM
{

SPI_HandleTypeDef* get_handle_ptr( void );

void gpio_init( void );

void gpio_deinit( void );

void enable( void );

void disable( void );

void  write_single( unsigned int address, unsigned int data );

void  read_single(unsigned int address );

void ReadCalValues(void);

void write_enable(void);

}

