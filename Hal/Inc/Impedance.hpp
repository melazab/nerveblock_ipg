/// @file    ext_flash_spi.hpp
/// @brief   This module handles the SPI interface to the External Flash.
/// @details Configures and initializes the SPI peripheral.
/// 			Handles enabling and disabling the GPIO pins for the SPI.
#pragma once
#include "CmdProtocolDefinitions.h"
#include "stm32wbxx_hal.h"

extern bool measuringimp;

namespace Impedance
{

void enable( void );

void CloseSwitches( CommandData_T* cmd );

void StartMeasurement(CommandData_T* cmd );

static void impMeasStreamDataTask(void );

void StopMeasurement(void);

void start_up_seq( void );

void pwr_dwn_seq( void );

}

