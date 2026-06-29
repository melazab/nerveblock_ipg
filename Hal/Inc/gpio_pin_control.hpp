/// @file    gpio_pin_control.hpp
/// @brief   Handles the control of individual GPIO pins for the command handler.
/// @details The command interface has a command that allows the GPIO pins to
///          be Set and Reset via command. This feature is used to support
///          development, testing, and debug. It was requested by the FPGA
///          developer as a tool to aide his development.
///          Some of this modules features:
///           - It will allow only GPIO pins that are used as I/O to be Set/Reset.
///           - It will keep track of the current state of the GPIO pins under
///             its control.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "CmdProtocolDefinitions.h"

namespace gpio_ctrl
{

typedef enum    // The command tokens
{
    GPIO_CTRL_PORTA     = 0x00,
    GPIO_CTRL_PORTB     = 0x01,
    GPIO_CTRL_PORTC     = 0x02,
    GPIO_CTRL_PORTD     = 0x03,
    GPIO_CTRL_PORTE     = 0x04,
    GPIO_CTRL_PORTF     = 0x05,
    GPIO_CTRL_PORTG     = 0x06,
    GPIO_CTRL_PORTH     = 0x07,
    GPIO_CTRL_PORTI     = 0x08,
    GPIO_CTRL_PORTI_Max = 0x09
} GPIO_CTRL_PORT_T;

/// @brief   Sets the given GPIO pin high or low.
/// @details SPI Control - Write one byte to radio SPI
///              cmd[0], bits 7-4 : GPIO Port, 0=GPIOA, 1=GPIOB, etc
///              cmd[0], bits 3-0 : GPIO Pin, 0=Pin0, 1=Pin1, etc
///              cmd[1] : 0 = Set low, 1 = Set high
/// @param   CommandData_T *cmd: A pointer to a command packet.
/// @returns CmdHandlerReturnCode_T: CMD_RESP_ACK, CMD_RESP_NACK
CmdHandlerReturnCode_T write_pin_handler( CommandData_T* cmd );

/// @brief   Reads the IDR register of the given GPIO port.
/// @details SPI Control - Write one byte to radio SPI
///              cmd[0], bits 7-4 : GPIO Port, 0=GPIOA, 1=GPIOB, etc
///          returns the GPIO port status for all 16 pins. (reads IDR)
///              respData[0] : Port pins  7-0.
///              respData[1] : Port pins 15-8.
/// @param   CommandData_T *cmd: A pointer to a command packet.
/// @returns CmdHandlerReturnCode_T: CMD_RESP_ACK, CMD_RESP_NACK
CmdHandlerReturnCode_T read_port_handler( CommandData_T* cmd );

}    // namespace gpio_ctrl
