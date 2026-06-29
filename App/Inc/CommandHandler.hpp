/// @file    CommandHandler.hpp
/// @brief   Recieves commands and routes them to the appropriate handler functions.


#pragma once

#include <stdbool.h>
#include "CmdProtocolDefinitions.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern bool harmonics;

/*****************************************************************************/
/*!
 * \brief   Proccesses the incoming command packets.
 *
 * \details Switches on the command token and takes the appropiate action.
 *          Sends the appropiate responses back to the serial interface.
 *          This function is private to the module but it is put here to
 *          facilitate testing.
 *
 * \param   CommandData_T cmd - A pointer to a structure containing the
 *                              command to handle.
 *
 * \returns void
 *
 *****************************************************************************/
void cmdHndlProcessCmd( CommandData_T* cmd );

/*****************************************************************************/
/*!
 * \brief   Receives a pointer to a Command Data Packet and handles or routes
 *          it to the correct module.
 *
 * \details The command will be placed into a message queue so that a seperate
 *          thread can handle the proccessing of the commands.
 *
 * \param   CommandData_T cmd - A pointer to a structure containing the
 *                              command to handle.
 *
 * \returns void
 *
 *****************************************************************************/
void SendCmdResponse( CommandToken_T token, ResponseToken_T response, int8_t *rspBuf, uint16_t bufLen );

void SendUCmdResponse( CommandToken_T token, ResponseToken_T response, uint8_t *rspBuf, uint16_t bufLen );

void SendZeroLengthResponse( CommandData_T* cmd );

void cmdHndlCommandRecv( CommandData_T* cmd );

uint32_t start_ramping ( void );

uint32_t get_total_duration (void);

/*****************************************************************************/
/*!
 * \brief   Initialize RTOS tasks and semaphores.
 *
 * \returns void
 *
 *****************************************************************************/
void cmdHndlModuleInit( void );

#ifdef __cplusplus
}    // extern "C"
#endif
