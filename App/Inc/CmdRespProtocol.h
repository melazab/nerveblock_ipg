/*****************************************************************************/
/*!
 * \file    CmdRespProtocol.h
 *
 * \brief   Parses the incoming command stream. Formats packets for 
 *          transmission.
 *
 * \details Has a parser / state machine to decode the incoming command 
 *          packets. Once a packet is decoded and validated it is passed
 *          on to the Command Handler. Has seperate but related functionality
 *          to encode data frames with the appropriate protocol wrapper.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    May 8, 2023
 *
 *****************************************************************************/
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "tx_api.h"

#include "CmdProtocolDefinitions.h"

#ifdef __cplusplus
extern "C"
{
#endif


void rampStimThreadEntry( uint32_t treadInput );

/*****************************************************************************/
/*!
 * \brief   Reset the Command Parser to its initial state.
 *
 * \returns void
 *
 *****************************************************************************/
void crProtReset(void);

/*****************************************************************************/
/*!
 * \brief   Recieves one byte and queues it up to parse.
 *
 * \param   uint8_t cmdByte - One byte of the input stream.
 *
 * \returns nothing
 *
 *****************************************************************************/
void crProtQueueByte(uint8_t cmdByte);

/*****************************************************************************/
/*!
 * \brief   Sets the pointer to the queue to use for the command parser
 *
 * \details The queues and the memory for the queues are controlled
 *          by the app_threadx module. This function provides a way for
 *          a pointer to the externally created queue to be passed into
 *          the module
 *
 * \param   TX_QUEUE *queuePtr - A pointer to the command parser queue.
 *
 * \returns nothing
 *
 *****************************************************************************/
void crProtSetCmdParserQueue(TX_QUEUE *queuePtr);

/*****************************************************************************/
/*!
 * \brief   Thread that parses incoming serial data.
 * 
 * \details Waits on the incoming data byte queue and passes the bytes on to
 *          the parser function.
 * 
 * \param   ULONG treadInput
 * 
 * \retval None
 *
*****************************************************************************/
void crProtParserThreadEntry(uint32_t treadInput);

/*****************************************************************************/
/*!
 * \brief   Receives one character of a potential command to parse.
 *
 * \param   uint8_t cmdChar - One character in a potential command byte stream.
 *
 * \returns bool -  true - If the character provided matches the current state
 *                         and causes the state machine to move to the next
 *                         state.
 *                  false - The character provided was not correct for the
 *                          current state.
 *
 *****************************************************************************/
bool crProtNewInCmdChar(uint8_t cmdChar);

/*****************************************************************************/
/*!
 * \brief   Is given a set of parameters (command token, response token,
 *          response data buffer, etc) and builds the complete response
 *          data into the provided buffer.
 *
 * \param  CommandToken_T cmdToken  - The command token that should be used in
 *                                    the serial data output.
 * \param  ResponseToken_T rspToken - The response token to use
 * \param  uint16_t bufLen           - The length of the data in the  buffer.
 * \param  uint8_t *rspBuf          - A buffer containing the command or
 *                                    response data.
 * \param  SERIAL_DATA_OUT_BUF_T *serOutBuf - A pointer to the serial data
 *                                    output buffer structure to build the
 *                                    serial data output in.
 *
 *****************************************************************************/
void crProtBuildSerialOutBuffer( CommandToken_T cmdToken, ResponseToken_T rspToken, uint16_t bufLen,
                                 uint8_t* rspBuf, SERIAL_DATA_OUT_BUF_T* serOutBuf );

void crProtBuildSerialOutBufferSIGNED( CommandToken_T cmdToken, ResponseToken_T rspToken, uint16_t bufLen,
                                 int8_t* rspBuf, SERIAL_DATA_OUT_BUF_T* serOutBuf );

/*****************************************************************************/
/*!
 * \brief   Sends a either an ACK or a NACK response with no response data.
 *
 * \param   CommandToken_T token - The command token that should be used in
 *                                 the packet.
 * \param   ResponseToken_T response - The response token that should be used in
 *                                 the packet.
 * \param  SERIAL_DATA_OUT_BUF_T *serOutBuf - A pointer to the serial data
 *                                    output buffer structure to build the
 *                                    serial data output in.
 *
 * \returns void
 *
 *****************************************************************************/

void crProtBuildZeroLengthResponse( CommandToken_T token, ResponseToken_T response,
                                    SERIAL_DATA_OUT_BUF_T* serOutBuf );

void build_status_command( uint8_t* rspBuf, SERIAL_DATA_OUT_BUF_T* serOutBuf );

#ifdef __cplusplus
}    // extern "C"
#endif
