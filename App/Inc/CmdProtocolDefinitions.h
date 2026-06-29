/*****************************************************************************/
/*!
 * \file    CmdProtocolDefinitions.h
 *
 * \brief   Defines all constants enums and structures related to the
 *          Command and Response protocol.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    Feb 11, 2023
 *
 *****************************************************************************/
#ifndef CMDPROTODEFS_H_
#define CMDPROTODEFS_H_

#include <stdint.h>

// The current command protocol allows for Command Data and Response Data
// lengths of up to 1023 bytes. The largest data length actually used by a
// command or response is 259. I don't want to create a buffer to hold the
// largest possible data lenght, just the largest used. So the command structure
// buffer is only 262.
#define CmdRespPacketHeaderSize 7
#define CmdRespPacketMaxSize    ( 130 + CmdRespPacketHeaderSize )
#define CmdDataMaxSize          130
#define CmdDataMaxBufferSize    130

// Command tokens used in the serial packet header.  Only a subset of values are
// named here; the full dispatch table in CommandHandler.cpp maps tokens 0x00-0x28
// to handler functions by index — keep this enum in sync with that table.
typedef enum
{
  CT_Invalid       = 0x00,
  CT_SpiTransfer   = 0x01,
  CT_ReadRegisters = 0x14,
  CT_ImpMeasStart  = 0x16,
  CT_ImpedanceData = 0x17,    // Unsolicited response token for impedance data stream
  CT_ChargeLimit   = 0x20,    // Unsolicited response token for charge-limit violation
} CommandToken_T;

typedef struct
{
    CommandToken_T cmdToken;
    uint16_t cmdDataLen;
    uint8_t cmdDataBuf[ CmdDataMaxBufferSize ];
} CommandData_T;

typedef struct
{
    uint16_t bufDataLen;
    uint8_t outDataBuf[ CmdRespPacketMaxSize ];
} SERIAL_DATA_OUT_BUF_T;

// Command Handler functions will no longer return a special code that indicates to the
// command process what to do with the results. Instead they will return the response token
// to send back. The typedef and defines below help with this transistion.
// The transistion is necessary because the new commands to support the RFID and FPGA often
// return a special response code that is outside of the standard ACK / NACK response codes.
typedef enum    // The response tokens
{
    CMD_RESP_ACK  = 0x00,
    CMD_RESP_NACK = 0x0F,
	CMD_RESP_E00  = 0x01,
	CMD_RESP_E01  = 0x02,
	CMD_RESP_E02  = 0x03,
	CMD_RESP_E03  = 0x04,
	CMD_RESP_E04  = 0x05,
	CMD_RESP_E05  = 0x06,
	CMD_RESP_E06  = 0x07,
	CMD_RESP_E07  = 0x08,
	CMD_RESP_E08  = 0x09,
	CMD_RESP_E09  = 0x0A,
	CMD_RESP_E0A  = 0x0B,

} StandardResponseToken_T;

typedef uint8_t ResponseToken_T;
typedef uint8_t CmdHandlerReturnCode_T;
#define CmdHndlRetZeroAck   	CMD_RESP_ACK
#define CmdHndlRetZeroNAck  	CMD_RESP_NACK
#define CmdHndlRetCmdPacket 	CMD_RESP_ACK
#define CmdHndlRetAmpOutRange	CMD_RESP_E00
#define CmdHndlRetFreqTooSmall	CMD_RESP_E01
#define CmdHndlRetFreqZero		CMD_RESP_E02
#define CmdHndlRetHVTooHigh		CMD_RESP_E03
#define CmdHndlRetPWOutRange	CMD_RESP_E04
#define CmdHndlRetDelayOutRange	CMD_RESP_E05
#define CmdHndlRetDurOutRange	CMD_RESP_E06
#define CmdHndlRetOffDur		CMD_RESP_E07
#define CmdHndlRetNCycles		CMD_RESP_E08
#define CmdHndlIDOutRange		CMD_RESP_E09
#define CmdHndlChargeLimit		CMD_RESP_E0A

#endif /* CMDPROTODEFS_H_ */
