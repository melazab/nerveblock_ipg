/*****************************************************************************/
/*!
 * \file    CmdRespProtocol.c
 *
 * \brief   See the H file for the module description.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    Feb 11, 2023
 *
 *****************************************************************************/

//#include "assertLocal.h"
#include "stm32wbxx.h"
#include "tx_api.h"
#include <assert.h>
#include <string.h>    // for memset

#include "CmdRespProtocol.h"
#include "CommandHandler.hpp"
#include "Crc16.h"
#include "app_entry.h"

// #include "Saturn_spi.hpp"

/** @defgroup CmdRespProtocol
 * @{ */

/** @defgroup Module_Vars_CmdRespProtocol
 * @{ */

typedef enum    // These states are named based on the byte  in the protocol
                // they are currently searching for.
{
    SyncByteOne,
    SyncByteTwo,
    CmdToken,
    RespCode,
    CmdDataLength,
    CmdData,
    CrcByteOne,
    CrcByteTwo
} CmdParserState_T;

static const uint8_t SyncByteOneValue   = 0xFC;
static const uint8_t SyncByteTwoValue   = 0x1A;
static const uint8_t CmdPckResponseCode = 0x00;

static CmdParserState_T curParserState = SyncByteOne;
static CommandData_T curCommand        = { CT_SpiTransfer, 0, { 0 } };

TX_QUEUE* inByteQueue = NULL;


/** @} */    // end group Module_Vars_CmdRespProtocol

/*****************************************************************************/
void crProtSetCmdParserQueue( TX_QUEUE* queuePtr )
/*****************************************************************************/
{
    inByteQueue = queuePtr;
}

/*****************************************************************************/
bool crProtNewInCmdChar( uint8_t cmdChar )
/*****************************************************************************/
{
    static uint16_t curDataByte        = 0;
    static uint16_t extendedCmdDataLen = 0;
    static uint16_t crcWord            = 0;

    bool charCorrect = false;
    static uint8_t crcDt[ CmdDataMaxSize + 3 ];    // keep this variable off the stack
    static uint16_t crcDataCount = 0;              // static because this variable needs
                                                   // to remember its value between calls
    switch ( curParserState )
    {
        case SyncByteOne:
            if ( SyncByteOneValue == cmdChar )
            {
                curParserState = SyncByteTwo;
                charCorrect    = true;
            }
            break;
        case SyncByteTwo:
            // We will always have to pass through here it we are
            // getting ready to build a new command. Initialize
            // the command structure here. Not in SyncByteOne
            // because we might go throug that state multiple times
            // if we are searching for the start of a new command.
            curCommand.cmdToken   = CT_Invalid;
            curCommand.cmdDataLen = 0x00;
            if ( SyncByteTwoValue == cmdChar )
            {
                curParserState = CmdToken;
                crcDataCount   = 0;
                charCorrect    = true;
            }
            else
            {
                curParserState = SyncByteOne;
            }
            break;
        case CmdToken:
            // Any command token will pass through the parser.
            // Invalid tokens will be NACK'd by the Command Handler
            curCommand.cmdToken     = (CommandToken_T)cmdChar;
            crcDt[ crcDataCount++ ] = cmdChar;
            curParserState          = RespCode;
            charCorrect             = true;
            break;
        case RespCode:
            // mask off 2 MSB since they are now being used as bits 8 and 9
            // of the extended Command Data Length. Mask off the next 2 bits
            // as well. Only the lower nibble is valid for the Command
            // Response Code
            if ( ( 0x0f & cmdChar ) == CmdPckResponseCode )
            {
                extendedCmdDataLen      = 0xc0 & cmdChar;
                curParserState          = CmdDataLength;
                crcDt[ crcDataCount++ ] = cmdChar;
                charCorrect             = true;
            }
            else
            {
                curParserState = SyncByteOne;
            }
            break;
        case CmdDataLength:
            curCommand.cmdDataLen   = ( extendedCmdDataLen << 2 ) | cmdChar;
            crcDt[ crcDataCount++ ] = cmdChar;
            curDataByte             = 0;    // get ready to collect the data bytes
            curParserState          = ( 0 == curCommand.cmdDataLen ) ? CrcByteOne : CmdData;
            memset( curCommand.cmdDataBuf, 0, sizeof( curCommand.cmdDataBuf ) );
            charCorrect = true;
            break;
        case CmdData:
            // Only put the data byte in the buffer if it will fit
            if ( curDataByte < CmdDataMaxBufferSize )
            {
                curCommand.cmdDataBuf[ curDataByte ] = cmdChar;
            }
            curDataByte++;
            crcDt[ crcDataCount++ ] = cmdChar;
            if ( curDataByte >= curCommand.cmdDataLen )
            {
                curParserState = CrcByteOne;
                charCorrect    = true;
                // clear crcWord in prep for Crc Byte states
            }
            break;
        case CrcByteOne:
            crcWord        = (uint16_t)cmdChar;
            curParserState = CrcByteTwo;
            charCorrect    = true;
            break;
        case CrcByteTwo:
        {
            crcWord += (uint16_t)cmdChar << 8;

            // TODO check crc
            uint16_t crcCalc = crc16GetCrc( crcDt, crcDataCount );
            (void)crcCalc;
            if ( true )    // TODO CRC disabled for now - was: (crcCalc == crcWord)
            // if (crcCalc == crcWord)
            {
                // assert(false); // TODO debugging
                cmdHndlCommandRecv( &curCommand );
                charCorrect = true;
            }
            curParserState = SyncByteOne;
        }
        break;
        default:
            curParserState = SyncByteOne;
            break;
    }

    return charCorrect;
}

void build_status_command( uint8_t* rspBuf, SERIAL_DATA_OUT_BUF_T* serOutBuf )
{
    uint16_t dtIndex = 0;

    uint8_t tmpCalc                    = (uint8_t)( 0x94 | 0x01 );
    serOutBuf->outDataBuf[ dtIndex++ ] = tmpCalc;
    serOutBuf->outDataBuf[ dtIndex++ ] = 0x03;
    for ( int i = 0; i < 2; i++ )
    {
        serOutBuf->outDataBuf[ dtIndex++ ] = rspBuf[ i ];
    }
    serOutBuf->bufDataLen = dtIndex;
}

/*****************************************************************************/
void crProtBuildSerialOutBuffer( CommandToken_T cmdToken, ResponseToken_T rspToken, uint16_t bufLen,
                                 uint8_t* rspBuf, SERIAL_DATA_OUT_BUF_T* serOutBuf )
/*****************************************************************************/
{
    uint16_t dtIndex = 0;

    serOutBuf->outDataBuf[ dtIndex++ ] = SyncByteOneValue;
    serOutBuf->outDataBuf[ dtIndex++ ] = SyncByteTwoValue;
    serOutBuf->outDataBuf[ dtIndex++ ] = cmdToken;
    uint8_t tmpCalc = (uint8_t)( ( ( bufLen & 0x0300 ) >> 2 ) | ( rspToken & 0x0f ) );
    serOutBuf->outDataBuf[ dtIndex++ ] = tmpCalc;
    // serOutBuf->outDataBuf[dtIndex++] = (uint8_t)(((bufLen & 0x0300) >> 2) | (rspToken & 0x0f));
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( bufLen & 0x00ff );
    for ( int i = 0; i < bufLen; i++ )
    {
        serOutBuf->outDataBuf[ dtIndex++ ] = rspBuf[ i ];
    }
    uint16_t crcCalc                   = 0x0000;
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( 0x00ff & crcCalc );
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( 0x00ff & ( crcCalc >> 8 ) );
    serOutBuf->bufDataLen              = dtIndex;
}
void crProtBuildSerialOutBufferSIGNED( CommandToken_T cmdToken, ResponseToken_T rspToken,
                                       uint16_t bufLen, int8_t* rspBuf,
                                       SERIAL_DATA_OUT_BUF_T* serOutBuf )
/*****************************************************************************/
{
    uint16_t dtIndex = 0;

    serOutBuf->outDataBuf[ dtIndex++ ] = SyncByteOneValue;
    serOutBuf->outDataBuf[ dtIndex++ ] = SyncByteTwoValue;
    serOutBuf->outDataBuf[ dtIndex++ ] = cmdToken;
    uint8_t tmpCalc = (uint8_t)( ( ( bufLen & 0x0300 ) >> 2 ) | ( rspToken & 0x0f ) );
    serOutBuf->outDataBuf[ dtIndex++ ] = tmpCalc;
    // serOutBuf->outDataBuf[dtIndex++] = (uint8_t)(((bufLen & 0x0300) >> 2) | (rspToken & 0x0f));
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( bufLen & 0x00ff );
    for ( int i = 0; i < bufLen; i++ )
    {
        serOutBuf->outDataBuf[ dtIndex++ ] = rspBuf[ i ];
    }
    uint16_t crcCalc                   = 0x0000;
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( 0x00ff & crcCalc );
    serOutBuf->outDataBuf[ dtIndex++ ] = (uint8_t)( 0x00ff & ( crcCalc >> 8 ) );
    serOutBuf->bufDataLen              = dtIndex;
}

/*****************************************************************************/
void crProtBuildZeroLengthResponse( CommandToken_T token, ResponseToken_T response,
                                    SERIAL_DATA_OUT_BUF_T* serOutBuf )
/*****************************************************************************/
{
    crProtBuildSerialOutBuffer( token, response, 0, NULL, serOutBuf );

    return;
}


/*****************************************************************************/
void crProtReset( void )
/*****************************************************************************/
{
    curParserState = SyncByteOne;
}

/*****************************************************************************/
void crProtQueueByte( uint8_t cmdByte )
/*****************************************************************************/
{
    UINT status;

    assert( inByteQueue != NULL );

    status = tx_queue_send( inByteQueue, &cmdByte, TX_NO_WAIT );

    /* Check completion status. */
    assert( status == TX_SUCCESS );
}

/*****************************************************************************/
void crProtParserThreadEntry( uint32_t treadInput )
/*****************************************************************************/
{
    uint32_t rcvByte;
    UINT status;

    assert( inByteQueue != NULL );

    for ( ;; )
    {
        // wait on queue
        status = tx_queue_receive( inByteQueue, &rcvByte, TX_WAIT_FOREVER );
        if ( status == TX_SUCCESS )
        {
            crProtNewInCmdChar( rcvByte );
            // debugCharBuf[debugBufP++] = rcvByte;
            // HAL_UART_Transmit(&huart6, &rcvByte, 1, 250);    // TMP serial echo
        }
    }

   // return;
}

// ThreadX entry for the ramp stimulation thread (created by start_stim_control()).
// Each iteration calls start_ramping(), which steps through one amplitude level
// and suspends itself on pause_ramp() until the next HF_STIM_SYNC edge.
// When start_ramping() returns non-zero it signals end of one ramp cycle;
// the thread sleeps for that off-duration before the next cycle begins.
// Exits when stop_stim_thread flag is set or the finite duration is reached,
// then calls stop_stim_control() so Thread Manager cleans up this thread.
void rampStimThreadEntry( uint32_t treadInput )
{
	uint32_t max_duration_ticks = get_total_duration();
	uint32_t elapsed_ticks = 0;
	bool continuous = (max_duration_ticks == 0);

	while ( !( get_ramp_stim_flag() ) && (continuous || elapsed_ticks < max_duration_ticks))
	{
		uint32_t off_dur = start_ramping();
		tx_thread_sleep( off_dur );
		elapsed_ticks += off_dur;
	}

	if(elapsed_ticks >= max_duration_ticks)
	{
		stop_stim_control();
	}
}

/** @}   */    // end group CmdRespProtocol
/*************** END OF FUNCTIONS
 * ***************************************************************************/
