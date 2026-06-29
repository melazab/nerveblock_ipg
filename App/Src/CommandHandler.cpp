/// @file    CommandHandler.cpp
/// @brief   See the H file for the module description.

#include "CommandHandler.hpp"
#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"
#include "FRAM.hpp"
#include "Impedance.hpp"
#include "Saturn2_spi.hpp"
#include "Saturn_spi.hpp"
#include "SerialInterface.h"
#include "USB.h"
#include "app_entry.h"
// #include "assertLocal.h"
#include "app_ble.h"
#include "battery_charging.h"
#include "gpio_pin_control.hpp"
#include "inductive_link_comm.h"
#include "main.h"
#include "p2p_server_app.h"
#include <stdio.h>


/** @defgroup CommandHandler
 * @{ */

// #region Private_Variables_CommandHandler
/** @defgroup Private_Variables_CommandHandler
 * @{ */
bool harmonics = false;
static CmdHandlerReturnCode_T
nullCmdHandler( CommandData_T* cmd );    // Forward function declaration
static CmdHandlerReturnCode_T cmdHndlLoopback( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_spi_transfer( CommandData_T* cmd );
static CmdHandlerReturnCode_T spi_transfer_saturn2( CommandData_T* cmd );
static CmdHandlerReturnCode_T spi_transfer_saturn( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StartContinuousStim( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_Amplitude( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_PulseWidth( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ElectrodetoChannel( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StartDelay( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ElectrodeEnable( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_Frequency1( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_Frequency2( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_DisableElectrode( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StopContinuousStim( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StimDuration( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ComplianceVoltage( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_OffDuration( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_NCycles( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_WriteCalValues( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_InterphaseDelay( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ReadRegisters( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ImpElectrode( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ImpMeasStart( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_DisableImpElectrode( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RAM1_1( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RAM2_1( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RAM1_2( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RAM2_2( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_WaveShape( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_PerPhasePW( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_PerPhaseAmp( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_ChargeLimit( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_Harmonic( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_DutyCycle( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RepitionFreq( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_MinRampAmp( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_MaxRampAmp( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StartRampStim( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_StopRampStim( CommandData_T* cmd );
static CmdHandlerReturnCode_T cmd_hndl_RampFiniteDur( CommandData_T* cmd );

typedef struct
{
    CmdHandlerReturnCode_T ( *handler )( CommandData_T* );
} CmdHandlerTable_T;

CmdHandlerTable_T cmdHandlerTable[] = {
    { &nullCmdHandler },                  // CT_Invalid              = 0x00 - 00
    { &cmd_hndl_spi_transfer },           // =0x01
    { &gpio_ctrl::write_pin_handler },    // =0x02
    { &gpio_ctrl::read_port_handler },    // =0x03
    { &cmd_hndl_StartContinuousStim },    // =0x04
    { &cmd_hndl_Amplitude },              // =0x05
    { &cmd_hndl_PulseWidth },             // =0x06
    { &cmd_hndl_StartDelay },             // =0x07
    { &cmd_hndl_ElectrodetoChannel },     // =0x08
    { &cmd_hndl_ElectrodeEnable },        // =0x09
    { &cmd_hndl_Frequency1 },             // =0x0A
    { &cmd_hndl_Frequency2 },             // =0x0B
    { &cmd_hndl_DisableElectrode },       // =0x0C
    { &cmd_hndl_StopContinuousStim },     // =0x0D
    { &cmd_hndl_StimDuration },           // 0E
    { &cmd_hndl_ComplianceVoltage },      // 0F
    { &cmd_hndl_OffDuration },            // 10
    { &cmd_hndl_NCycles },                // 11
    { &cmd_hndl_WriteCalValues },         // 12
    { &cmd_hndl_InterphaseDelay },        // 13
    { &cmd_hndl_ReadRegisters },          // 14
    { &cmd_hndl_ImpElectrode },           // 15
    { &cmd_hndl_ImpMeasStart },           // 16
    { &nullCmdHandler },                  // 17
    { &cmd_hndl_DisableImpElectrode },    // 18
    { &cmd_hndl_RAM1_1 },                 // 19
    { &cmd_hndl_RAM2_1 },                 // 1A
    { &cmd_hndl_RAM1_2 },                 // 1B
    { &cmd_hndl_RAM2_2 },                 // 1C
    { &cmd_hndl_PerPhasePW },             // 1D
    { &cmd_hndl_PerPhaseAmp },            // 1E
    { &cmd_hndl_WaveShape },              // 1F
    { &cmd_hndl_ChargeLimit },            // 20
    { &cmd_hndl_Harmonic },               // 21
    { &cmd_hndl_DutyCycle },              // 22
    { &cmd_hndl_RepitionFreq },           // 23
    { &cmd_hndl_MinRampAmp },             // 24
    { &cmd_hndl_MaxRampAmp },             // 25
    { &cmd_hndl_StartRampStim },          // 26
    { &cmd_hndl_StopRampStim },           // 27
	{ &cmd_hndl_RampFiniteDur},			  // 28

    //        { &Saturn1_spi::ElectrodeEnable}		// =0x3D

};

#define CtMaxToken ( sizeof( cmdHandlerTable ) / sizeof( CmdHandlerTable_T ) )

// #endregion

void cmdHndlProcessCmd( CommandData_T* cmd )
{
    static SERIAL_DATA_OUT_BUF_T serOutBuf;

    if ( ( cmd->cmdToken > CT_Invalid )
         && ( cmd->cmdToken < ( sizeof( cmdHandlerTable ) / sizeof( CmdHandlerTable_T ) ) ) )
    {
        // The commands in this range are handled by jumping to the command handler
        // function from the table above. This is a cleaner and more efficient
        // method of handling the commands and all the command handlers will
        // eventually be converted over to use this method.

        CmdHandlerReturnCode_T rStat = cmdHandlerTable[ cmd->cmdToken ].handler( cmd );
            crProtBuildSerialOutBuffer( cmd->cmdToken,
                                        rStat,
                                        cmd->cmdDataLen,
                                        cmd->cmdDataBuf,
                                        &serOutBuf );
    }
    else
    {
        crProtBuildZeroLengthResponse( cmd->cmdToken, CMD_RESP_NACK, &serOutBuf );
    }

    if (APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER)
    {
        Trigger_Send_Notification( &serOutBuf );
    }
    else
    {
        usbSendSerialDataBuf( &serOutBuf );
        // serIntSendSerialDataBuf( &serOutBuf );
    }

}

void SendCmdResponse( CommandToken_T token, ResponseToken_T response, int8_t* rspBuf,
                      uint16_t bufLen )
{
    static SERIAL_DATA_OUT_BUF_T serOutBuf;

    crProtBuildSerialOutBufferSIGNED( token, response, bufLen, rspBuf, &serOutBuf );

    if (APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER)
    {
        Trigger_Send_Notification( &serOutBuf );
    }
    else
    {
        usbSendSerialDataBuf( &serOutBuf );
        // serIntSendSerialDataBuf( &serOutBuf );
    }
}

void SendUCmdResponse( CommandToken_T token, ResponseToken_T response, uint8_t* rspBuf,
                       uint16_t bufLen )
{
    static SERIAL_DATA_OUT_BUF_T serOutBuf;

    crProtBuildSerialOutBuffer( token, response, bufLen, rspBuf, &serOutBuf );

    if (APP_BLE_Get_Server_Connection_Status() == APP_BLE_CONNECTED_SERVER)
    {
        Trigger_Send_Notification( &serOutBuf );
    }
    else
    {
        usbSendSerialDataBuf( &serOutBuf );
        // serIntSendSerialDataBuf( &serOutBuf );
    }
}

void SendZeroLengthResponse( CommandData_T* cmd )
{
    static SERIAL_DATA_OUT_BUF_T serOutBuf;

    crProtBuildZeroLengthResponse( cmd->cmdToken, CMD_RESP_ACK, &serOutBuf );

    usbSendSerialDataBuf( &serOutBuf );
}



CmdHandlerReturnCode_T cmdHndlLoopback( CommandData_T* cmd )
{
    (void)cmd;
    // Just echo back the command packet. It already has the correct
    // info in it.
    return CmdHndlRetCmdPacket;
}

extern "C" void cmdHndlCommandRecv( CommandData_T* cmd )
{
    cmdHndlProcessCmd( cmd );    // TODO do we need a queue
}

CmdHandlerReturnCode_T nullCmdHandler( CommandData_T* cmd )
{
    cmd->cmdDataLen = 0;
    return CMD_RESP_NACK;
}


CmdHandlerReturnCode_T cmd_hndl_spi_transfer( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == 1 )
    {
        // first byte is the channel, 4 = External Flash SPI
        return spi_transfer_saturn( cmd );
    }

    if ( cmd->cmdDataBuf[ 0 ] == 2 )
    {
        // first byte is the channel, 4 = External Flash SPI
        return spi_transfer_saturn2( cmd );
    }
    {
        // All other channels are currently unsupported
        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T spi_transfer_saturn( CommandData_T* cmd )
{
    uint8_t write = cmd->cmdDataBuf[ 1 ];

    if ( write == 1 )
    {
        Saturn1_spi::SATURN_WRITE_SINGLE( ( (uint16_t)cmd->cmdDataBuf[ 2 ] )
                                              | ( cmd->cmdDataBuf[ 3 ] << 8 ),
                                          cmd->cmdDataBuf[ 4 ] );
    }
    else
    {
        Saturn1_spi::ReadReturn( cmd );
    }

    return CmdHndlRetCmdPacket;
}

CmdHandlerReturnCode_T spi_transfer_saturn2( CommandData_T* cmd )
{
    uint8_t write = cmd->cmdDataBuf[ 1 ];

    if ( write == 1 )
    {
        Saturn2_spi::SATURN_WRITE_SINGLE( ( (uint16_t)cmd->cmdDataBuf[ 2 ] )
                                              | ( cmd->cmdDataBuf[ 3 ] << 8 ),
                                          cmd->cmdDataBuf[ 4 ] );
    }
    else
    {
        // Saturn2_spi::ReadReturn( cmd );
    }

    return CmdHndlRetCmdPacket;
}

CmdHandlerReturnCode_T cmd_hndl_ElectrodetoChannel( CommandData_T* cmd )
{
    Saturn1_spi::ElectrodetoChannel( cmd );
    Saturn1_spi::UpdateSequencerAddresses( 2 );
    Saturn1_spi::ElectrodetoChannel( cmd );
    Saturn1_spi::UpdateSequencerAddresses( 1 );

    Saturn2_spi::ElectrodetoChannel( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_StartContinuousStim( CommandData_T* cmd )
{
    Saturn1_spi::Starttick();
    Saturn2_spi::Starttick();

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_Amplitude( CommandData_T* cmd )
{
    if ( ( ( (uint16_t)cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) ) > 0x3a98 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetAmpOutRange;
    }
    else
    {
        Saturn1_spi::Amplitude( cmd );
        Saturn2_spi::Amplitude( cmd );
        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_PulseWidth( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == 0x01 )
    {
        return Saturn1_spi::PulseWidth( cmd );
    }
    else
    {
        return Saturn2_spi::PulseWidth( cmd );
    }
}

CmdHandlerReturnCode_T cmd_hndl_StartDelay( CommandData_T* cmd )
{
    if ( ( cmd->cmdDataLen ) > 3 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetDelayOutRange;
    }
    else
    {
        Saturn1_spi::StartDelay( cmd );
        Saturn2_spi::StartDelay( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_ElectrodeEnable( CommandData_T* cmd )
{
    Saturn1_spi::ElectrodeEnable( cmd );
    Saturn2_spi::ElectrodeEnable( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_DisableElectrode( CommandData_T* cmd )
{
    Saturn1_spi::DisableElectrode( cmd );
    Saturn2_spi::DisableElectrode( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_Frequency1( CommandData_T* cmd )
{
    if ( !harmonics )
    {
        Saturn1_spi::ElectrodeEnable( cmd );
        Saturn2_spi::DisableElectrode( cmd );
        return Saturn1_spi::Frequency( cmd );
    }
    else
    {
        uint8_t returnval = Saturn1_spi::Frequency( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 2 );
        Saturn1_spi::Frequency( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 1 );
        return returnval;
    }
}

CmdHandlerReturnCode_T cmd_hndl_Frequency2( CommandData_T* cmd )
{
    Saturn2_spi::ElectrodeEnable( cmd );
    Saturn1_spi::DisableElectrode( cmd );
    return Saturn2_spi::Frequency( cmd );
}

CmdHandlerReturnCode_T cmd_hndl_StopContinuousStim( CommandData_T* cmd )
{
    Saturn1_spi::StopContinuousStim();
    Saturn2_spi::StopContinuousStim();

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_StimDuration( CommandData_T* cmd )
{
    if ( ( cmd->cmdDataLen ) > 3 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetDurOutRange;
    }
    else
    {
        Saturn1_spi::StimDuration( cmd );
        Saturn2_spi::StimDuration( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_ComplianceVoltage( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] > 0x55 )
    {    // 0x55 = 16.363 V
        cmd->cmdDataLen = 0;
        return CmdHndlRetHVTooHigh;
    }
    else
    {
        Saturn1_spi::SATURN_WRITE_SINGLE( 0x0020, cmd->cmdDataBuf[ 0 ] );
        Saturn2_spi::SATURN_WRITE_SINGLE( 0x0020, cmd->cmdDataBuf[ 0 ] );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_OffDuration( CommandData_T* cmd )
{
    if ( ( cmd->cmdDataLen ) > 3 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetOffDur;
    }
    else
    {
        Saturn1_spi::OffDuration( cmd );
        Saturn2_spi::OffDuration( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_NCycles( CommandData_T* cmd )
{
    if ( ( cmd->cmdDataLen ) > 2 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetNCycles;
    }
    else
    {
        Saturn1_spi::NCycles( cmd );
        Saturn2_spi::NCycles( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_WriteCalValues( CommandData_T* cmd )
{
    Saturn2_spi::gpio_deinit();
    FRAM::gpio_init();

    FRAM::write_enable();
    FRAM::write_single( ( cmd->cmdDataBuf[ 0 ] << 8 | cmd->cmdDataBuf[ 1 ] ),
                        cmd->cmdDataBuf[ 2 ] );

    FRAM::gpio_deinit();
    Saturn2_spi::gpio_init();

    Saturn1_spi::Calibration();
    Saturn2_spi::Calibration();

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_InterphaseDelay( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == 0x01 )
    {
        if ( harmonics )
        {
            uint8_t returnval = Saturn1_spi::InterphaseDelay( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 2 );
            Saturn1_spi::InterphaseDelay( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 1 );
            return returnval;
        }
        else
        {
            return Saturn1_spi::InterphaseDelay( cmd );
        }
    }
    else
    {
        return Saturn2_spi::InterphaseDelay( cmd );
    }
}

CmdHandlerReturnCode_T cmd_hndl_ReadRegisters( CommandData_T* cmd )
{
    Saturn2_spi::read_registers( cmd );

    return CmdHndlRetCmdPacket;
}

CmdHandlerReturnCode_T cmd_hndl_ImpElectrode( CommandData_T* cmd )
{
    Impedance::CloseSwitches( cmd );
    Saturn2_spi::ElectrodeEnable( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_ImpMeasStart( CommandData_T* cmd )
{
    Impedance::StartMeasurement( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_DisableImpElectrode( CommandData_T* cmd )
{
    // Impedance::OpenSwitches( cmd );
    Saturn2_spi::DisableElectrode( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_RAM1_1( CommandData_T* cmd )
{
    Saturn1_spi::RAM1( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_RAM2_1( CommandData_T* cmd )
{
    Saturn1_spi::RAM2( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_RAM1_2( CommandData_T* cmd )
{
    Saturn2_spi::RAM1( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_RAM2_2( CommandData_T* cmd )
{
    Saturn2_spi::RAM2( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_WaveShape( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == 0x01 )
    {
        if ( harmonics )
        {
            Saturn1_spi::SelectShape( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 2 );
            Saturn1_spi::SelectShape( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 1 );
        }
        else
        {
            Saturn1_spi::SelectShape( cmd );
        }
    }
    else
    {
        Saturn2_spi::SelectShape( cmd );
    }
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_PerPhasePW( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 0 ] == 0x01 )
    {
        if ( harmonics )
        {
            uint8_t returnval = Saturn1_spi::PerPhasePW( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 2 );
            Saturn1_spi::PerPhasePW( cmd );
            Saturn1_spi::UpdateSequencerAddresses( 1 );
            return returnval;
        }
        else
        {
            return Saturn1_spi::PerPhasePW( cmd );
        }
    }

    else
    {
        return Saturn2_spi::PerPhasePW( cmd );
    }
}

CmdHandlerReturnCode_T cmd_hndl_PerPhaseAmp( CommandData_T* cmd )
{
    if ( ( ( (uint16_t)cmd->cmdDataBuf[ 2 ] ) | ( cmd->cmdDataBuf[ 3 ] << 8 ) ) > 0x3a98 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetAmpOutRange;
    }
    else if ( harmonics )
    {
        Saturn1_spi::PerPhaseAmp( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 2 );
        Saturn1_spi::PerPhaseAmp( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 1 );
        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
    else
    {
        Saturn1_spi::PerPhaseAmp( cmd );
        Saturn2_spi::PerPhaseAmp( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_ChargeLimit( CommandData_T* cmd )
{
    Saturn1_spi::EditChargeLimit( cmd );
    Saturn2_spi::EditChargeLimit( cmd );

    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_Harmonic( CommandData_T* cmd )
{
    if ( cmd->cmdDataBuf[ 1 ] == 0 )
    {    // Turn off harmonics mode
        harmonics = false;
    }
    else if ( cmd->cmdDataBuf[ 1 ] == 1 )    // Fundamental frequency i.e. lower frequency
    {
        harmonics = true;
        Saturn1_spi::ElectrodeEnable( cmd );
        Saturn2_spi::DisableElectrode( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 2 );
        Saturn1_spi::DisableElectrode( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 1 );
    }
    else if ( cmd->cmdDataBuf[ 1 ] == 2 )
    {    // 2nd harmonic i.e. higher frequency

        harmonics = true;
        Saturn1_spi::ElectrodeEnable( cmd );
        Saturn2_spi::DisableElectrode( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 2 );
        Saturn1_spi::ElectrodeEnable( cmd );
        Saturn1_spi::UpdateSequencerAddresses( 1 );
    }
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_DutyCycle( CommandData_T* cmd )
{
    Saturn1_spi::Duty_Cycle( cmd );
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_RepitionFreq( CommandData_T* cmd )
{
    Saturn1_spi::Rep_Freq( cmd );
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_MinRampAmp( CommandData_T* cmd )
{
    if ( ( ( (uint16_t)cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) ) > 0x3a98 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetAmpOutRange;
    }
    else
    {
        Saturn1_spi::Min_Amplitude_Ramp( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_MaxRampAmp( CommandData_T* cmd )
{
    if ( ( ( (uint16_t)cmd->cmdDataBuf[ 1 ] ) | ( cmd->cmdDataBuf[ 2 ] << 8 ) ) > 0x3a98 )
    {
        cmd->cmdDataLen = 0;
        return CmdHndlRetAmpOutRange;
    }
    else
    {
        Saturn1_spi::Max_Amplitude_Ramp( cmd );

        cmd->cmdDataLen = 0;
        return CmdHndlRetZeroAck;
    }
}

CmdHandlerReturnCode_T cmd_hndl_RampFiniteDur( CommandData_T* cmd )
{
    Saturn1_spi::Finite_Ramp_Duration( cmd );
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_StartRampStim( CommandData_T* cmd )
{
    start_stim_control();
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

CmdHandlerReturnCode_T cmd_hndl_StopRampStim( CommandData_T* cmd )
{
    stop_stim_control();
    cmd->cmdDataLen = 0;
    return CmdHndlRetZeroAck;
}

/** @} */    // end group CommandHandler
