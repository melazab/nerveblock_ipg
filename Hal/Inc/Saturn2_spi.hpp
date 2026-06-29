
#pragma once

#include "stm32wbxx_hal.h"

#include "CmdProtocolDefinitions.h"
#include "dio_stm32.hpp"

extern uint32_t duration2;
extern unsigned int ARG7_2;

// #ifdef __cplusplus
// extern "C"
// {
// #endif

namespace Saturn2_spi
{


SPI_HandleTypeDef* get_handle_ptr( void );
void enable( void );
void peripheral_init( void );
void gpio_init( void );
void gpio_deinit( void );
void STIM_POWER_UP (void);
void PULSE_STIM_CLK (unsigned int pulses);
void PULSE_STIM_SCK (unsigned int pulses);
void SATURN_WRITE_SINGLE (unsigned int address, unsigned int data);
void SATURN_READ_SINGLE (unsigned int address);
void SATURN_WRITE_MASK (unsigned int address, unsigned int data, unsigned int mask);
void SATURN_DIG_STARTUP(void);
void SATURN_BBC_STARTUP_R1 (void);
void default_chunk_write (unsigned int start_address);
void read_registers( CommandData_T* cmd );
CmdHandlerReturnCode_T PulseWidth( CommandData_T* cmd );
CmdHandlerReturnCode_T Frequency( CommandData_T* cmd );
CmdHandlerReturnCode_T update_delay ( CommandData_T* cmd);
CmdHandlerReturnCode_T InterphaseDelay( CommandData_T* cmd );
CmdHandlerReturnCode_T ReadReturn( CommandData_T* cmd );
CmdHandlerReturnCode_T PerPhasePW(CommandData_T* cmd);
void PerPhaseAmp(CommandData_T* cmd);
void ElectrodetoChannel( CommandData_T* cmd );
void StartContinuousStim( void );
void Amplitude( CommandData_T* cmd );
void StartDelay( CommandData_T* cmd );
void ElectrodeEnable ( CommandData_T* cmd );
void DisableElectrode( CommandData_T* cmd );
void StopContinuousStim( void );
void StimDuration (CommandData_T* cmd  );
void CurrentTick(uint32_t tick  );
void Starttick(void);
void OffDuration(CommandData_T* cmd);
void NCycles(CommandData_T* cmd);
void Calibration(void);
void DEFAULT_STIM(void);
void AddDelay (unsigned int delay2);
void RAM1(CommandData_T* cmd);
void RAM2(CommandData_T* cmd);
void SelectShape( CommandData_T* cmd );
void EditChargeLimit(CommandData_T* cmd);
void chargecheck (CommandData_T* cmd);
void UpdateSequencerAddresses (unsigned int period);

}
