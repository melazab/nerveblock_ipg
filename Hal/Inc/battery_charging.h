#ifdef __cplusplus
extern "C"{
#endif

#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"

void batt_charging_entry( ULONG treadInput );

void ADC1_Init_BATT( void );

void ADC1_VREF( void );

void meas_voltages( float* vbat_ptr, float* vrect_ptr );

void return_voltages( uint8_t* vbat_ptr, uint8_t* vrect_ptr );

CmdHandlerReturnCode_T cmd_hndl_Status( CommandData_T* cmd );

#ifdef __cplusplus
} // extern "C"
#endif