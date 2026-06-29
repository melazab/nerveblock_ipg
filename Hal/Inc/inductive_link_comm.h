#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "CmdProtocolDefinitions.h"
#include "CmdRespProtocol.h"

void timIntSendSerialDataBuf( SERIAL_DATA_OUT_BUF_T* serOutBuf );

void nfc_rx_falling_edge( void );

void TIM2_ConfigureOutputCompare_CH4(void);

void TIM2_ConfigureInputCapture_CH3(void) ;

#ifdef __cplusplus
}    // extern "C"
#endif