/*****************************************************************************/
/*! 
 * \file    SerialInterface.h
 *
 * \brief   Handles the incoming byte stream from the UART interfaces. Sends 
 *          response packets to the UART interfaces.
 *
 * \details Interfaces with the semaphore from the UART receive interrupt 
 *          handler to know when data is ready in the receiver. Receives 
 *          response data via a queue and applies the appropiate protocol
 *          wrapper around it before sending it to the UART interface.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    Feb 8, 2023
 *
 *****************************************************************************/
#ifndef SERIALINTERFACE_H_
#define SERIALINTERFACE_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "CmdProtocolDefinitions.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/*****************************************************************************/
/*!
 * \brief   Performs the SerialInterface initialization.
 * 
 * \details TODO more details about what the module does.
 * 
 * \param   void.
 *
 * \returns void
 *
 *****************************************************************************/
void serIfInit(void);

/*****************************************************************************/
/*!
 * \brief   Initializes the GPIO pins for the UART for the ST Debug port.
 * 
 * \details This function is called by HAL_UART_MspInit() in the MSP module
 *          and is needed to support the HAL Driver and HAL_UART_Init().
 * 
 * \param   void.
 *
 * \returns void
 *
 *****************************************************************************/
void serIfGpioInit(void);

/*****************************************************************************/
/*!
 * \brief   Deinitializes the GPIO pins for the UART for the ST Debug port.
 * 
 * \details This function is called by HAL_UART_MspDeInit() in the MSP module
 *          and is needed to support the HAL Driver and HAL_UART_DeInit().
 * 
 * \param   void.
 *
 * \returns void
 *
 *****************************************************************************/
void serIfGpioDeInit(void);

/*****************************************************************************/
/*!
 * \brief   Thread to process incoming bytes on the serial port.
 * 
 * \details The thread that waits on a semaphore from the serial port and DMA
 *          ISRs. When signaled by the semaphore it takes bytes out of the
 *          DMA buffer and sends them to the command parser.
 * 
 * \param   ULONG treadInput
 * 
 * \retval None
 *
*****************************************************************************/
void serIfByteInThreadEntry(uint32_t treadInput);

/*****************************************************************************/
/*!
 * \brief   Writes a buffer of bytes to the ST Debug port.
 * 
 * \param   uint8_t *buf - A pointer to the memory area containing the data
 *                         to write to the port.
 * \param   uint16_t bufLen - The number of bytes in the buffer to be written.
 *
 * \returns void
 *
 *****************************************************************************/
void serIfWrite(uint8_t *buf, uint16_t bufLen);

/*****************************************************************************/
/*!
 * \brief   Sends the pre-built data packet.
 *
 * \param  SERIAL_DATA_OUT_BUF_T *serOutBuf - A pointer to the serial data
 *                                            output buffer to send out the 
 *                                            interface.
 * 
 * \returns void
 *
 *****************************************************************************/
void serIntSendSerialDataBuf(SERIAL_DATA_OUT_BUF_T *serOutBuf);


#ifdef __cplusplus
} // extern "C"
#endif

#endif /*SERIALINTERFACE_H_*/

/*** End of File **************************************************************/
