

#ifndef USB_H_
#define USB_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#include "CmdProtocolDefinitions.h"
extern uint32_t usbtick;
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
void usbInit(void);

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
void usbGpioInit(void);

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
void usbGpioDeInit(void);

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
void usbByteInThreadEntry(uint32_t treadInput);

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
void usbWrite(uint8_t *buf, uint16_t bufLen);

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
void usbSendSerialDataBuf(SERIAL_DATA_OUT_BUF_T *serOutBuf);

void usbDataReady( uint16_t size );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*SERIALINTERFACE_H_*/

/*** End of File **************************************************************/
