/*****************************************************************************/
/*!
 * \file    Crc16.h
 *
 * \brief   Provides 16-bit CRC Calculations.
 * 
 * \details The polynomial and the initail value are configurable when the
 *          module is initialized. When the module is initialize a lookup
 *          table is calculated and that is used during CRC calculations
 *          to speed up the calculations.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    Aug 15, 2021
 *
 *****************************************************************************/

#ifndef CRC16_H_
#define CRC16_H_
    
#include <stdint.h>
#include <stdbool.h>

#ifndef CRCVER
#define CRCVER 2
#endif

#if (CRCVER==1)
    #define CRC_POLY_MODBUS     0xA001  // Seems like the polynomial bits need to 
                                        // be reversed in version one
    #define CRC_INIT_MODBUS     0xFFFF
    #define CRC_POLY_CCITT      0x8408
    #define CRC_INIT_CCITT_AUG  0x1D0F
    #define CRC_POLY_DEFAULT    0x8408
    #define CRC_INIT_DEFAULT    0x0000
#else
    #define CRC_POLY_MODBUS     0x8005 
    #define CRC_POLY_BUYPASS    0x8005 
    #define CRC_POLY_CCITT      0x1021
    #define CRC_POLY_DEFAULT    0x1021

    #define CRC_INIT_DEFAULT    0x0000
    #define CRC_INIT_CCITT_AUG  0x1D0F
    #define CRC_INIT_XMODEM     0x0000  //poly = CCITT
    #define CRC_INIT_CCITT_NEG  0xFFFF
    #define CRC_INIT_MODBUS     0xFFFF
    #define CRC_INIT_BUYPASS    0x0000
#endif



/*****************************************************************************/
/*!
 * \brief   Initialize the CRC16 module.
 *
 * \param [in]  uint16_t poly16     The 16-bit polynomial to use for the
 *                                  CRC calculation.
 * \param [in]  uint16_t init16     The 16-bit initial value to use for the
 *                                  CRC calculation.
 *
 * \returns void
 *
 *****************************************************************************/
void crc16ModuleInit(uint16_t poly16, uint16_t init16);

/*****************************************************************************/
/*!
 * \brief   Resets the CRC16 module.
 *
 * \details Resets the CRC Initial value to 0x0000.
 *          Resets the CRC Lookup Table Initialize flag to false so that the
 *          module thinks that the Lookup Table is uninitialized.
 *          This module's main purpose will be in testing. It is unlikely it
 *          will be used in production code.
 *
 * \returns void
 *
 *****************************************************************************/
void crc16ModuleReset(void);

/*****************************************************************************/
/*!
 * \brief Determine is the CRC is correct for the given array of bytes.
 * 
 * \details 
 * The function takes a pointer to a byte array and the size of the byte 
 * array. It then verifies that the CRC of the byte array is correct.  It 
 * is assumed that the last two bytes of the array are the 
 * 16 bit CRC for CRC-CCITT (0xFFFF).
 * 
 * \param [in]  ba          A pointer to the byte array.
 * \param [in]  arraySize   The number of bytes in the array.
 * 
 * \returns bool - That is True if the CRC is correct for the data portion of
 *          the array and False if the CRC of the data does not match the CRC.
 * 
 *****************************************************************************/
bool crc16IsCrcCorrect(uint8_t *ba, int arraySize);

/*****************************************************************************/
/*!
 * \brief Retrieves the CRC value
 * 
 * \details 
 * The function takes a pointer to a byte array and calculates the
 * 16 bit CRC (CRC-CCITT (0xFFFF)) for the data.  The CRC is returned.
 * 
 * @returns uint16_t - The 16 bit CRC of the data bytes in the array.
 * 
 *****************************************************************************/
uint16_t crc16GetCrc(uint8_t *ba, int arraySize);

#endif /* CRC16_H_ */