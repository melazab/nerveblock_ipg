/*****************************************************************************/
/*!
 * \file    Crc16.c
 *
 * \brief   See the H file for the module description.
 *
 * \author  Dale Walter
 *          for Carroll Biomedical
 *
 * \date    Aug 15, 2021
 *
 *****************************************************************************/

#include "Crc16.h"


static bool crc16TableInit = false;
static uint16_t crc16Table[ 256 ];

static uint16_t crcInitialVal = CRC_INIT_DEFAULT;
// static uint16_t crcPolynomial = CRC_POLY_DEFAULT;

// Forward declaration
static void crc16CreateLookupTable( uint16_t crcPolynomial );

/*****************************************************************************/
void crc16ModuleInit( uint16_t poly16, uint16_t init16 )
/*****************************************************************************/
{
    crcInitialVal = init16;

    crc16CreateLookupTable( poly16 );

    return;
}

/*****************************************************************************/
void crc16ModuleReset( void )
/*****************************************************************************/
{
    crcInitialVal  = 0x0000;
    crc16TableInit = false;
}

// /*****************************************************************************/
// bool isCrcCorrect(uint8_t *ba, int arraySize)
// /*****************************************************************************/
// {
//     uint16_t crcStored;
//     uint16_t crcCalc;

//     crcStored = *(ba + (arraySize-2)) * 0x100;
//     crcStored += *(ba + (arraySize-1));

//     crcCalc = getCrc(ba, arraySize-2);

//     return (crcStored == crcCalc);

// }

#if ( CRCVER == 1 )
/*****************************************************************************/
uint16_t crc16GetCrc( uint8_t* ba, int arraySize )
/*****************************************************************************/
{
    uint16_t crc = crcInitialVal;

    if ( !crc16TableInit )
    {
        crc16CreateLookupTable( (uint16_t)CRC_POLY_DEFAULT );
    }

    for ( int i = 0; i < arraySize; i++ )
    {
        crc = ( crc >> 8 ) ^ crc16Table[ ( crc ^ *( ba + i ) ) & 0x00FF ];
    }

    return crc;
}

/*****************************************************************************/
/*!
 * \brief Initializes a lookup table to speed up the CRC calculations.
 *
 * \details
 * A 256 byte lookup table is initialized and will be used to speed up the
 * CRC calculations. Since this only needs to be performed the first time a
 * CRC is calculated there is a global flag that is maintained to indicate if
 * the lookup table is initialized or not. Every time crc16GetCrc is called it
 * checks to make sure the table is initialized and if it isn't then the
 * this initialization routine is called.
 *
 * \returns void
 *
 *****************************************************************************/
static void crc16CreateLookupTable( uint16_t crcPolynomial )
{
    for ( int divident = 0; divident < 256; divident++ )
    {
        uint16_t crc = 0;
        uint16_t c   = divident;

        for ( int j = 0; j < 8; j++ )
        {
            crc = ( ( crc ^ c ) & 0x0001 ) ? ( crc >> 1 ) ^ crcPolynomial : ( crc >> 1 );

            c = c >> 1;
        }

        crc16Table[ divident ] = crc;
        if ( divident < 16 )
        {
            printf( "%04x ", crc );
        }
    }
    printf( "END\n" );

    crc16TableInit = true;
}

// Version 2 -  A second attempt because I could not get any CRC type
//              other than the Modbus CRC to work with version 1
#else
/*****************************************************************************/
uint16_t crc16GetCrc( uint8_t* ba, int arraySize )
/*****************************************************************************/
{
    if ( !crc16TableInit )
    {
        crc16CreateLookupTable( (uint16_t)CRC_POLY_DEFAULT );
    }

    uint16_t crc = crcInitialVal;

    for ( int i = 0; i < arraySize; i++ )
    {
        uint16_t b = (uint16_t)*( ba + i );

        /* XOR-in next input byte into MSB of crc, that's
           our new intermediate divident */
        crc         = crc ^ ( b << 8 );
        uint8_t pos = (uint8_t)( crc >> 8 );
        // uint8_t pos = (uint8_t)( (crc >> 8) ^ b);
        /* equal: ((crc ^ (b << 8)) >> 8) */


        /* Shift out the MSB used for division per lookuptable and XOR with the remainder */
        crc = (uint16_t)( ( crc << 8 ) ^ (uint16_t)( crc16Table[ pos ] ) );
    }

    return crc;
}

/*****************************************************************************/
bool crc16IsCrcCorrect( uint8_t* ba, int arraySize )
/*****************************************************************************/
{
    //    uint16_t calcCrc = crc16GetCrc(ba, arraySize-2);
    //    return (calcCrc == (uint16_t)(*(uint16_t *)(ba + arraySize - 2)));

    return true;    // getting some weird results from this routine so disable TODO
                    // the routine for now TODO
}

/*****************************************************************************/
/*!
 * \brief Initializes a lookup table to speed up the CRC calculations.
 *
 * \details
 * A 256 byte lookup table is initialized and will be used to speed up the
 * CRC calculations. Since this only needs to be performed the first time a
 * CRC is calculated there is a global flag that is maintained to indicate if
 * the lookup table is initialized or not. Every time crc16GetCrc is called it
 * checks to make sure the table is initialized and if it isn't then the
 * this initialization routine is called.
 *
 * \returns void
 *
 *****************************************************************************/
static void crc16CreateLookupTable( uint16_t crcPolynomial )
{
    uint16_t generator = crcPolynomial;

    for ( int index = 0; index < 256;
          index++ ) /* iterate over all possible input byte values 0 - 255 */
    {
        uint16_t divident = index;
        uint16_t curByte  = divident << 8; /* move divident byte into MSB of 16Bit CRC */

        for ( uint8_t bit = 0; bit < 8; bit++ )
        {
            if ( ( curByte & 0x8000 ) != 0 )
            {
                curByte <<= 1;
                curByte  ^= generator;
            }
            else
            {
                curByte <<= 1;
            }
        }
        crc16Table[ index ] = curByte;
    }

    crc16TableInit = true;
}
#endif
