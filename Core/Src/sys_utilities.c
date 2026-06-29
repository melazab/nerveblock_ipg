/// @file    sys_utilities.c
/// @brief   See the H file for the module description.
/// @author  Dale Walter for Carroll Biomedical
/// @date    May 5, 2025

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32wbxx_hal.h"

/**
 * @brief   This function is executed in case of error occurrence.
 * @details If is called by many of the MX generated routines.
 *          It will flash one of the LED indefinatly
 * @retval  None
 */
void custom_error_handler( void )
{
    
}

int _write( int le, char* ptr, int len )
{
    int DataIdx;

    for ( DataIdx = 0; DataIdx < len; DataIdx++ )
    {
        ITM_SendChar( *ptr++ );
    }

    return len;
}

// #undef USE_FULL_ASSERT     // needed to prevent a warning
// #define USE_FULL_ASSERT    // also can't figure out why this is needed when USE_FULL_ASSERT is
//                            // defined in the Makefile
// #ifdef USE_FULL_ASSERT
// /**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
// void assert_failed( uint8_t* file, uint32_t line )
// {
//     /* USER CODE BEGIN 6 */
//     /* User can add his own implementation to report the file name and line number,
//        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//     printf( "assert_param failed: file %s on line %ld\r\n", file, line );
//     /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */
