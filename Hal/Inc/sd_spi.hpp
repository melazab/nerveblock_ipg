/// @file    ext_flash_spi.hpp
/// @brief   This module handles the SPI interface to the External Flash.
/// @details Configures and initializes the SPI peripheral.
/// 			Handles enabling and disabling the GPIO pins for the SPI.
#pragma once

#include "stm32wbxx_hal.h"

namespace SD
{

/// @brief   Get a pointer to the handle for the SPI port that controls
///          the Flash.
/// @returns The pointer to the handle (SPI port register structure).
SPI_HandleTypeDef* get_handle_ptr( void );

/// @brief   Initializes the GPIO pins for the Flash SPI port.
/// @details This function is called by HAL_SPI_MspInit() in the spi module
///          and is needed to support the HAL SPI Driver and HAL_SPI_Init().
/// @param   void.
/// @returns void
void gpio_init( void );

/// @brief   Deinitializes the GPIO pins for the Flash SPI port.
/// @details This function is called by HAL_SPI_MspDeInit() in the spi module
///          and is needed to support the HAL SPI Driver and HAL_SPI_DeInit().
/// @param   void.
/// @returns void
void gpio_deinit( void );

/// @brief   Enables the SPI port for the Flash.
/// @details The Flash needs to be powered before this routine is called.
/// 			Initializes the SPI peripheral if it has not been initiailize.
/// 			Configures the GPIO pins for SPI mode.
/// 			Deasserts the Chip Select pin.
/// @param   void.
/// @returns void
void enable( void );

/// @brief   Disables the SPI port for the Flash.
/// @details After this routine is called it is safe to power down the Flash.
///          Deconfigures the SPI port pins so they are not in SPI mode. If they
///          are left in SPI mode when the IC is powered off they could
///          parasitically power the IC and cause high current draw.
///          Asserts the Chip Select pin. i.e. Sets it low so that it doesn't
///          parasitically power the IC and cause high current draw.
/// @param   void.
/// @returns void
void disable( void );

/// @brief      Write a block of data to the SPI.
/// @param[in]  data_length: The number of bytes to write.
/// @param[in]  data_buffer: A pointer to the data bytes to write.
/// @returns    HAL_StatusTypeDef - The error codes defined by the HAL
HAL_StatusTypeDef write( const uint8_t data_length, const uint8_t* data_buffer );

/// @brief      Writes multiple buffers of SPI data as one transaction.
/// @param[in]  preamble_length: The number of bytes to write for the preamble.
/// @param[in]  preamble_buffer: A pointer to the preamble.
/// @param[in]  data_length: The number of bytes to write from the data buffer.
/// @param[in]  data_buffer: A pointer to the second data buffer to write.
/// @returns    HAL_StatusTypeDef - The error codes defined by the HAL
HAL_StatusTypeDef write_page( const uint8_t preamble_length,
                                            const uint8_t* preamble_buffer,
                                            const uint16_t data_length,
                                            const uint8_t* data_buffer );

/// @brief      Writes/Reads multiple buffers of SPI data as one transaction.
/// @param[in]  preamble_length: The number of bytes to write from the first buffer.
/// @param[in]  preamble_buffer: A pointer to the first data buffer to write.
/// @param[in]  data_length: The number of bytes to read to the second buffer.
/// @param[out]  data_buffer: A pointer to the read data buffer to read to.
/// @returns    HAL_StatusTypeDef - The error codes defined by the HAL
HAL_StatusTypeDef read_page( const uint8_t preamble_length,
                                           const uint8_t* preamble_buffer,
                                           const uint16_t data_length, uint8_t* data_buffer );

/// @brief      Write and read data to/from the SPI.
/// @param[in]  data_length: The number of bytes to write.
/// @param[in]  tx_buffer: A pointer to the data bytes to write.
/// @param[out] rx_buffer: A pointer to the buffer where the data read from the SPI will be return
///                        to. Must have a size of at least data_length.
/// @returns    HAL_StatusTypeDef - The error codes defined by the HAL
HAL_StatusTypeDef write_read( const uint8_t data_length, const uint8_t* tx_buffer,
                                            uint8_t* rx_buffer );

}
