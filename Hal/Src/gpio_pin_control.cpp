/// @file    gpio_pin_control.cpp
/// @brief   See the H file for the module description.
/// @author  Dale Walter for Carroll Biomedical
/// @date    May 12, 2023

#include "stm32wbxx_hal.h"

#include "gpio_pin_control.hpp"

GPIO_TypeDef* Port_Translator[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH };
uint16_t Pin_Translator[ 16 ]
    = { GPIO_PIN_0,  GPIO_PIN_1,  GPIO_PIN_2,  GPIO_PIN_3, GPIO_PIN_4,  GPIO_PIN_5,
        GPIO_PIN_6,  GPIO_PIN_7,  GPIO_PIN_8,  GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11,
        GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };

CmdHandlerReturnCode_T gpio_ctrl::write_pin_handler( CommandData_T* cmd )
{
    uint8_t port_index = 0x0F & ( cmd->cmdDataBuf[ 0 ] >> 4 );

    // if (port_index >= (sizeof(port_translator) / sizeof(GPIO_TypeDef*)))
    if ( port_index >= ( 7 ) )
    {
        cmd->cmdDataLen = 0;
        return CMD_RESP_NACK;
    }
    uint8_t pin_index = 0x0F & cmd->cmdDataBuf[ 0 ];
    bool set          = ( 0x01 == ( 0x01 & cmd->cmdDataBuf[ 1 ] ) );

    HAL_GPIO_WritePin( Port_Translator[ port_index ],
                       Pin_Translator[ pin_index ],
                       ( set ) ? GPIO_PIN_SET : GPIO_PIN_RESET );

    cmd->cmdDataLen = 0;
    return CMD_RESP_ACK;
}

CmdHandlerReturnCode_T gpio_ctrl::read_port_handler( CommandData_T* cmd )
{
    uint8_t port_index = 0x0F & ( cmd->cmdDataBuf[ 0 ] >> 4 );

    // if (port_index >= (sizeof(Port_Translator) / sizeof(GPIO_TypeDef*)))
    if ( port_index >= ( 7 ) )
    {
        cmd->cmdDataLen = 0;
        return CMD_RESP_NACK;
    }

    *( (uint16_t*)&( cmd->cmdDataBuf[ 0 ] ) ) = (uint16_t)( Port_Translator[ port_index ]->IDR );
    cmd->cmdDataLen                           = 2;

    return CMD_RESP_ACK;
}
