#ifndef __DIO_STM32_H
#define __DIO_STM32_H

#include "dio_base.hpp"    // for dio_base
#include <cstdint>         // for std::uint32_t


class dio_stm32 : public dio_base
{
public:
    // Constructors and Destructors
    dio_stm32()  = default;
    ~dio_stm32() = default;
    dio_stm32( dioPort_t p_port, dioPin_t p_pin, logic_level p_logic_level );

    // Class methods
    void write( dio_base::dioState_t state ) override;
    dioState_t read() override;
    void activate() override;
    void deactivate() override;
};

#endif /* __DIO_STM32_HPP */
