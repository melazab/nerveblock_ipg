#include "dio_stm32.hpp"

extern "C" {
#include "stm32wbxx_hal.h"
}

// :port( port ),
dio_stm32::dio_stm32(dioPort_t p_port, dioPin_t p_pin,
                     logic_level p_logic_level) {
  port = p_port;
  pin = p_pin;

  if (Positive_Logic == p_logic_level) {
    pin_level_activated = HIGH;
    pin_level_deactivated = LOW;
  } else {
    pin_level_activated = LOW;
    pin_level_deactivated = HIGH;
  }
}

void dio_stm32::write(dio_base::dioState_t state) {
  HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin, (GPIO_PinState)state);
}

dio_base::dioState_t dio_stm32::read() {
  return (dioState_t)HAL_GPIO_ReadPin((GPIO_TypeDef *)port, pin);
}

void dio_stm32::activate() {
  HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin,
                    (GPIO_PinState)pin_level_activated);
}

void dio_stm32::deactivate() {
  HAL_GPIO_WritePin((GPIO_TypeDef *)port, pin,
                    (GPIO_PinState)pin_level_deactivated);
}
