# @file   STM32WBxxHalDriverInc.mk
# @brief  Sub-Makefile to make HAL library and drivers
# @author Dale Walter
#             for Carroll Biomedical
# @date   Apr 23, 2025

HAL_DRV_SRC_ROOT = $(HAL_DRV_SUB_ROOT)/Src

################################################################################
# C sources
C_SOURCES +=  \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_adc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_adc_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_rcc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_ll_rcc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_rcc_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_flash.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_flash_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_gpio.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_hsem.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_dma.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_dma_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_pwr.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_pwr_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_cortex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_exti.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_rtc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_rtc_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_spi.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_spi_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_tim.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_tim_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_uart.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_uart_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_i2c_ex.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_i2c.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_ipcc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_hal_rng.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_ll_adc.c \
$(HAL_DRV_SRC_ROOT)/stm32wbxx_ll_i2c.c \

################################################################################
# C includes
C_INCLUDES +=  \
-I$(HAL_DRV_SUB_ROOT)/Inc \
-I$(HAL_DRV_SUB_ROOT)/Inc/Legacy \
-I$(CMSIS_DRV_SUB_ROOT)/Device/ST/STM32WBxx/Include \
-I$(CMSIS_DRV_SUB_ROOT)/Include
