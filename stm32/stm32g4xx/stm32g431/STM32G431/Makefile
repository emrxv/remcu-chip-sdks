
# include $(MCU_UTIL_PATH)/python.mk

# List of C source files.
C_SRC = \
$(MCU_SDK_PATH)/Libraries/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_opamp.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_dma.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_tim.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_hrtim.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_exti.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_comp.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_crc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_spi.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_rcc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_ll_fmc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_rtc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_iwdg.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_gpio.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_dac.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_adc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_i2c.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_wwdg.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_usart.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_flash.c \

SRC_NOT_USED = \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_pwr.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_misc.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_dbgmcu.c \
$(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/src/stm32g4xx_hal_can.c \

# List of include paths.
INC_PATH = \
       $(MCU_SDK_PATH)/Libraries/STM32G4xx_HAL_Driver/inc/ \
       $(MCU_SDK_PATH)/Libraries/CMSIS/Include/ \
       $(MCU_SDK_PATH)/Libraries/CMSIS/Device/ST/STM32G4xx/Include/ \
       $(MCU_SDK_PATH)/Projects/stm32G4XX_hal_driver_Templates/



INC_NOT_USED = \

# TOOLCHAIN OPTIONS
#-------------------------------------------------------------------------------

DEFS = -DREMCU_LIB -DUSE_STDPERIPH_DRIVER -D$(DEVICE)



include $(TARGET_MK)


