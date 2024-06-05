cmake_minimum_required (VERSION 3.10)

set(MCU_TYPE STM32G4xx)

set(MCU_LIB_NAME HAL_Driver)

set(MCU_MAJOR_VERSION_LIB V1.2.3)

set(MCU_MINOR_VERSION_LIB 01)

set(MCU_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/..)

include(${REMCU_VM_PATH}/cmake/mcu_build_target.cmake)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/defines.h.in ${ALL_INCLUDE_DIR}/device_defines.h)

add_custom_command(     
      TARGET MCU_LIB 
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/${MCU_TYPE}_${MCU_LIB_NAME}.py"
      ${ALL_INCLUDE_DIR})

FILE(GLOB COPY_HEADER_FROM_LIB 
      ${MCU_SDK_PATH}/Libraries/CMSIS/Include/core_cm4.h
      ${MCU_SDK_PATH}/Libraries/CMSIS/Device/ST/STM32G4xx/Include/*.h
      ${MCU_SDK_PATH}/Libraries/STM32G4xx_HAL_Driver-1.2.3/inc/*.h
      ${MCU_SDK_PATH}/Projects/stm32g4xx_hal_driver_Templates/stm32g4xx_hal_conf.h
      ${MCU_SDK_PATH}/Libraries/CMSIS/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c
      )


file(INSTALL ${COPY_HEADER_FROM_LIB}
            DESTINATION  ${ALL_INCLUDE_DIR}
            )


add_subdirectory(test)
