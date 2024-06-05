
cmake_minimum_required(VERSION 3.10)

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

set(DESTINATION_DIR_NAME_PREFIX ${DESTINATION_DIR_NAME_PREFIX}/STM32G4XX/)

set(CMAKE_MODULE_PATH ${REMCU_VM_PATH}/cmake)
include(MultiBuild)

add_subdirectory(stm32g431)
