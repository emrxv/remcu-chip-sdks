cmake_minimum_required(VERSION 3.10)

include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

set(CMAKE_MODULE_PATH ${REMCU_VM_PATH}/cmake)
include(MultiBuild)

ExternalProject_Add(STM32G431
      DOWNLOAD_COMMAND ""
      SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431/"
      CMAKE_ARGS ${CMAKE_ARGS}
      )