cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(raw_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.1.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.1.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
pico_sdk_init()

add_executable(raw main.cc)

add_subdirectory(../.. MPU9250)

pico_add_extra_outputs(raw)

target_link_libraries(raw pico_stdlib MPU9250)

set(MPU9250_BUS "I2C" CACHE STRING "Select I2C or SPI bus for accessing MPU9250")
if (MPU9250_BUS MATCHES "^SPI$")
    target_compile_definitions(MPU9250 INTERFACE MPU9250_SPI)
endif()
