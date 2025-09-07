cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(AHRS_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "2.1.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 2.1.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
pico_sdk_init()

add_executable(AHRS main.cc Madgwick.cc Mahony.cc)

pico_enable_stdio_uart(AHRS 1)
pico_enable_stdio_usb(AHRS 0)

add_subdirectory(../.. MPU9250)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(AHRS)

target_link_libraries(AHRS hardware_gpio hardware_i2c hardware_spi pico_binary_info pico_stdio pico_time MPU9250)

set(MPU9250_BUS "I2C" CACHE STRING "Select I2C or SPI bus for accessing MPU9250")
if (MPU9250_BUS MATCHES "^SPI$")
    add_compile_definitions(MPU9250_SPI=1)
endif()

set(AHRS_ALGO "MADGWICK" CACHE STRING "Select Madgwick or Mahony AHRS algorithm")
if (AHRS_ALGO MATCHES "^MAHONY$")
    add_compile_definitions(AHRS_MAHONY=1)
endif()
