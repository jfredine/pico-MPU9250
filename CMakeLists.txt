add_library(MPU9250 INTERFACE)
target_sources(MPU9250 INTERFACE MPU9250.cc)
target_include_directories(MPU9250 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(MPU9250 INTERFACE pico_stdlib hardware_i2c)
