add_library(mpu6050 INTERFACE)

target_sources(mpu6050 INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/mpu6050.c
)

target_include_directories(mpu6050 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Pull in pico libraries that we need
target_link_libraries(mpu6050 INTERFACE pico_stdlib hardware_i2c)