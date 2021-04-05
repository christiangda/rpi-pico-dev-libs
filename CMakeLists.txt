cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(rpi-pico-dev-libs C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(RPI_PICO_DEV_LIBS_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()
include(example_auto_set_url.cmake)

add_subdirectory(drivers)
add_subdirectory(libraries)
add_subdirectory(examples)

