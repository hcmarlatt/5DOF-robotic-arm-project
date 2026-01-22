cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(movement_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(movement_test
    movement_test.c
)

pico_enable_stdio_usb(movement_test 1)
pico_enable_stdio_uart(movement_test 0)

pico_add_extra_outputs(movement_test)

target_link_libraries(movement_test pico_stdlib hardware_pwm)
