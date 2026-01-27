cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(move_all C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(move_all
    move_all.c
)

pico_enable_stdio_usb(move_all 1)
pico_enable_stdio_uart(move_all 0)

pico_add_extra_outputs(move_all)

target_link_libraries(move_all pico_stdlib hardware_pwm)
