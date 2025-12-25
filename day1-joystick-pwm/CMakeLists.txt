cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(breadboard C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(breadboard
    led.c
)

pico_enable_stdio_usb(breadboard 1)
pico_enable_stdio_uart(breadboard 0)

pico_add_extra_outputs(breadboard)

target_link_libraries(breadboard pico_stdlib hardware_adc hardware_pwm)