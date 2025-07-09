cmake_minimum_required (VERSION 3.13)
set(PICO_SDK_PATH ~/projects/rp2040/pico-sdk-2.0)
include(pico_sdk_import.cmake)
project(reverb C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(reverb
)

target_sources(reverb PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/reverb.c
        ${CMAKE_CURRENT_LIST_DIR}/reverb_dsp.c
        ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
      )


target_compile_definitions(reverb PRIVATE
PICO_DEFAULT_UART_BAUD_RATE=115200

)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(reverb PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_compile_definitions(reverb PUBLIC PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

pico_enable_stdio_uart(reverb 0)
pico_enable_stdio_usb(reverb 0)
pico_add_extra_outputs(reverb)

target_link_libraries(reverb tinyusb_device tinyusb_board pico_unique_id pico_stdlib hardware_dma hardware_i2c hardware_pwm hardware_adc hardware_spi hardware_timer pico_multicore)
