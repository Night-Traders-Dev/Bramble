cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(bramble_test C CXX ASM)

pico_sdk_init()

add_executable(hello_world
    hello_world.c
)

# Use custom linker script
#pico_set_linker_script(hello_world ${CMAKE_CURRENT_SOURCE_DIR}/memmap.ld)

# Enable UART stdio
pico_enable_stdio_uart(hello_world 1)
pico_enable_stdio_usb(hello_world 0)

# Generate extra outputs
pico_add_extra_outputs(hello_world)

# Link libraries
target_link_libraries(hello_world 
    pico_stdlib
    hardware_uart
    hardware_gpio
)
