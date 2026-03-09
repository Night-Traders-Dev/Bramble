cmake_minimum_required(VERSION 3.10)
project(Bramble 
    VERSION 0.1.0
    DESCRIPTION "A lightweight RP2040 emulator in C"
    LANGUAGES C
)

# C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -g")

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.c
    src/cpu.c
    src/instructions.c
    src/membus.c
    src/uf2.c
    src/elf.c
    src/gpio.c
    src/timer.c
    src/nvic.c
    src/clocks.c
    src/adc.c
    src/rom.c
    src/uart.c
    src/spi.c
    src/i2c.c
    src/pwm.c
    src/dma.c
    src/pio.c
    src/gdb.c
    src/usb.c
)

# Executable
add_executable(bramble ${SOURCES})

# Optional: Enable testing
enable_testing()

# Library of emulator sources (shared between main executable and tests)
set(LIB_SOURCES
    src/cpu.c
    src/instructions.c
    src/membus.c
    src/uf2.c
    src/elf.c
    src/gpio.c
    src/timer.c
    src/nvic.c
    src/clocks.c
    src/adc.c
    src/rom.c
    src/uart.c
    src/spi.c
    src/i2c.c
    src/pwm.c
    src/dma.c
    src/pio.c
    src/usb.c
)

# Test executable
add_executable(bramble_tests tests/test_suite.c ${LIB_SOURCES})
add_test(NAME bramble_tests COMMAND bramble_tests)

# Install target
install(TARGETS bramble DESTINATION bin)
