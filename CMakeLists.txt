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

# Optional FUSE support
option(ENABLE_FUSE "Enable FUSE filesystem mount support" OFF)

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
    src/rtc.c
    src/netbridge.c
    src/wire.c
    src/storage.c
    src/sdcard.c
    src/emmc.c
    src/fatfs.c
    src/fuse_mount.c
    src/w5500.c
    src/bme280.c
    src/corepool.c
    src/cyw43.c
    src/tapif.c
)

# Executable
add_executable(bramble ${SOURCES})
target_link_libraries(bramble m pthread)  # math + pthreads
add_custom_command(TARGET bramble POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:bramble> ${CMAKE_SOURCE_DIR}/bramble
    COMMENT "Copying bramble to project root"
)

# FUSE support
if(ENABLE_FUSE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FUSE3 REQUIRED fuse3)
    target_compile_definitions(bramble PRIVATE ENABLE_FUSE)
    target_include_directories(bramble PRIVATE ${FUSE3_INCLUDE_DIRS})
    target_link_libraries(bramble ${FUSE3_LIBRARIES} pthread)
endif()

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
    src/rtc.c
    src/netbridge.c
    src/wire.c
    src/storage.c
    src/sdcard.c
    src/emmc.c
    src/fatfs.c
    src/fuse_mount.c
    src/w5500.c
    src/bme280.c
    src/corepool.c
    src/gdb.c
    src/cyw43.c
    src/tapif.c
)

# Test executable
add_executable(bramble_tests tests/test_suite.c ${LIB_SOURCES})
target_link_libraries(bramble_tests m pthread)
add_test(NAME bramble_tests COMMAND bramble_tests)

# Benchmark executable
add_executable(bramble_bench tests/benchmark.c ${LIB_SOURCES})
target_link_libraries(bramble_bench m pthread)

# Install target
install(TARGETS bramble DESTINATION bin)
