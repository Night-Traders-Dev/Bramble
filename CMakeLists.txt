cmake_minimum_required(VERSION 3.10)
project(Bramble
    VERSION 0.43.0
    DESCRIPTION "A lightweight RP2040/RP2350 emulator in C"
    LANGUAGES C
)

# C Standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -pedantic -g -Wno-stringop-overflow")

# ========================================================================
# Optional Features
# ========================================================================

option(ENABLE_FUSE "Enable FUSE filesystem mount support (requires libfuse3)" OFF)

# Include directories
include_directories(include include/rp2040 include/rp2350_rv include/rp2350_arm)

# ========================================================================
# Source files (shared between all targets)
# ========================================================================

set(EMU_SOURCES
    src/cpu.c
    src/instructions.c
    src/thumb32.c
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
    src/devtools.c
    src/rp2350_rv/rv_cpu.c
    src/rp2350_rv/rv_clint.c
    src/rp2350_rv/rv_membus.c
    src/rp2350_rv/rv_bootrom.c
    src/rp2350_rv/rp2350_periph.c
    src/rp2350_rv/picobin.c
    src/rp2350_arm/m33_cpu.c
)

# ========================================================================
# FUSE support (auto-detect or explicit)
# ========================================================================

if(ENABLE_FUSE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FUSE3 REQUIRED fuse3)
    add_compile_definitions(ENABLE_FUSE)
    include_directories(${FUSE3_INCLUDE_DIRS})
    set(FUSE_LIBS ${FUSE3_LIBRARIES})
    message(STATUS "FUSE support: enabled (${FUSE3_VERSION})")
else()
    set(FUSE_LIBS "")
    message(STATUS "FUSE support: disabled (use -DENABLE_FUSE=ON to enable)")
endif()

# Common link libraries
set(COMMON_LIBS m pthread ${FUSE_LIBS})

# ========================================================================
# Targets
# ========================================================================

# Main executable
add_executable(bramble src/main.c ${EMU_SOURCES})
target_link_libraries(bramble ${COMMON_LIBS})
add_custom_command(TARGET bramble POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:bramble> ${CMAKE_SOURCE_DIR}/bramble
    COMMENT "Copying bramble to project root"
)

# Test executable
enable_testing()
add_executable(bramble_tests tests/test_suite.c ${EMU_SOURCES})
target_link_libraries(bramble_tests ${COMMON_LIBS})
add_test(NAME bramble_tests COMMAND bramble_tests)

# Benchmark executable
add_executable(bramble_bench tests/benchmark.c ${EMU_SOURCES})
target_link_libraries(bramble_bench ${COMMON_LIBS})

# Install target
install(TARGETS bramble DESTINATION bin)

# ========================================================================
# Build summary
# ========================================================================

message(STATUS "")
message(STATUS "Bramble RP2040 Emulator v${PROJECT_VERSION}")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  FUSE mount:    ${ENABLE_FUSE}")
message(STATUS "  C compiler:    ${CMAKE_C_COMPILER}")
message(STATUS "")
