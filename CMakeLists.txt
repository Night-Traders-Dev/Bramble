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
    src/membus.c
    src/uf2.c
)

# Executable
add_executable(bramble ${SOURCES})

# Optional: Enable testing
enable_testing()

# Install target
install(TARGETS bramble DESTINATION bin)
