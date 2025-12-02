#!/bin/sh
# Build script for Bramble test firmware

set -e  # Exit on error

echo "Building Bramble test firmware..."

# Create build directory
mkdir -p build
cd build

# Compile assembly
echo "[1/5] Compiling boot.S..."
arm-none-eabi-as ../boot.S -o boot.o \
    -mcpu=cortex-m0plus -mthumb

# Compile C code
echo "[2/5] Compiling hello_world.c..."
arm-none-eabi-gcc -c ../hello_world.c -o hello_world.o \
    -mcpu=cortex-m0plus -mthumb -O2 -Wall -Wextra

# Link
echo "[3/5] Linking..."
arm-none-eabi-gcc boot.o hello_world.o -o hello_world.elf \
    -T ../memmap.ld -nostdlib -mcpu=cortex-m0plus -mthumb

# Generate binary
echo "[4/5] Creating binary..."
arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin

# Convert to UF2
echo "[5/5] Converting to UF2..."
python3 ../uf2conv.py hello_world.bin -b 0x10000100 -o ../../hello_world.uf2

echo "✓ Build complete: hello_world.uf2"
echo ""
echo "Run with: ./bramble hello_world.uf2"
