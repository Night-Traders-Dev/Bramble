#!/bin/bash
echo "Building Bramble test firmware..."

# Create build directory
mkdir -p build
cd build

# Compile boot.S
echo "[1/5] Compiling boot.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../boot.S -o boot.o

# Compile hello_world.c
echo "[2/5] Compiling hello_world.c..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O0 -c ../hello_world.c -o hello_world.o

# Link
echo "[3/5] Linking..."
arm-none-eabi-ld -T ../linker.ld boot.o hello_world.o -o hello_world.elf

# Create binary
echo "[4/5] Creating binary..."
arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin

# Convert to UF2
echo "[5/5] Converting to UF2..."
python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000000 -f 0xE48BFF56
cp hello_world.elf ../
cd ../ && rm -rf build

echo "✓ Build complete: hello_world.uf2"
echo ""
echo "Run with: ./bramble hello_world.uf2"
