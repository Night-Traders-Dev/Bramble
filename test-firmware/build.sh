#!/bin/bash
echo "Building Bramble test firmware..."
mkdir -p build
cd build

echo "[1/3] Compiling hello_world.S..."
arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o

echo "[2/3] Linking..."
arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf

echo "[3/3] Converting to UF2..."
arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin
python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000000 -f 0xE48BFF56
cd ../ && rm -rf build

echo "✓ Build complete: hello_world.uf2"
