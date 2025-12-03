#!/bin/bash
set -e

echo "Building Bramble test firmware..."
mkdir -p build
cd build

# Determine which firmware to build
TARGET="${1:-hello_world}"

case "$TARGET" in
    hello_world|hello)
        echo "[1/3] Compiling hello_world.S..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o
        echo "[2/3] Linking..."
        arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf
        echo "[3/3] Converting to UF2..."
        arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin
        python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ Build complete: hello_world.uf2"
        ;;
    
    gpio|gpio_test)
        echo "[1/3] Compiling gpio_test.S..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../gpio_test.S -o gpio_test.o
        echo "[2/3] Linking..."
        arm-none-eabi-ld -T ../linker.ld gpio_test.o -o gpio_test.elf
        echo "[3/3] Converting to UF2..."
        arm-none-eabi-objcopy -O binary gpio_test.elf gpio_test.bin
        python3 ../uf2conv.py gpio_test.bin -o ../../gpio_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ Build complete: gpio_test.uf2"
        ;;
    
    all)
        echo "Building all test firmware..."
        # Hello World
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o
        arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf
        arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin
        python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ hello_world.uf2"
        
        # GPIO Test
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../gpio_test.S -o gpio_test.o
        arm-none-eabi-ld -T ../linker.ld gpio_test.o -o gpio_test.elf
        arm-none-eabi-objcopy -O binary gpio_test.elf gpio_test.bin
        python3 ../uf2conv.py gpio_test.bin -o ../../gpio_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ gpio_test.uf2"
        
        echo "✓ All firmware built successfully"
        ;;
    
    *)
        echo "Usage: ./build.sh [hello_world|gpio|all]"
        echo "  hello_world (default) - Build hello world test"
        echo "  gpio                  - Build GPIO test"
        echo "  all                   - Build all tests"
        cd .. && rm -rf build
        exit 1
        ;;
esac

cd .. && rm -rf build
echo "Done!"
