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

    timer|timer_test)
        echo "[1/3] Compiling timer_test.S..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../timer_test.S -o timer_test.o
        echo "[2/3] Linking..."
        arm-none-eabi-ld -T ../linker.ld timer_test.o -o timer_test.elf
        echo "[3/3] Converting to UF2..."
        arm-none-eabi-objcopy -O binary timer_test.elf timer_test.bin
        python3 ../uf2conv.py timer_test.bin -o ../../timer_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ Build complete: timer_test.uf2"
        ;;

    alarm|alarm_test)
        echo "[1/3] Compiling alarm_test.S..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../alarm_test.S -o alarm_test.o
        echo "[2/3] Linking..."
        arm-none-eabi-ld -T ../linker.ld alarm_test.o -o alarm_test.elf
        echo "[3/3] Converting to UF2..."
        arm-none-eabi-objcopy -O binary alarm_test.elf alarm_test.bin
        python3 ../uf2conv.py alarm_test.bin -o ../../alarm_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ Build complete: alarm_test.uf2"
        ;;

    interrupt|interrupt_test)
        echo "[1/3] Compiling interrupt_test.S..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../interrupt_test.S -o interrupt_test.o
        echo "[2/3] Linking..."
        arm-none-eabi-ld -T ../linker.ld interrupt_test.o -o interrupt_test.elf
        echo "[3/3] Converting to UF2..."
        arm-none-eabi-objcopy -O binary interrupt_test.elf interrupt_test.bin
        python3 ../uf2conv.py interrupt_test.bin -o ../../interrupt_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "✓ Build complete: interrupt_test.uf2"
        ;;

    all)
        echo "Building all test firmware..."

        # Hello World
        echo "  - Building hello_world.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../hello_world.S -o hello_world.o
        arm-none-eabi-ld -T ../linker.ld hello_world.o -o hello_world.elf
        arm-none-eabi-objcopy -O binary hello_world.elf hello_world.bin
        python3 ../uf2conv.py hello_world.bin -o ../../hello_world.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ hello_world.uf2"

        # GPIO Test
        echo "  - Building gpio_test.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../gpio_test.S -o gpio_test.o
        arm-none-eabi-ld -T ../linker.ld gpio_test.o -o gpio_test.elf
        arm-none-eabi-objcopy -O binary gpio_test.elf gpio_test.bin
        python3 ../uf2conv.py gpio_test.bin -o ../../gpio_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ gpio_test.uf2"

        # Timer Test
        echo "  - Building timer_test.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../timer_test.S -o timer_test.o
        arm-none-eabi-ld -T ../linker.ld timer_test.o -o timer_test.elf
        arm-none-eabi-objcopy -O binary timer_test.elf timer_test.bin
        python3 ../uf2conv.py timer_test.bin -o ../../timer_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ timer_test.uf2"

        # Alarm Test
        echo "  - Building alarm_test.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../alarm_test.S -o alarm_test.o
        arm-none-eabi-ld -T ../linker.ld alarm_test.o -o alarm_test.elf
        arm-none-eabi-objcopy -O binary alarm_test.elf alarm_test.bin
        python3 ../uf2conv.py alarm_test.bin -o ../../alarm_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ alarm_test.uf2"

        # Interrupt Test
        echo "  - Building interrupt_test.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../interrupt_test.S -o interrupt_test.o
        arm-none-eabi-ld -T ../linker.ld interrupt_test.o -o interrupt_test.elf
        arm-none-eabi-objcopy -O binary interrupt_test.elf interrupt_test.bin
        python3 ../uf2conv.py interrupt_test.bin -o ../../interrupt_test.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ interrupt_test.uf2"

        echo "  - Building shell_loop.uf2..."
        arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -c ../shell_loop.S -o shell_loop.o
        arm-none-eabi-ld -T ../linker.ld shell_loop.o -o shell_loop.elf
        arm-none-eabi-objcopy -O binary shell_loop.elf shell_loop.bin
        python3 ../uf2conv.py shell_loop.bin -o ../../shell_loop.uf2 -b 0x10000000 -f 0xE48BFF56
        echo "    ✓ shell_loop.uf2"


        echo ""
        echo "✓ All firmware built successfully (5/5)"
        ;;

    *)
        echo "Usage: ./build.sh [TARGET]"
        echo ""
        echo "Available targets:"
        echo "  hello_world (default) - Build hello world test"
        echo "  gpio                  - Build GPIO test"
        echo "  timer                 - Build timer test"
        echo "  alarm                 - Build timer alarm test"
        echo "  interrupt             - Build timer interrupt test (full flow)"
        echo "  all                   - Build all tests"
        echo ""
        cd .. && rm -rf build
        exit 1
        ;;
esac

cd .. && rm -rf build
echo "Done!"
