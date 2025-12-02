#!/bin/sh

echo "*==Bramble RP2040 Emulator==*\n"
mkdir -p build && cd build
echo "running cmake and make...bramble\n"
cmake .. && make -j$(nproc)
echo "moving bramble binary and removing build directory...\n"
mv ./bramble ../bramble
cd ../ && rm -rf build
echo "*==Bramble Build Complete==*\n"
