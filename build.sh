#!/bin/sh

make clean
git pull origin main
git submodule update --init --recursive --remote

echo "*==Bramble RP2040 Emulator==*\n"
mkdir -p build && cd build
echo "running cmake and make...bramble\n"
cmake .. && make CORES=2 -j$(nproc)
echo "moving bramble binary and removing build directory...\n"
mv ./bramble ../bramble
mv ./bramble_tests ../bramble_tests
cd ../ && rm -rf build
echo "*==Bramble Build Complete==*\n"
