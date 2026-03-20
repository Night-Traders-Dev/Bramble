#!/bin/sh
# Bramble RP2040/RP2350 Emulator - Build Script
#
# Usage:
#   ./build.sh              Build with auto-detected features
#   ./build.sh --clean      Clean rebuild
#   ./build.sh --no-fuse    Build without FUSE support
#   ./build.sh --release    Release build (optimized, no debug symbols)
#   ./build.sh --debug      Debug build (symbols, no optimization)
#   ./build.sh --help       Show this help
#
# Architecture support:
#   RP2040    - ARM Cortex-M0+ (Thumb-1)     [complete]
#   RP2350_RV - RISC-V Hazard3 (RV32IMAC)    [in progress]
#   RP2350_ARM - ARM Cortex-M33 (ARMv8-M)    [planned]

set -e

# Defaults
BUILD_TYPE="Release"
CLEAN=0
FUSE_FLAG=""
EXTRA_FLAGS=""

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --clean)    CLEAN=1 ;;
        --no-fuse)  FUSE_FLAG="-DENABLE_FUSE=OFF" ;;
        --release)  BUILD_TYPE="Release" ;;
        --debug)    BUILD_TYPE="Debug" ;;
        --help|-h)
            echo "Bramble RP2040/RP2350 Emulator - Build Script"
            echo ""
            echo "Usage: ./build.sh [options]"
            echo ""
            echo "Options:"
            echo "  --clean      Clean rebuild (removes build directory)"
            echo "  --no-fuse    Disable FUSE filesystem mount support"
            echo "  --release    Release build (default, optimized)"
            echo "  --debug      Debug build (symbols, no optimization)"
            echo "  --help       Show this help"
            echo ""
            echo "Auto-detection:"
            echo "  FUSE: enabled if libfuse3 is found (pkg-config fuse3)"
            echo ""
            echo "Architecture targets (all built together):"
            echo "  src/rp2040/       ARM Cortex-M0+ (RP2040)"
            echo "  src/rp2350_rv/    RISC-V Hazard3 (RP2350)"
            echo "  src/rp2350_arm/   ARM Cortex-M33 (RP2350, planned)"
            echo ""
            echo "Build output:"
            echo "  ./bramble          Main emulator binary"
            echo "  ./bramble_tests    Test suite"
            echo "  ./bramble_bench    Performance benchmark"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg (try --help)"
            exit 1
            ;;
    esac
done

echo "╔════════════════════════════════════════════╗"
echo "║   Bramble RP2040/RP2350 Emulator - Build ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Clean if requested
if [ "$CLEAN" = "1" ]; then
    echo "[Build] Cleaning build directory..."
    rm -rf build
fi

# Auto-detect FUSE if not explicitly disabled
if [ -z "$FUSE_FLAG" ]; then
    if pkg-config --exists fuse3 2>/dev/null; then
        FUSE_FLAG="-DENABLE_FUSE=ON"
        FUSE_VER=$(pkg-config --modversion fuse3)
        echo "[Build] FUSE support: enabled (libfuse3 ${FUSE_VER})"
    else
        FUSE_FLAG="-DENABLE_FUSE=OFF"
        echo "[Build] FUSE support: disabled (libfuse3 not found)"
        echo "        Install: sudo apt install libfuse3-dev"
    fi
else
    echo "[Build] FUSE support: disabled (--no-fuse)"
fi

echo "[Build] Build type: ${BUILD_TYPE}"
echo ""

# Create build directory
mkdir -p build
cd build

# Configure
echo "[Build] Configuring..."
cmake .. \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    ${FUSE_FLAG} \
    ${EXTRA_FLAGS} \
    2>&1 | grep -E "^--|Configuring|Generating|Build files" || true

echo ""

# Build
NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
echo "[Build] Compiling (${NPROC} jobs)..."
make -j"${NPROC}" 2>&1 | grep -E "error|warning|Linking|Built|Copying" || true

cd ..

echo ""
echo "╔════════════════════════════════════════╗"
echo "║   Build Complete                       ║"
echo "╚════════════════════════════════════════╝"
echo ""

# Show what was built
if [ -f bramble ]; then
    SIZE=$(ls -lh bramble | awk '{print $5}')
    echo "  bramble          ${SIZE}"
fi
if [ -f build/bramble_tests ]; then
    SIZE=$(ls -lh build/bramble_tests | awk '{print $5}')
    echo "  bramble_tests    ${SIZE}"
fi
if [ -f build/bramble_bench ]; then
    SIZE=$(ls -lh build/bramble_bench | awk '{print $5}')
    echo "  bramble_bench    ${SIZE}"
fi

echo ""
echo "Run: ./bramble <firmware.uf2> [options]"
echo "Test: ./build/bramble_tests"
echo "Bench: ./build/bramble_bench"
