#!/bin/bash

# Compilation script for hardware_adapter C code and system_managerCPP

set -e  # Exit on error

# Get the script directory and workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR"

echo "=========================================="
echo "Compilation Script"
echo "=========================================="
echo "Source directory: $SRC_DIR"
echo ""

CC=/usr/bin/gcc
CXX=/usr/bin/g++
export CC CXX
echo "Compiler: $CC / $CXX"
echo ""

# Step 1: Clean and compile hardware_adapter C code
echo "=========================================="
echo "Step 1: Cleaning and compiling hardware_adapter C code"
echo "=========================================="
HARDWARE_ADAPTER_DIR="$SRC_DIR/hardware_adapter"

if [ ! -d "$HARDWARE_ADAPTER_DIR" ]; then
    echo "Error: hardware_adapter directory not found at $HARDWARE_ADAPTER_DIR"
    exit 1
fi

cd "$HARDWARE_ADAPTER_DIR"
echo "Cleaning hardware_adapter in: $HARDWARE_ADAPTER_DIR"
if make clean 2>/dev/null; then
    echo "  ✓ Cleaned previous build"
else
    echo "  (No previous build to clean)"
fi

echo "Building hardware_adapter in: $HARDWARE_ADAPTER_DIR"
# Build hardware_adapter in debug mode with 3 parallel jobs
make -j3 debug

if [ $? -eq 0 ]; then
    echo "  ✓ hardware_adapter compilation successful"
else
    echo "  ✗ hardware_adapter compilation failed"
    exit 1
fi

echo ""

# Step 2: Clean and compile system_managerCPP
echo "=========================================="
echo "Step 2: Cleaning and compiling system_managerCPP"
echo "=========================================="
SYSTEM_MANAGER_DIR="$SRC_DIR/system_managerCPP"

if [ ! -d "$SYSTEM_MANAGER_DIR" ]; then
    echo "Error: system_managerCPP directory not found at $SYSTEM_MANAGER_DIR"
    exit 1
fi

cd "$SYSTEM_MANAGER_DIR"
echo "Cleaning system_managerCPP in: $SYSTEM_MANAGER_DIR"

# Clean previous build
if [ -f "clean_build.sh" ]; then
    chmod +x clean_build.sh
    ./clean_build.sh
else
    # Fallback: remove build directory if clean script doesn't exist
    if [ -d "build" ]; then
        rm -rf build
        echo "  ✓ Removed build directory"
    else
        echo "  (No previous build to clean)"
    fi
fi

echo "Building system_managerCPP in: $SYSTEM_MANAGER_DIR"

# Check if build.sh exists and use it, otherwise use CMake directly
if [ -f "build.sh" ]; then
    echo "Using build.sh script..."
    chmod +x build.sh
    ./build.sh Debug
else
    echo "Using CMake directly..."
    mkdir -p build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug ..
    # Build with 3 parallel jobs
    make -j3
    cd ..
fi

if [ $? -eq 0 ]; then
    echo "  ✓ system_managerCPP compilation successful"
else
    echo "  ✗ system_managerCPP compilation failed"
    exit 1
fi

echo ""
echo "=========================================="
echo "Compilation complete!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✓ hardware_adapter cleaned and compiled"
echo "  ✓ system_managerCPP cleaned and compiled"
echo ""
