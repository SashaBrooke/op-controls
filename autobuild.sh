#!/usr/bin/env bash
set -e  # Exit on any error

###########################################################################
# autobuild.sh
#
# This script cleans the existing build directory, configures CMake with a
# specified TARGET_GROUP, and builds the project.
#
# Usage:
#   ./autobuild.sh [--firmware|--test]
#
# Arguments:
#   --firmware   Build the firmware target (default if no argument given)
#   --test       Build the testing target
#
# Notes:
#   - Only one argument is allowed. Providing multiple arguments will fail.
#   - The build directory will always be cleaned and recreated.
#   - The project will be built in the "build/" directory.
###########################################################################

# --- Check number of arguments ---
if [ $# -gt 1 ]; then
    echo "Error: Only one argument allowed."
    echo "Usage: $0 [--firmware|--test]"
    exit 1
fi

# --- Determine TARGET_GROUP ---
if [ $# -eq 0 ]; then
    TARGET_GROUP="FIRMWARE"  # default
else
    case "$1" in
        --firmware)
            TARGET_GROUP="FIRMWARE"
            ;;
        --test)
            TARGET_GROUP="TESTING"
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--firmware|--test]"
            exit 1
            ;;
    esac
fi

# --- Determine build directory and marker file ---
BUILD_DIR="build"
MARKER_FILE="$BUILD_DIR/.build_type_${TARGET_GROUP}"

# --- Check if we need to clean ---
NEEDS_CLEAN=false
if [ -d "$BUILD_DIR" ]; then
    if [ -f "$MARKER_FILE" ]; then
        echo "Detected previous $TARGET_GROUP build. Skipping clean step."
    else
        echo "Build type has changed or is unknown. Cleaning build directory..."
        NEEDS_CLEAN=true
    fi
else
    echo "No existing build directory found."
    NEEDS_CLEAN=true
fi

# --- Clean and recreate build directory if needed ---
if [ "$NEEDS_CLEAN" = true ]; then
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
else
    echo "Reusing existing build directory."
fi

# --- Configure CMake ---
echo "Configuring CMake with TARGET_GROUP=$TARGET_GROUP..."
cmake -S . -B "$BUILD_DIR" -DTARGET_GROUP="$TARGET_GROUP"

# --- Build the project ---
echo "Building project..."
cmake --build "$BUILD_DIR"

# --- Create marker file ---
echo "Creating build type marker..."
rm -f "$BUILD_DIR/.build_type_"*  # Remove any old markers
touch "$MARKER_FILE"

echo "Build completed successfully!"
