# OhPossum Control System

## Overview

This project contains the firmware for the OhPossum control system. The firmware supports dual-axis movement (pan/tilt rotation) and control commands using *Protobuf messages*.

## Setting up the project

To set up this project, the following are required:
- Docker
- Visual Studio Code (VS Code) and the Dev Container Extension

To access the dev container:
- Open this project in VS Code.
- Open VS Code Shell Command and select "Open Folder in Container"

**Note:** To open VS Code Shell Command you use `control+shift+p` on Windows and `cmd+shift+p` on Mac OS.

## Building the project

A helper script has been provided to automate the build process. This ensures the correct build directory is created and the correct compiler is used for either firmware or tests.

**Example autobuild:**
- Make the script executable (only have to do once)
    - `chmod +x autobuild.sh`
- Run the script from the root (default) directory of the project
    - `./autobuild --firmware` (firmware)
    - `./autobuild --test` (tests)

For more information, see the documentation in the `autobuild.sh` script.

**Note:** CMake can still be used manually to build the project. However, **you** must ensure that the build directory has been cleared if changing from building firmware to tests (or vice versa).

**Example manual CMake:**
- Make or clear `build` directory
    - `mkdir build` (if not already present)
    - `rm -rf build` (if already present)
- Create the new build directory
    - `cd build`
- Configure the project
    - `cmake ..` OR `cmake .. -DTARGET_GROUP=FIRMWARE` (firmware)
    - `cmake .. -DTARGET_GROUP=TESTING` (tests)
- Build the project
    - `make` (optional `make -j8`)

## Running the tests

Assuming the tests are built (as described in the steps above):

- Navigate to the build directory (if not already there)
    - `cd build`
- Run the tests
    - `ctest` (optionally `ctest --output-on-error -v` for more detail)
