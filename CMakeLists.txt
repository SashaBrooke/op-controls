# Configure firmware by default
if(NOT DEFINED TARGET_GROUP)
    set(TARGET_GROUP "FIRMWARE")
    message(STATUS "TARGET_GROUP not set, defaulting to FIRMWARE")
endif()

if(TARGET_GROUP STREQUAL "FIRMWARE")
    message(STATUS "Configuring for firmware build")

    # Set minimum required version of CMake
    cmake_minimum_required(VERSION 3.12)

    # Include build functions from Pico SDK
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    # Locate pre-installed picotool
    find_package(picotool REQUIRED)

    # Set name of project and C/C++ standards
    project(ohpossum-controls C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Initialize the Pico SDK
    pico_sdk_init()

    # # Generate protobuf headers
    # set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)
    # find_package(Nanopb REQUIRED)
    # nanopb_generate_cpp(TARGET proto 
    #     ${CMAKE_SOURCE_DIR}/lib/op-comms/internal.proto
    # )

    # Add includes
    include_directories(src)

    set(SOURCES
        src/gimbal.c
        src/as5600.c
        src/pid.c
        src/command.c
        src/gimbal_configuration.c
        src/rotary_utils.c
        src/pinout.c
    )

    # Optional: Automatically collect all .c files in the src/ directory
    # file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.c)

    # Add the main executable
    add_executable(${PROJECT_NAME}
        ${SOURCES}
    )

    # Link required libraries to the project
    target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_flash
        hardware_sync
        hardware_timer
    )

    # Pico-specific configuration
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)

    # moves everything except the .uf2 (only output used) into /bin
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    # generate .uf2 .elf .bin
    pico_add_extra_outputs(${PROJECT_NAME})

elseif(TARGET_GROUP STREQUAL "TESTING")
    message(STATUS "Configuring for test build")

    # Set minimum required version of CMake
    cmake_minimum_required(VERSION 3.12)

    project(ohpossum-controls-tests C)
    set(CMAKE_C_STANDARD 11)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    set(UNITY_DIR ${CMAKE_SOURCE_DIR}/lib/unity/src)
    set(FW_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)
    set(TEST_SOURCE_DIR ${CMAKE_SOURCE_DIR}/test)

    # --- Unity library ---
    add_library(Unity STATIC
        ${UNITY_DIR}/unity.c
    )

    target_include_directories(Unity PUBLIC
        ${UNITY_DIR}
    )

    enable_testing()

    # --- Unit tests ---
    # add_executable(test_servo_utils
    #     ${TEST_SOURCE_DIR}/test_file.c
    #     ${FW_SOURCE_DIR}/file.c
    # )

    # target_include_directories(test_file PRIVATE
    #     ${FW_SOURCE_DIR}
    # )

    # target_link_libraries(test_file Unity)
    # add_test(NAME file_test COMMAND test_file)
else()
    message(WARNING "Unknown TARGET_GROUP: ${TARGET_GROUP}")
endif()
