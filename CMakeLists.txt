# Configure firmware by default
if(NOT DEFINED TARGET_GROUP)
    set(TARGET_GROUP "FIRMWARE")
    message(STATUS "TARGET_GROUP not set, defaulting to FIRMWARE")
endif()

if(TARGET_GROUP STREQUAL "FIRMWARE")
    message(STATUS "Configuring for firmware build")

    # Set minimum required version of CMake
    cmake_minimum_required(VERSION 3.12)

    # Set the target board and platform
    set(PICO_BOARD pico_w)
    set(PICO_PLATFORM rp2040)

    # Set any variables required for importing libraries
    SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

    # Import libraries
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
    include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

    # Locate pre-installed picotool
    find_package(picotool REQUIRED)

    # Set name of project and C/C++ standards
    project(ohpossum-controls C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    # Initialize the Pico SDK
    pico_sdk_init()

    # Generate protobuf headers
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)
    find_package(Nanopb REQUIRED)
    
    nanopb_generate_cpp(TARGET proto
        ${CMAKE_SOURCE_DIR}/lib/op-comms/extcomm.proto
    )

    # Add includes
    include_directories(src)

    set(SOURCES
        src/main.c
        src/hardware_configuration.c
        src/task_manager.c
        src/comms/command.c
        src/comms/packet.c
        src/controls/pid.c
        src/gimbal/gimbal_configuration.c
        src/gimbal/gimbal_engine.c
        src/gimbal/gimbal_state.c
        src/gimbal/gimbal.c
        src/sensors/as5600.c
        src/tasks/comm_process_task.c
        src/tasks/comm_rx_task.c
        src/tasks/comm_tx_task.c
        src/tasks/control_task.c
        src/tasks/stream_task.c
        src/utils/rotary_utils.c
    )

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
        FreeRTOS-Kernel-Heap4
        proto
    )

    # Pico-specific configuration
    pico_enable_stdio_usb(${PROJECT_NAME} 1)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)

    # Moves everything except the .uf2 (only output used) into /bin
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )

    # Generate .uf2 .elf .bin
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
