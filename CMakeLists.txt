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
        ${CMAKE_SOURCE_DIR}/lib/op-comms/intcomm.proto
    )

    set(FW_SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

    # Add includes
    include_directories(${FW_SOURCE_DIR})

    set(SOURCES
        ${FW_SOURCE_DIR}/main.c
        ${FW_SOURCE_DIR}/usb_descriptors.c
        ${FW_SOURCE_DIR}/hardware_configuration.c
        ${FW_SOURCE_DIR}/task_manager.c
        ${FW_SOURCE_DIR}/comms/proto_codec.c
        ${FW_SOURCE_DIR}/comms/command.c
        ${FW_SOURCE_DIR}/comms/packet.c
        ${FW_SOURCE_DIR}/controls/pid.c
        ${FW_SOURCE_DIR}/gimbal/gimbal_configuration.c
        ${FW_SOURCE_DIR}/gimbal/gimbal_engine.c
        ${FW_SOURCE_DIR}/gimbal/gimbal_state.c
        ${FW_SOURCE_DIR}/gimbal/gimbal.c
        ${FW_SOURCE_DIR}/sensors/as5600.c
        ${FW_SOURCE_DIR}/tasks/comm_process_task.c
        ${FW_SOURCE_DIR}/tasks/comm_rx_task.c
        ${FW_SOURCE_DIR}/tasks/comm_tx_task.c
        ${FW_SOURCE_DIR}/tasks/control_task.c
        ${FW_SOURCE_DIR}/tasks/stream_task.c
        ${FW_SOURCE_DIR}/tasks/sync_task.c
        ${FW_SOURCE_DIR}/tasks/usb_task.c
        ${FW_SOURCE_DIR}/utils/rotary_utils.c
    )

    # Add the main executable
    add_executable(${PROJECT_NAME}
        ${SOURCES}
    )

    target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    # Link required libraries to the project
    target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        pico_flash
        hardware_i2c
        hardware_pwm
        hardware_flash
        hardware_sync
        hardware_timer
        tinyusb_device
        tinyusb_board
        FreeRTOS-Kernel-Heap4
        proto
    )

    # Pico-specific configuration
    pico_enable_stdio_usb(${PROJECT_NAME} 0)
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

    # Generate protobuf headers for host tests
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)
    find_package(Nanopb REQUIRED)

    nanopb_generate_cpp(TARGET proto
        ${CMAKE_SOURCE_DIR}/lib/op-comms/intcomm.proto
    )

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
    add_executable(test_command
        ${TEST_SOURCE_DIR}/test_command.c
        ${TEST_SOURCE_DIR}/mocks/mock_as5600.c
        ${TEST_SOURCE_DIR}/mocks/mock_gimbal_configuration.c
        ${FW_SOURCE_DIR}/comms/command.c
        ${FW_SOURCE_DIR}/comms/packet.c
        ${FW_SOURCE_DIR}/comms/proto_codec.c
        ${FW_SOURCE_DIR}/gimbal/gimbal_state.c
        ${FW_SOURCE_DIR}/controls/pid.c
        ${FW_SOURCE_DIR}/utils/rotary_utils.c
    )

    target_include_directories(test_command PRIVATE
        ${FW_SOURCE_DIR}
        ${TEST_SOURCE_DIR}/mocks
    )

    target_link_libraries(test_command
        Unity
        proto
        m
    )

    add_test(NAME test_command COMMAND test_command)
else()
    message(WARNING "Unknown TARGET_GROUP: ${TARGET_GROUP}")
endif()
