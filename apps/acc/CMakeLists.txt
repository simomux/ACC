set(MICROROS_SDK ${CMAKE_SOURCE_DIR}/micro_ros_raspberrypi_pico_sdk)

# Import the prebuilt micro-ROS static library
add_library(microros STATIC IMPORTED GLOBAL)
set_target_properties(microros PROPERTIES
    IMPORTED_LOCATION ${MICROROS_SDK}/libmicroros/libmicroros.a
)

# Define the executable
# pico_uart_transport.c is part of the micro-ROS SDK and must be compiled with
# the same include paths; it is not a library, just a source file.
add_executable(acc
    main.c
    task_sensor.c
    task_dimmer.c
    task_alert.c
    task_brake.c
    task_oled.c
    task_microros.c
    # i2c_manager.c
)

# micro-ROS headers
target_include_directories(acc PRIVATE
    ${MICROROS_SDK}/libmicroros/include
)

# Disable CRLF translation — micro-ROS uses binary framing, CRLF corrupts it
target_compile_definitions(acc PRIVATE
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

# Aggiungi la libreria u8g2
add_subdirectory(${CMAKE_SOURCE_DIR}/3rdparty/u8g2 ${CMAKE_BINARY_DIR}/3rdparty/u8g2_build)

target_link_libraries(acc
    pico_stdlib
    pico_async_context_freertos
    hardware_adc
    hardware_timer
    hardware_pwm
    hardware_i2c
    hardware_uart
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    u8g2
    microros
)

# Configure Pico IO settings
pico_enable_stdio_usb(acc 1)
pico_enable_stdio_uart(acc 0)

# Set output directory
set_target_properties(acc PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(acc)
