# Define the executable
add_executable(acc
    main.c
    task_sensor.c
    task_dimmer.c
    task_alert.c
    task_brake.c
    task_oled.c
    # i2c_manager.c
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
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    u8g2
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
