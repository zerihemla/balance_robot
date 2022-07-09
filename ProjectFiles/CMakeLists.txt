add_executable(balance_robot
        main.c
        led.c
        rtosUtility.c
        logInterface.c
        tester.c
        mpu6050.c
        i2cInterface.c
)

target_link_libraries(balance_robot pico_stdlib freertos hardware_i2c)

pico_enable_stdio_usb(balance_robot 1)
pico_enable_stdio_uart(balance_robot 1)

pico_add_extra_outputs(balance_robot)
