if (TARGET tinyusb_device)

  add_executable(pico_accel_i2c
          pico_accel_i2c.c
          )

  # Pull in our (to be renamed) simple get you started dependencies
  target_link_libraries(pico_accel_i2c pico_stdlib mpu6050)

  # enable usb output, disable uart output
  pico_enable_stdio_usb(pico_accel_i2c 1)
  pico_enable_stdio_uart(pico_accel_i2c 0)

  # create map/bin/hex file etc.
  pico_add_extra_outputs(pico_accel_i2c)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building pico_accel_i2c because TinyUSB submodule is not initialized in the SDK")
endif()