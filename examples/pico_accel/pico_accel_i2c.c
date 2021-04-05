/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// #define I2C_PORT i2c0
// #define GPIO_SDA 4
// #define GPIO_SLC 5

// By default these devices are on bus address 0x68
static int accel_addr = 0x68;

// Tiny 128×32 is a OLED Display based on SSD1306 Display controller chip.
static int oled_addr = 0x3c;

// Initialise Accelerometer Function
static void accel_init()
{
  // Check to see if connection is correct
  sleep_ms(100);

  // Register 117 – Who Am I WHO_AM_I
  uint8_t reg = 0x75;
  uint8_t who[1];

  i2c_write_blocking(i2c_default, accel_addr, &reg, 1, true);
  i2c_read_blocking(i2c_default, accel_addr, who, 1, false);

  if (who[0] != 0x68)
  {
    while (1)
    {
      printf("Accelerator ID Not Correct - Check Connection!");
      sleep_ms(5000);
    }
  }
}

#ifdef i2c_default
static void mpu6050_reset()
{
  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that could be added here
  uint8_t buf[] = {0x6B, 0x00};
  i2c_write_blocking(i2c_default, accel_addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto incrementing
  // so we don't need to keep sending the register we want, just the first.

  uint8_t buffer[6];

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(i2c_default, accel_addr, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, accel_addr, buffer, 6, false);

  for (int i = 0; i < 3; i++)
  {
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
  }

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  i2c_write_blocking(i2c_default, accel_addr, &val, 1, true);
  i2c_read_blocking(i2c_default, accel_addr, buffer, 6, false); // False - finished with bus

  for (int i = 0; i < 3; i++)
  {
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    ;
  }

  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  i2c_write_blocking(i2c_default, accel_addr, &val, 1, true);
  i2c_read_blocking(i2c_default, accel_addr, buffer, 2, false); // False - finished with bus

  *temp = buffer[0] << 8 | buffer[1];
}
#endif

int main()
{
  stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/mpu6050_i2c example requires a board with I2C pins
  puts("Default I2C pins were not defined");
#else

  sleep_ms(10000);

  printf("Hello, MPU6050! Reading raw data from registers...\n");

  // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
  i2c_init(i2c_default, 400 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  mpu6050_reset();
  accel_init();

  int16_t acceleration[3], gyro[3], temp;

  while (1)
  {
    mpu6050_read_raw(acceleration, gyro, &temp);

    // These are the raw numbers from the chip, so will need tweaking to be really useful.
    // See the datasheet for more information
    printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
    printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    printf("Temp. = %f\n", (temp / 340.0) + 36.53);

    sleep_ms(10000);
  }

#endif
  return 0;
}
