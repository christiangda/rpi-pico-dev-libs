/**
 *
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "mpu6050_i2c.h"

#define MPU6050_ADDRESS 0x68
#define MPU6050_BAUD_RATE 400000
#define MPU6050_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define MPU6050_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#define MPU6050_ENABLE_PICOTOOL 1
#define MPU6050_CLOCK MPU6050_SET_CLOCK_INTERNAL

int main()
{
    stdio_init_all();

    // Initialize
    mpu6050_init();

    float accel_x, accel_y, accel_z;
    uint8_t id;

    id = mpu6050_get_id();

    while (1)
    {
        // mpu6050_get_acceleration(&accel_x, &accel_y, &accel_z);
        accel_x = mpu6050_get_acceleration_x();
        accel_y = mpu6050_get_acceleration_y();
        accel_z = mpu6050_get_acceleration_z();

        printf("Device ID: %d, Accelerations: X = %6.2f, Y = %6.2f, Z = %6.2f\n", id, accel_x, accel_y, accel_z);

        sleep_ms(500);
    }
}