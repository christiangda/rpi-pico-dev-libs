/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Christian González Di Antonio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

/*
 * This driver supports
 *
 * */


#ifndef _MPU6050_I2C_H
#define _MPU6050_I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

// i2c port to use for the i2c mpu6050 communication
#if !defined(MPU6050_I2C_PORT)
#    define MPU6050_I2C_PORT i2c_default
#endif

// Address to use for the i2c mpu6050 communication
#if !defined(MPU6050_ADDRESS)
#    define MPU6050_ADDRESS 0x68
#endif

// Frequency to use for the i2c mpu6050 communication
#if !defined(MPU6050_BAUD_RATE)
#    define MPU6050_BAUD_RATE 400000
#endif

// i2c SDA port to use for the i2c mpu6050 communication
#if !defined(MPU6050_SDA_PIN)
#    define MPU6050_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#endif

// i2c SCL port to use for the i2c mpu6050 communication
#if !defined(MPU6050_SCL_PIN)
#    define MPU6050_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#endif

// Expose pins to picotool
#if !defined(MPU6050_ENABLE_PICOTOOL)
#    define MPU6050_ENABLE_PICOTOOL 0
#endif

/**
 * @brief Use this to set the Clock
 * An internal 8MHz oscillator, gyroscope based clock, or external sources
 * can be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 */
typedef enum {
    MPU6050_SET_CLOCK_INTERNAL       = 0x00,
    MPU6050_SET_CLOCK_PLL_X_GYRO     = 0x01,
    MPU6050_SET_CLOCK_PLL_Y_GYRO     = 0x02,
    MPU6050_SET_CLOCK_PLL_Z_GYRO     = 0x03,
    MPU6050_SET_CLOCK_PLL_EXT_32KHZ  = 0x04,
    MPU6050_SET_CLOCK_PLL_EXT_19MHZ  = 0x05,
    MPU6050_SET_CLOCK_STOP_RESET     = 0x07,
} clock_sel;

// Clock selection
#if !defined(MPU6050_CLOCK)
#    define MPU6050_CLOCK MPU6050_SET_CLOCK_INTERNAL
#endif

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void mpu6050_init();
void mpu6050_reset();

// Register 104 – Signal Path Reset SIGNAL_PATH_RESET
void mpu6050_reset_accelerometer_path();
void mpu6050_reset_gyroscope_path();
void mpu6050_reset_temperature_path();

// getters
uint8_t mpu6050_get_id();

// setters
void mpu6050_set_clock(uint8_t);
void mpu6050_test_conn();


#ifdef __cplusplus
 }
#endif

#endif /* _MPU6050_I2C_H */
