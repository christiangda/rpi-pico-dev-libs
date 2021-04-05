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
#include <stdio.h>

#include "mpu6050_i2c.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#if defined(MPU6050_ENABLE_PICOTOOL) && MPU6050_ENABLE_PICOTOOL == 1
#include "pico/binary_info.h"
#endif


#define W_DATA       0xD0
#define R_DATA       0xD1
#define WAIT_TIME_MS 100

// Register reset values
typedef enum
{
    RESET_VALUE               = 0x00,
    PWR_MGMT_1_RESET_VALUE    = 0x00, // docs 0x40??
    WHO_AM_I_RESET_VALUE      = 0x68,

    // Register 104 – Signal Path Reset SIGNAL_PATH_RESET
    GYRO_RESET_VALUE          = 0x04,
    ACCEL_RESET_VALUE         = 0x02,
    TEMP_RESET_VALUE          = 0x01,
} reg_reset_values;

// Register Map
typedef enum
{
    SELF_TEST_X       = 0x0D, // R/W
    SELF_TEST_Y       = 0x0E, // R/W
    SELF_TEST_Z       = 0x0F, // R/W
    SELF_TEST_A       = 0x10, // R/W

    SMPLRT_DIV        = 0x19, // R/W

    CONFIG            = 0x1A, // R/W
    GYRO_CONFIG       = 0x1B, // R/W
    ACCEL_CONFIG      = 0x1C, // R/W

    FIFO_EN           = 0x23, // R/W

    I2C_MST_CTRL      = 0x24, // R/W
    I2C_SLV0_ADDR     = 0x25, // R/W
    I2C_SLV0_REG      = 0x26, // R/W
    I2C_SLV0_CTRL     = 0x27, // R/W
    I2C_SLV1_ADDR     = 0x28, // R/W
    I2C_SLV1_REG      = 0x29, // R/W
    I2C_SLV1_CTRL     = 0x2A, // R/W
    I2C_SLV2_ADDR     = 0x2B, // R/W
    I2C_SLV2_REG      = 0x2C, // R/W
    I2C_SLV2_CTRL     = 0x2D, // R/W
    I2C_SLV3_ADDR     = 0x2E, // R/W
    I2C_SLV3_REG      = 0x2F, // R/W
    I2C_SLV3_CTRL     = 0x30, // R/W
    I2C_SLV4_ADDR     = 0x31, // R/W
    I2C_SLV4_REG      = 0x32, // R/W
    I2C_SLV4_DO       = 0x33, // R/W
    I2C_SLV4_CTRL     = 0x34, // R/W
    I2C_SLV4_DI       = 0x35, // R
    I2C_MST_STATUS    = 0x36, // R

    INT_PIN_CFG       = 0x37, // R/W
    INT_ENABLE        = 0x38, // R/W
    INT_STATUS        = 0x3A, // R

    ACCEL_XOUT_H      = 0x3B, // R
    ACCEL_XOUT_L      = 0x3C, // R
    ACCEL_YOUT_H      = 0x3D, // R
    ACCEL_YOUT_L      = 0x3E, // R
    ACCEL_ZOUT_H      = 0x3F, // R
    ACCEL_ZOUT_L      = 0x40, // R

    TEMP_OUT_H        = 0x41, // R
    TEMP_OUT_L        = 0x42, // R

    GYRO_XOUT_H       = 0x43, // R
    GYRO_XOUT_L       = 0x44, // R
    GYRO_YOUT_H       = 0x45, // R
    GYRO_YOUT_L       = 0x46, // R
    GYRO_ZOUT_H       = 0x47, // R
    GYRO_ZOUT_L       = 0x48, // R

    EXT_SENS_DATA_00  = 0x49, // R
    EXT_SENS_DATA_01  = 0x4A, // R
    EXT_SENS_DATA_02  = 0x4B, // R
    EXT_SENS_DATA_03  = 0x4C, // R
    EXT_SENS_DATA_04  = 0x4D, // R
    EXT_SENS_DATA_05  = 0x4E, // R
    EXT_SENS_DATA_06  = 0x4F, // R
    EXT_SENS_DATA_07  = 0x50, // R
    EXT_SENS_DATA_08  = 0x51, // R
    EXT_SENS_DATA_09  = 0x52, // R
    EXT_SENS_DATA_10  = 0x53, // R
    EXT_SENS_DATA_11  = 0x54, // R
    EXT_SENS_DATA_12  = 0x55, // R
    EXT_SENS_DATA_13  = 0x56, // R
    EXT_SENS_DATA_14  = 0x57, // R
    EXT_SENS_DATA_15  = 0x58, // R
    EXT_SENS_DATA_16  = 0x59, // R
    EXT_SENS_DATA_17  = 0x5A, // R
    EXT_SENS_DATA_18  = 0x5B, // R
    EXT_SENS_DATA_19  = 0x5C, // R
    EXT_SENS_DATA_20  = 0x5D, // R
    EXT_SENS_DATA_21  = 0x5E, // R
    EXT_SENS_DATA_22  = 0x5F, // R
    EXT_SENS_DATA_23  = 0x60, // R

    I2C_SLV0_DO       = 0x63, // R/W
    I2C_SLV1_DO       = 0x64, // R/W
    I2C_SLV2_DO       = 0x65, // R/W
    I2C_SLV3_DO       = 0x66, // R/W

    I2C_MST_DELAY_CTRL= 0x67, // R/W

    SIGNAL_PATH_RESET = 0x68, // W

    USER_CTRL         = 0x6A, // R/W

    PWR_MGMT_1        = 0x6B, // R/W
    PWR_MGMT_2        = 0x6C, // R/W

    FIFO_COUNTH       = 0x72, // R/W
    FIFO_COUNTL       = 0x73, // R/W
    FIFO_R_W          = 0x74, // R/W

    WHO_AM_I          = 0x75, // R
} reg_map;


/**
 * @brief Write a single bit from an 8-bit device register.
 * @param reg Register reg to write to
 * @param value Container for single byte value
 */
static void i2c_write_reg(uint8_t reg, uint8_t value){
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDRESS, buf, 2, false);
}

/**
 * @brief Read a single bit from an 8-bit device register.
 * @param reg Register reg to read from
 * @param value Container for single byte value
 * @return Status of read operation (true = success)
 */
static uint8_t i2c_read_reg(uint8_t reg, uint8_t *value){
    int ret;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDRESS, &reg, 1, true);
    ret = i2c_read_blocking(MPU6050_I2C_PORT, MPU6050_ADDRESS, value, 1, false);

    return ret >0; true; false;
}

static uint8_t i2c_read_regs(uint8_t reg, uint8_t *values, size_t len){
    int ret;
    i2c_write_blocking(MPU6050_I2C_PORT, MPU6050_ADDRESS, &reg, 1, true);
    ret = i2c_read_blocking(MPU6050_I2C_PORT, MPU6050_ADDRESS, values, len, false);

    return ret >0; true; false;
}

/**
 * @brief Register 104 – Signal Path Reset
 *  Used to reset the analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors.
 *  The reset will revert the signal path analog to digital converters and filters to their power up configurations.
 *
 * @param data
 */
static void signal_path_write(uint8_t data){
    i2c_write_reg(SIGNAL_PATH_RESET, data);
}

void mpu6050_init(){
    i2c_init(MPU6050_I2C_PORT, MPU6050_BAUD_RATE);

    gpio_set_function(MPU6050_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SDA_PIN);

    gpio_set_function(MPU6050_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MPU6050_SCL_PIN);

#if defined(MPU6050_ENABLE_PICOTOOL) && MPU6050_ENABLE_PICOTOOL == 1
    bi_decl(bi_2pins_with_func(MPU6050_SDA_PIN, MPU6050_SCL_PIN, GPIO_FUNC_I2C));
#endif

    // Full reset
    mpu6050_reset();

    // check id
    uint8_t id;
    id = mpu6050_get_id();
    if(id != MPU6050_ADDRESS ){
        while (1)
        {
            printf("MPU-6050 id is not correct, please check the connection!");
            sleep_ms(WAIT_TIME_MS *10 );
        }
    }
}

/**
 * @brief Resets the accelerometer analog and digital signal paths.
 *
 */
void mpu6050_reset_accelerometer_path(){
    signal_path_write(ACCEL_RESET_VALUE);
}

/**
 * @brief Resets the gyroscope analog and digital signal paths.
 *
 */
void mpu6050_reset_gyroscope_path(){
    signal_path_write(GYRO_RESET_VALUE);
}

/**
 * @brief Resets the temperature sensor analog and digital signal paths.
 *
 */
void mpu6050_reset_temperature_path(){
    signal_path_write(TEMP_RESET_VALUE);
}

// getters
/**
 * @brief
 *
 * @return uint8_t
 */
uint8_t mpu6050_get_id(){
    uint8_t value;
    i2c_read_reg(WHO_AM_I, &value);
    return value;
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param z
 */
void mpu6050_get_acceleration(float *x, float *y, float *z){
    uint8_t data[6];
    int16_t accel_x, accel_y, accel_z;

    // reading the 6 bytes (registers) at the same time
    // ACCEL_XOUT[15:8] --> data[0]
    // ACCEL_XOUT[7:0]  --> data[1]
    // ACCEL_YOUT[15:8] --> data[2]
    // ACCEL_YOUT[7:0]  --> data[3]
    // ACCEL_ZOUT[15:8] --> data[4]
    // ACCEL_ZOUT[7:0]  --> data[5]
    i2c_read_regs(ACCEL_XOUT_H, data, 6);

    accel_x = (((int16_t)data[0]) << 8) | data[1];
    accel_y = (((int16_t)data[2]) << 8) | data[3];
    accel_z = (((int16_t)data[4]) << 8) | data[5];

    *x = (float)accel_x / 100.0;
    *y = (float)accel_y / 100.0;
    *z = (float)accel_z / 100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_acceleration_x(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // ACCEL_XOUT[15:8] --> data[0]
    // ACCEL_XOUT[7:0]  --> data[1]
    i2c_read_regs(ACCEL_XOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_acceleration_y(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // ACCEL_YOUT[15:8] --> data[0]
    // ACCEL_YOUT[7:0]  --> data[1]
    i2c_read_regs(ACCEL_YOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_acceleration_z(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // ACCEL_ZOUT[15:8] --> data[0]
    // ACCEL_ZOUT[7:0]  --> data[1]
    i2c_read_regs(ACCEL_ZOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param z
 */
void mpu6050_get_rotation(float *x, float *y, float *z){
    uint8_t data[6];
    int16_t accel_x, accel_y, accel_z;

    // reading the 6 bytes (registers) at the same time
    // GYRO_XOUT[15:8] --> data[0]
    // GYRO_XOUT[7:0]  --> data[1]
    // GYRO_YOUT[15:8] --> data[2]
    // GYRO_YOUT[7:0]  --> data[3]
    // GYRO_ZOUT[15:8] --> data[4]
    // GYRO_ZOUT[7:0]  --> data[5]
    i2c_read_regs(GYRO_XOUT_H, data, 6);

    accel_x = (((int16_t)data[0]) << 8) | data[1];
    accel_y = (((int16_t)data[2]) << 8) | data[3];
    accel_z = (((int16_t)data[4]) << 8) | data[5];

    *x = (float)accel_x / 100.0;
    *y = (float)accel_y / 100.0;
    *z = (float)accel_z / 100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_rotation_x(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // GYRO_XOUT[15:8] --> data[0]
    // GYRO_XOUT[7:0]  --> data[1]
    i2c_read_regs(GYRO_XOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_rotation_y(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // GYRO_YOUT[15:8] --> data[0]
    // GYRO_YOUT[7:0]  --> data[1]
    i2c_read_regs(GYRO_YOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief
 *
 * @return int16_t
 */
float mpu6050_get_rotation_z(){
    uint8_t data[2];

    // reading the 2 bytes (registers) at the same time
    // GYRO_ZOUT[15:8] --> data[0]
    // GYRO_ZOUT[7:0]  --> data[1]
    i2c_read_regs(GYRO_ZOUT_H, data, 2);

    return (float)((((int16_t)data[0]) << 8) | data[1])/100.0;
}

/**
 * @brief The most recent temperature sensor measurement.
 * Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
 * @return float
 */
float mpu6050_get_temperature(){
    uint8_t data[2];
    uint8_t temp;

    // reading the 2 bytes (registers) at the same time
    // TEMP_OUT[15:8] --> data[0]
    // TEMP_OUT[7:0]  --> data[1]
    i2c_read_regs(TEMP_OUT_H, data, 2);

    temp = (((int16_t)data[0]) << 8) | data[1];

    return (temp/340.0) + 36.53;
}

// setters
void mpu6050_set_clock(uint8_t source){
    i2c_write_reg(PWR_MGMT_1, MPU6050_CLOCK);
}

void mpu6050_reset(){

    // DEVICE_RESET (register 107)
    i2c_write_reg(PWR_MGMT_1, PWR_MGMT_1_RESET_VALUE);
    sleep_ms(WAIT_TIME_MS);

    //SIGNAL_PATH_RESET (register 104)
    // data = 00000111 (100 + 010 + 001) = 0x07 = 7
    signal_path_write(GYRO_RESET_VALUE + ACCEL_RESET_VALUE + TEMP_RESET_VALUE);
    sleep_ms(WAIT_TIME_MS);
}

