#include <stdio.h>

#include "mpu6050_i2c.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"


#define W_DATA       0xD0
#define R_DATA       0xD1
#define WAIT_TIME_MS 100


#define RESET_VALUE             0x00
#define PWR_MGMT_1_RESET_VALUE  0x80 // 0x40 in the manual?
#define WHO_AM_I_RESET_VALUE    0x68


#define SMPLRT_DIV     0x19

#define CONFIG         0x1A
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C

#define ACCEL_XOUT_H   0x3B
#define ACCEL_XOUT_L   0x3C
#define ACCEL_YOUT_H   0x3D
#define ACCEL_YOUT_L   0x3E
#define ACCEL_ZOUT_H   0x3F
#define ACCEL_ZOUT_L   0x40

#define TEMP_OUT_H     0x41
#define TEMP_OUT_L     0x42

#define GYRO_XOUT_H    0x43
#define GYRO_XOUT_L    0x44
#define GYRO_YOUT_H    0x45
#define GYRO_YOUT_L    0x46
#define GYRO_ZOUT_H    0x47
#define GYRO_ZOUT_L    0x48

#define SIGNAL_PATH_RESET 0x68
#define USER_CTRL         0x6A

#define PWR_MGMT_1     0x6B
#define PWR_MGMT_2     0x6C
#define WHO_AM_I       0x75


void mpu6050_test_conn(){

}

void mpu6050_reset_signals(){
    uint8_t buf[] = {SIGNAL_PATH_RESET, 0x7};
    i2c_write_blocking(MPU6050_I2C_PORT,MPU6050_ADDRESS,&buf,2,false);
}

void mpu6050_reset(){

    uint8_t buf[] = {PWR_MGMT_1, PWR_MGMT_1_RESET_VALUE};
    uint8_t status[1];

    i2c_write_blocking(MPU6050_I2C_PORT,MPU6050_ADDRESS,&buf,2,false);
    sleep_ms(WAIT_TIME_MS);
    i2c_read_blocking(i2c_default, accel_addr, status, 1, false);

    if (status[0] != 0x00)
    {
        while (1)
        {
            printf("Resetting device");
            sleep_ms(WAIT_TIME_MS);
        }
    }
    // resetting giro, accel and temp sensor
    mpu6050_reset_signals()
    sleep_ms(WAIT_TIME_MS);
}
