

#ifndef _MPU6050_I2C_H__
#define _MPU6050_I2C_H__

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


//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void mpu6050_reset_signals();
void mpu6050_test_conn();
void mpu6050_reset();
void mpu6050_init();
void mpu6050_write();


#ifdef __cplusplus
 }
#endif

#endif /* _MPU6050_I2C_H__ */
