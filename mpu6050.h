#ifndef _mpu6050_
#define _mpu6050_

typedef struct MPU6050_STATE {
    uint8_t accel_res; uint16_t accel_res_val;                      // 0=> 16384, 1=>8192, 2=>4096, 3=>2048  
    uint8_t gyro_res;                                               // 0=> 131,   1=>65.5, 2=>32.8, 3=>16.4 

    int16_t accel_x_deviation, accel_y_deviation, accel_z_deviation;// accelerometer standard deviation
    int16_t gyro_x_deviation, gyro_y_deviation, gyro_z_deviation;   // gyroscope standard deviation

    int16_t accel_x_offset, accel_y_offset, accel_z_offset;         // accelerometer offset
    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;            // gyroscope offset
}MPU6050_STATE;

typedef struct MPU6050_DATA {
    int16_t accel_raw[3];               // RAW X - Y - Z Acceleration
    int16_t gyro_raw[3];                // RAW X - Y - Z Gyroscope Data
    int16_t temp_raw;                   // RAW Temperature

    int16_t accel_no_offset[3];
    int16_t gyro_no_offset[3];

    float accel_mod_no_gravity;         // accelerometer vecotr module sqrt(X^2 + Y^2 + Z^2) without gravity constant
    uint16_t gyro_mod;                  // gyroscope vecotr module sqrt(X^2 + Y^2 + Z^2)

    float accel_convert[3];             // converted acceleration measures
    float gyro_convert[3];              // converted gyroscope measures 

    float accel_no_gravity[3];          //user's data without offset and gravity constant

    float distance;                     // computed distance
    float theta_roll, theta_pitch, theta_yaw;      // theta angle
}MPU6050_DATA;

typedef struct MPU6050
{
    struct MPU6050_STATE mpu6050_state;
    struct MPU6050_DATA mpu6050_data;
}MPU6050;

typedef struct MPU6050_SELFTEST
{
    uint8_t STR_X, STR_Y, STR_Z;            //STR => SELFT-TEST-RESPONSE
    float FT_X, FT_Y, FT_Z;                 //FT => FACTORY TRIMMER
    uint8_t X_TEST, Y_TEST, Z_TEST, A_TEST; // TEST REGISTER
    float X_ERROR, Y_ERROR, Z_ERROR;        //Errors given in %
}MPU6050_SELFTEST;
#endif
