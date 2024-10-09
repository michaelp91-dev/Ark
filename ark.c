#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "mpu6050.h"

// Define i2c connections + addresses
#define I2C_PORT i2c0
#define I2C_SDA 2
#define I2C_SCL 3

// registers
const uint8_t addr = 0x68;
const uint8_t reset_addr = 0x6B
const uint8_t gyro_config = 0x1B;
const uint8_t accel_config = 0x1C;
const uint8_t gyro_out = 0x43;
const uint8_t accel_out = 0x6B;

const float RAD_2_DEG = 57.29578; // [deg/rad]
const float TEMP_LSB_2_DEGREE = 340.0; // [bit/celsius]
const float TEMP_LSB_OFFSET = 12412.0;
float gyro_lsb_to_degsec, acc_lsb_to_g;
float gyro_x_offset, gyro_y_offset, gyro_z_offset;
float accel_x_offset, accel_y_offset, accel_z_offset;
float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
float angleAccX, angleAccY;
float angleX, angleY, angleZ;
uint64_t Told;
float filterGyroCoef;

//data
int16_t gyro_raw[3];
int16_t accel_raw[3];
int16_t temp_raw;

//helpers
write_data(uint8_t reg, uint8_t data) {
  uint8_t buf[] = {reg, data};
  i2c_write_blocking(i2c, addr, buf, 2, false);

#ifdef I2C_PORT
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {reset_addr, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(1000);
    printf("\r\nMPU6050 Setup Complete\r\n");
} 

static void set_gyro_config(int config_num) {
    switch (config_num) {
  case 0: // range = +- 250 deg/s
    gyro_lsb_to_degsec = 131.0;
    write_data(gyro_config, 0x00);
    break;
  case 1: // range = +- 500 deg/s
    gyro_lsb_to_degsec = 65.5;
    write_data(gyro_config, 0x08);
    break;
  case 2: // range = +- 1000 deg/s
    gyro_lsb_to_degsec = 32.8;
    write_data(gyro_config, 0x10);
    break;
  case 3: // range = +- 2000 deg/s
    gyro_lsb_to_degsec = 16.4;
    write_data(gyro_config, 0x18);
    break;
    }
}

static void set_accel_config(int config_num) {
    switch (config_num) {
    case 0: // range = +- 2 g
      acc_lsb_to_g = 16384.0;
      write_data(accel_config, 0x00);
      break;
    case 1: // range = +- 4 g
      acc_lsb_to_g = 8192.0;
      write_data(accel_config, 0x08);
      break;
    case 2: // range = +- 8 g
      acc_lsb_to_g = 4096.0;
      write_data(accel_config, 0x10);
      break;
    case 3: // range = +- 16 g
      acc_lsb_to_g = 2048.0;
      write_data(accel_config, 0x18);
      break;
      }
}

void mpu_calc_offset(bool is_calc_gyro, bool is_calc_acc)
{
    int32_t accel_X_offset, accel_Y_offset, accel_Z_offset = 0;
    int32_t gyro_X_offset, gyro_Y_offset, gyro_Z_offset = 0;
    uint8_t CALC_TIME = 120
    
    for(uint8_t i = 0; i < CALC_TIME; i++) // make 200 meaesurements and get average
    {
        mpu_read_raw();
        accel_X_offset += accel_raw[0];
        accel_Y_offset += accel_raw[1];
        accel_Z_offset += accel_raw[2] - accel_res_val;

        gyro_X_offset += gyro_raw[0];
        gyro_Y_offset += gyro_raw[1];
        gyro_Z_offset += gyro_raw[2];

        sleep_ms(1000); // wait for next measurement from mpu
    }
    if(is_calc_acc) {
        accel_x_offset = accel_X_offset / CALC_TIME; 
        accel_y_offset = accel_Y_offset / CALC_TIME; 
        accel_z_offset = accel_Z_offset / CALC_TIME;
    }
    if(is_gyro_calc) {
        gyro_x_offset = gyro_X_offset / CALC_TIME; 
        gyro_y_offset = gyro_Y_offset / CALC_TIME; 
        gyro_z_offset = gyro_Z_offset / CALC_TIME;
    }
}


void mpu_read()
{
    mpu_read_raw();

    accel_no_offset[0] = accel_raw[0] - accel_x_offset;
    accel_no_offset[1] = accel_raw[1] - accel_y_offset;
    accel_no_offset[2] = accel_raw[2] - accel_z_offset; 

    gyro_no_offset[0] = gyro_raw[0] - gyro_x_offset;
    gyro_no_offset[1] = gyro_raw[1] - gyro_y_offset;
    gyro_no_offset[2] = gyro_raw[2] - gyro_z_offset;    
}

static void mpu6050_read_raw() {
    uint8_t buffer[14]
    i2c_write_blocking(I2C_PORT, addr, accel_out, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 14, false);

    int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]
    for (int i = 0; i < 7; i++) {
      rawData[i] = rawData8[2 * i] << 8;
      rawData[i] |= rawData8[2 * i + 1];
    }
    accel_raw[0] = rawData[0];
    accel_raw[1] = rawData[1];
    accel_raw[2] = rawData[2];
    temp_raw = rawData[3];
    gyro_raw[0] = rawData[4];
    gyro_raw[1] = rawData[5];
    gyro_raw[2] = rawData[6];
}
#endif

int main()
{
    stdio_init_all();
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(i2c_default, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    mpu6050_reset();

    mpu_calc_offset(true, true)
    
    while (1) {
        sleep_ms(100);
    }
}




