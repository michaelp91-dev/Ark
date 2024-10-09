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


// Helpers
void i2c_write_reg(uint8_t i2c_address, uint8_t reg, uint8_t data)
{
    uint8_t tab[] = {reg, data};
    i2c_write_blocking(i2c1, i2c_address, tab, sizeof(tab)/sizeof(tab[0]), false);
}

// registers
MPU6050_REG mpu6050_reg = {
    .address = 0x68,        //device address
    .who_i_am_add = 0x75,
    .reset_add = 0x6B,      //reset address
    .accel_add = 0x3B,      //accelerator data address register
    .gyro_add = 0x43,       //gryoscope data address register
    .temp_add = 0x41,       //temperature data address register
    .acc_config = 0x1C,     //accelerometer resolution config register and calibration
    .gyro_config = 0x1B,    // gyroscope resolution config register and calibration
    .gyro_res = 0x1B,       // gyroscope resolution config register and calibration
    .XA_TEST = 0x0D,        //XA_TEST and XG_test register
    .YA_TEST = 0x0E,        //YA_TEST and YG_test register
    .ZA_TEST = 0x0F,        //ZA_TEST and ZG_test register
    .A_TEST = 0x10,         //second accelerometer test register XA_TEST[1:0]
    .config = 0x1A,         //gyroscope DLPF_CFG set register
    .SMPLRT_DIV = 0x19,     //sample rate divider
    .FIFO_EN = 0x23,        // fifo enable
    .FIFO_COUNTER_H = 0x72, // fifo counter high[15 : 8]
    .FIFO_COUNTER_L = 0x73, // fifo counter high[7 : 0]
    .INT_ENABLE = 0x38,     //  interrupt register
    .USER_CTRL = 0x6A       // user control
};

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {reset_addr, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(1000);
    printf("\r\nMPU6050 Setup Complete\r\n");
} 

void mpu_set_sample_rate(uint8_t divider)
{
    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.config, 0b00000110); // => set DLPF as 1kHz

    switch(divider)
    {
        case 1:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000001);
        break;

        case 2:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000010);
        break;

        case 4:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00000100);
        break;

        case 8:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00001000);
        break;

        case 16:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00010000);
        break;
        
        case 32:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b00100000);
        break;

        case 64:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b01000000);
        break;

        case 128:
            i2c_write_reg(mpu6050_reg.address, mpu6050_reg.SMPLRT_DIV, 0b10000000);
        break;
    }  
}

void mpu_setresolution(uint8_t gy_res, uint8_t ac_res)
{
    uint8_t check, resolution = 0;
    uint8_t res_index = 0;
    uint16_t res_value = 0;
 
    //GYROSCOPE RESOLUTION
    switch(gy_res)
    {
        case 0: //+- 250
            resolution = 0b00000000;
            res_index = 0;
            break;
        case 1: //+- 500
            resolution = 0b00001000;
            res_index = 1;
            break;
        case 2: //+- 1000
            resolution = 0b00010000;
            res_index = 2;
            break;
        case 3: //+- 2000
            resolution = 0b00011000;
            res_index = 3;
            break;
    }
    i2c_write_reg(mpu6050_reg.address, mpu6050_reg.gyro_res, resolution);
    gyro_res = res_index;

    //ACCELEROMETER RESOLUTION
    switch(ac_res)
    {
        case 0: //+- 2
            resolution = 0b00000000;
            res_index = 0;
            res_value = 16384;
            break;
        case 1: //+- 4
            resolution = 0b00001000;
            res_index = 1;
            res_value = 8192;
            break;
        case 2: //+- 8
            resolution = 0b00010000;
            res_index = 2;
            res_value = 4096;
            break;
        case 3: //+- 16
            resolution = 0b00011000;
            res_index = 3;
            res_value = 2048;
            break;
    }
    i2c_write_reg(mpu6050_reg.address,mpu6050_reg.acc_config ,resolution);
    accel_res = res_index;
    accel_res_val = res_value;
}

void mpu_calc_offset(bool is_calc_gyro, bool is_calc_acc)
{
    int32_t accel_X_offset, accel_Y_offset, accel_Z_offset = 0;
    int32_t gyro_X_offset, gyro_Y_offset, gyro_Z_offset = 0;
    uint8_t CALC_TIME = 120
    
    for(uint8_t i = 0; i < CALC_TIME; i++) // make 200 meaesurements and get average
    {
        mpu_read_raw(mpu6050);
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

void mpu_get_statistic()
{
    int16_t acc_x[200], acc_y[200], acc_z[200];
    int16_t gyro_x[200], gyro_y[200], gyro_z[200];
    int16_t var_acc_x, var_acc_y, var_acc_z, var_gyro_x, var_gyro_y, var_gyro_z;


    for(uint8_t i; i < 200; i++) // measure output 50 times
    {
        mpu_read(mpu6050);
        acc_x[i] = accel_no_offset[0];  
        acc_y[i] = accel_no_offset[1]; 
        acc_z[i] = accel_no_offset[2];
        gyro_x[i] = gyro_no_offset[0];  
        gyro_y[i] = gyro_no_offset[1];  
        gyro_z[i] = gyro_no_offset[2];
        sleep_ms(20); // wait fornext measurement
    }

    var_acc_x = get_variance(acc_x, 200); var_acc_y = get_variance(acc_y, 200); var_acc_z = get_variance(acc_z, 200);
    var_gyro_x = get_variance(gyro_x, 200); var_gyro_y= get_variance(gyro_y, 200); var_gyro_z = get_variance(gyro_z, 200);

    accel_x_deviation = var_acc_x; 
    accel_y_deviation = var_acc_y; 
    accel_z_deviation = var_acc_z; 
    gyro_x_deviation = var_gyro_x; 
    gyro_y_deviation = var_gyro_y; 
    gyro_z_deviation = var_gyro_z;
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
    
    mpu_convert(mpu6050); 
    mpu_remove_gravity(mpu6050);
    mpu_get_theta(mpu6050);
}

static void mpu6050_read_raw() {
    uint8_t buffer[6];

    // Start reading acceleration registers
    i2c_write_blocking(I2C_PORT, mpu6050_reg.address, &mpu6050_reg.accel_add, 1, true);
    i2c_read_blocking(I2C_PORT, mpu6050_reg.address, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel_raw[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // gyro data from reg
    i2c_write_blocking(I2C_PORT, mpu6050_reg.address, &mpu6050_reg.gyro_add, 1, true);
    i2c_read_blocking(I2C_PORT, mpu6050_reg.address, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro_raw[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // temperature from reg
    i2c_write_blocking(I2C_PORT, mpu6050_reg.address, &mpu6050_reg.temp_add, 1, true);
    i2c_read_blocking(I2C_PORT, mpu6050_reg.address, buffer, 2, false);
    temp_raw = buffer[0] << 8 | buffer[1];
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

    mpu_set_sample_rate(1);
    mpu_setresolution(0, 0);
    mpu_calc_offset(true, true)
    mpu_get_statistic(mpu6050);
    
    while (1) {
        sleep_ms(100);
    }
}




