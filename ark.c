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
static int addr = 0x68;
static int reset_addr = 0x6B;
static int accel_addr = 0x3B;
static int gyro_addr = 0x43;
static int temp_addr = 0x41;

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {reset_addr, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
    sleep_ms(1000);
    printf("\r\nMPU6050 Setup Complete\r\n");
} 

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    i2c_write_blocking(i2c_default, addr, &accel_addr, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // gyro data from reg 0x43 for 6 bytes
    i2c_write_blocking(i2c_default, addr, &gyro_addr, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // temperature from reg 0x41 for 2 bytes
    i2c_write_blocking(i2c_default, addr, &temp_addr, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    temp = buffer[0] << 8 | buffer[1];
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
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

    //mpu
    MPU6050 mpu;
    
    while (1) {
        mpu6050_read_raw(accel_raw, gyro_raw, temp_raw);
;

        sleep_ms(100);
    }
}




