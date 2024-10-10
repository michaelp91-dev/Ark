from machine import I2C, PIN
import time

SDA = Pin(2, Pin.OUT)
SCL = Pin(3, Pin.OUT)
PORT = 0
i2c = I2C(PORT, scl=SCL, sda=SDA, freq=400000)

addr = 0x68;
reset_addr = 0x6B
gyro_config = 0x1B;
accel_config = 0x1C;
gyro_out = 0x43;
accel_out = 0x6B;

gyro_degsec = 131.0
accel_g = 16384.0
accel_offset = [0,0,0]
gyro_offset = [0,0,0]
temp_offset = 12412.0
temp_deg = 340.0
offset_time = 120

def bytes_toint(msb, lsb):
    if not msb & 0x80:
        return msb << 8 | lsb
    return (((msb^255) << 8) | (lsb^255) + 1)    

def mpu_reset():
    buffer = bytearray(1)
    buffer[0] = 0x01
    i2c.writeto_mem(addr, reset_addr, buffer)
    time.sleep(1)
    print("MPU initialized")
    
def set_gyro_config(config_num):
    if config_num == 0:
        gyro_degsec = 131.0
        i2c.writeto_mem(addr, gyro_config, 0x00)
     elif config_num == 1:
         gyro_degsec = 65.5
         i2c.writeto_mem(addr, gyro_config, 0x08)
     elif config_num == 2:
         gyro_degsec = 32.8
         i2c.writeto_mem(addr, gyro_config, 0x10)
     elif config_num == 3:
         gyro_degsec = 16.4
         i2c.writeto_mem(addr, gyro_config, 0x18)    
         
def set_accel_config(config_num):
    if config_num == 0:
        accel_g = 16384.0
        i2c.writeto_mem(addr, accel_config, 0x00)
     elif config_num == 1:
         accel_g = 8192.0
         i2c.writeto_mem(addr, accel_config, 0x08)
     elif config_num == 2:
         accel_g = 4096.0
         i2c.writeto_mem(addr, accel_config, 0x10)
     elif config_num == 3:
         accel_g = 2048.0
         i2c.writeto_mem(addr, accel_config, 0x18)
         
 def calc_offsets(calc_gyro, calc_accel):
     if is calc_gyro:
         gyro_offset = [0,0,0]
         
     if is calc_accel:
         accel_offset = [0,0,0]
         
     offs = [0,0,0,0,0,0]
     print("Beginning calibration")
         
     for i in range(offset_time):
         percent = i / offset_time
         data_read = read_data()
         offs[0] = offs[0] + data_read[0]
         offs[1] = offs[1] + data_read[1]
         offs[2] = offs[2] + data_read[2]
         offs[3] = offs[3] + data_read[3]
         offs[4] = offs[4] + data_read[4]
         offs[5] = offs[5] + data_read[5]
         print(str(percent) + "% complete")
         time.sleep(1)
         
     if is calc_accel:
         accel_offset[0] = offs[0] / offset_time
         accel_offset[1] = offs[1] / offset_time
         accel_offset[2] = offs[2] / offset_time
         print("Accel offsets: " + str(accel_offset))
         
     if is calc_gyro:
         gyro_offset[0] = offs[3] / offset_time
         gyro_offset[1] = offs[4] / offset_time
         gyro_offset[2] = offs[5] / offset_time
         print("Gyro offsets: " + str(gyro_offset))
         
     print("Calibration complete")    
    
def read_data(): 
    buffer = bytearray(14)
    i2c.readfrom_mem_into(addr, accel_out, buffer);

    raw_data = [0,0,0,0,0,0,0]
    for i in range(7):
      raw_data[i] = bytes_toint(buffer[i], buffer[i+1])
      
    data = [0,0,0,0,0,0,0]
    data[0] = raw_data[0] / accel_g - accel_offset[0]
    data[1] = raw_data[1] / accel_g - accel_offset[1]
    data[2] = raw_data[2] / accel_g - accel_offset[2]
    data[3] = raw_data[4] / gyro_degsec - gyro_offset[0]
    data[4] = raw_data[5] / gyro_degsec - gyro_offset[1]
    data[5] = raw_data[6] / gyro_degsec - gyro_offset[2]
    data[6] = (raw_data[3] + temp_offse) / temp_deg
    return data
    
    
def main():
    mpu_reset()
    set_accel_config(0)
    set_gyro_config(0)
    time.sleep(5)
    calc_offsets(true, true)
    input("Ready for test.")  
    
    while True:
        print(read_data)
        time.sleep_ms(10)
        
main()        
