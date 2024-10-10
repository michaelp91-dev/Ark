from machine import I2C, PIN
import time

SDA = Pin(2, Pin.OUT)
SCL = Pin(3, Pin.OUT)
PORT = 0
i2c = I2C(PORT, scl=SCL, sda=SDA, freq=400000)

 = I2C(PORT, scl=SCL, sda=SDA, freq=400000)

addr = 0x68;
reset_addr = 0x6B
gyro_config = 0x1B;
accel_config = 0x1C;
gyro_out = 0x43;
accel_out = 0x6B;

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
    
def read_raw():
    def read_data(): 
    buffer = bytearray(14)
    i2c.readfrom_mem_into(addr, accel_out, buffer);

    raw_data = [0,0,0,0,0,0,0]
    for i in range(7):
      raw_data[i] = bytes_toint(buffer[i], buffer[i+1])
      
    return raw_data
    
while True:
    print(read_raw)
    time.sleep(0.5)
