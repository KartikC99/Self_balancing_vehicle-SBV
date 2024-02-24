from machine import Pin, I2C
from imu import MPU6050	#IMU Library

import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000) #I2C for MPU6050
imu = MPU6050(i2c)

LED = machine.Pin("LED", machine.Pin.OUT)
LED.on()	#Internal LED for checking system operation

while True:
    
    Ax=round(imu.accel.x,4)
    Ay=round(imu.accel.y,4)
    Az=round(imu.accel.z,4)
    
    Gx=round(imu.gyro.x,4)
    Gy=round(imu.gyro.y,4)
    Gz=round(imu.gyro.z,4)
    
    print(Ax, Ay, Az, Gx, Gy, Gz) #Raw Data from IMU
    time.sleep(0.01)
    
    
    
    