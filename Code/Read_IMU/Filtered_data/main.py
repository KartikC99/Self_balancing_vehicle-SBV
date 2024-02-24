

from machine import Pin, I2C
from imu import MPU6050	#IMU Library
from filters import MovingAverageFilter	# Filter Library

import math
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000) #I2C for MPU6050
imu = MPU6050(i2c)

#************************Parameter Adjust************************

CompCoef=0.1     #Coeficient of compensation error

OfsetGY = -0.387754     #Zero ofset Y-Axis of Gyro (deg/sec)
OfsetAY = -1.198136     #Zero ofset Y-Axis of Angle from Accel (deg)
Pitch_gy = 0.0000
Coef = 0.95     #Coeficient of Complementary Filtyer

SamplingTime=0.01

ReadAVerageData = MovingAverageFilter(100)	#Window Filter for setting ofset

#****************************************************************

ErrorP=0.0
        
start_time = time.ticks_ms()

LED = machine.Pin("LED", machine.Pin.OUT)
LED.on()	#Internal LED for checking system operation

while True:
    
    Ax=round(imu.accel.x,4)
    Ay=round(imu.accel.y,4)
    Az=round(imu.accel.z,4)
    
    Gx=round(imu.gyro.x,4)
    Gy=round(imu.gyro.y,4)
    Gz=round(imu.gyro.z,4)
    
    end_time = time.ticks_ms()
    loop_time = time.ticks_diff(end_time, start_time)/ 1000.0
    start_time = time.ticks_ms()
    
    Gy=Gy+OfsetGY #With Gyro ofset
    #Gy=Gy #Without Gyro ofset
    
    Pitch_acc = math.atan2(-Ax, math.sqrt(Ay**2 + Az**2))
    
    Pitch_gy+=Gy*loop_time

    Pitch_Acc_Deg=(Pitch_acc*180)/math.pi + OfsetAY #With Accel ofset
    #Pitch_Acc_Deg=(Pitch_acc*180)/math.pi #Without Accel ofset
    
    Pitch_Fusion = Coef*Pitch_gy + (1-Coef)*Pitch_Acc_Deg+ErrorP*CompCoef
    
    ErrorP=ErrorP+(Pitch_Acc_Deg-Pitch_Fusion)*loop_time
            
    #GyZeroAverage= ReadAVerageData.update(Gy)	#Use for read gyro ofset
    #print(0-GyZeroAverage)
        
    #AccelZeroAverage= ReadAVerageData.update(Pitch_Acc_Deg)	#Use for read accel ofset
    #print(0-AccelZeroAverage)
    
    #print(Ax, Ay, Az, Gx, Gy, Gz) #Raw Data from IMU
    
    print(Gy,Pitch_Acc_Deg, Pitch_Fusion)	#Read Angle and control signal
    time.sleep(SamplingTime)
    
    
    
    