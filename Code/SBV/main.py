

from machine import Pin, I2C, PWM
from imu import MPU6050	# Import the MPU6050 class from the imu module
from PID import PIDController	# Import the PIDController class
from filters import MovingAverageFilter	# Import the MovingAverageFilter class

import math
import time

# Initialize the I2C communication with MPU6050 using specified pins and frequency
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000) 
imu = MPU6050(i2c) # Create an instance of the MPU6050 class


#************************Parameter Adjust************************

Kp=0		# Proportional gain
Kd=0		# Derivative gain
Ki=0    	# Integral gain

# Additional parameters for control and calibration
CompCoef=0.1     #Coeficient of compensation error

OfsetGY = -0.387754     # Gyroscope Y-axis offset (deg/sec)
OfsetAY = -1.198136     # Accelerometer Y-axis offset (deg)
Pitch_gy = 0.0000
Coef = 0.95     # Coefficient for complementary filter

SamplingTime=0.01 # Time interval for sampling

Factor1=1.0     #Speed Ratio Motor 1
Factor2=1.0     #Speed Ratio Motor 2


# Filter for averaging readings
ReadAVerageData = MovingAverageFilter(100)	#Window Filter for setting ofset

#****************************************************************

setpoint = 0  # Desired angle setpoint (usually 0 for balancing)

# Initialize the PID controller with specified gains and setpoint
pid_controller = PIDController(Kp, Ki, Kd, setpoint)
dt = SamplingTime # Time delta for PID calculations



# Pin setup for motor direction control
Direct1 = Pin(18,Pin.OUT)
Direct2 = Pin(19,Pin.OUT)
Direct3 = Pin(21,Pin.OUT)
Direct4 = Pin(20,Pin.OUT)

# PWM setup for motor speed control
Drive1 = PWM(Pin(16))
Drive1.freq(5000)
Drive1.duty_u16(0)

Drive2 = PWM(Pin(17))
Drive2.freq(5000)
Drive2.duty_u16(0)

Drive1.duty_u16(0)
Drive2.duty_u16(0)


# Function to control motor direction and speed
def MotorMove(Direction, Speed):
    # Direction 0 stops the motors, 1 moves forward, 2 moves backward
    if(Direction==0):
        Direct1.value(0)
        Direct2.value(0)
        Direct3.value(0)
        Direct4.value(0)
        
    elif(Direction==1):
        Direct1.value(1)
        Direct2.value(0)
        Direct3.value(1)
        Direct4.value(0)
    
    elif(Direction==2):
        Direct1.value(0)
        Direct2.value(1)
        Direct3.value(0)
        Direct4.value(1)
    # Adjust PWM duty cycle based on speed and factor ratios
    Drive1.duty_u16(int(Speed*Factor1))
    Drive2.duty_u16(int(Speed*Factor2))

ErrorP=0.0 # Initialize error for proportional control
        
start_time = time.ticks_ms() # Start time for loop timing

LED = machine.Pin("LED", machine.Pin.OUT)
LED.on()	# Turn on LED to indicate system operation

# Main loop for reading sensor data, calculating PID, and controlling motors
while True:
    # Read accelerometer and gyroscope data
    Ax=round(imu.accel.x,4)
    Ay=round(imu.accel.y,4)
    Az=round(imu.accel.z,4)
    
    Gx=round(imu.gyro.x,4)
    Gy=round(imu.gyro.y,4)
    Gz=round(imu.gyro.z,4)
    
    # Calculate loop time
    end_time = time.ticks_ms()
    loop_time = time.ticks_diff(end_time, start_time)/ 1000.0
    start_time = time.ticks_ms()
    
    Gy=Gy+OfsetGY #With Gyro ofset Apply gyroscope offset
    #Gy=Gy #Without Gyro ofset
    
    # Calculate pitch angle from accelerometer data
    Pitch_acc = math.atan2(-Ax, math.sqrt(Ay**2 + Az**2))
    Pitch_gy+=Gy*loop_time  # Integrate gyroscope data for angle

    Pitch_Acc_Deg=(Pitch_acc*180)/math.pi + OfsetAY #With Accel ofset
    #Pitch_Acc_Deg=(Pitch_acc*180)/math.pi #Without Accel ofset
    
    Pitch_Fusion = Coef*Pitch_gy + (1-Coef)*Pitch_Acc_Deg+ErrorP*CompCoef
    
    ErrorP=ErrorP+(Pitch_Acc_Deg-Pitch_Fusion)*loop_time
    
    control_signal = pid_controller.update(Pitch_Fusion,dt)
    
    if(control_signal>0):
        MotorMove(2,control_signal)
    
    elif(control_signal<0):
        MotorMove(1,-control_signal)
    
    else:
        MotorMove(0,0)
        
    #GyZeroAverage= ReadAVerageData.update(Gy)	#Use for finding the gyro offset, uncomment it and find the value of OfsetGY, after changing the value the printing value should be almost zero. Make sure to place the vehicle flat
    #print(0-GyZeroAverage)
        
    #AccelZeroAverage= ReadAVerageData.update(Pitch_Acc_Deg)	#Use for finding the accel offset, uncomment it and find the value of OfsetAY, after changing the value the printing value should be almost zero. Make sure to place the vehicle flat
    #print(0-AccelZeroAverage)
    
    #print(Pitch_Fusion,control_signal)	#Read Angle and control signal
    #time.sleep(0.01)
    
    
    
    
