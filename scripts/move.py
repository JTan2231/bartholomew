#! /usr/bin/python2
# This is Python Library for four wheel Land rover
#
#Python Module to externalise all rover hardware
#
# Created by Praveen Damacharla, July 2015 Version 1.0
#
# Copyright @Praveen Damacharla
#Contact praveendamacharla@gmail.com
#
# This code is in not in the Public Domain for now and Under development stage
#
#======================================================================
#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
#======================================================================
#======================================================================
# Motor Functions
#
# stop(): Stops both motors
# forward(speed): Sets all four motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets all four motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
#======================================================================
#=====================================================================
#UltraSocinic Function
#
#getDistance() - Returns distance in cm to nearest reflecting object in three angles. 0 == No object
#
#=======================================================================
#=======================================================================
#GyroScope Functions
#gyro_xyz() : gyro scope x,y,z data return
#accel_xyz() :accelorometer x,y,z data returns
#========================================================================
#========================================================================
#Kinect sensor functions
#
#
#
#
#===========================================================================
#===========================================================================
import RPi.GPIO as GPIO, sys, threading, time, os  #Import all necessary libraries
import smbus, math
import numpy as np
#
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
# Define Motor Control Pins Direction and PWM
RF=21  # Right Front
RB=22  # Right Back
LF=19  # Left Front
LB=23  # Left Back
p_RF=15
p_RB=16
p_LF=11
p_LB=18
sonar1_in=36
sonar1_out=32
sonar2_in=37
sonar2_out=33
sonar3_in=38
sonar3_out=35
S=[0,0,0]
#========================================================================
#========================================================================
# General Functions
#
#init()-Initilises GPIO pins, motors, sensors, .. etc
def init():
    global pwm_RF, pwm_RB, pwm_LF, pwm_LB
    #initilises motor direction controls as OUTPUTs
    GPIO.setup(RF,GPIO.OUT)
    GPIO.setup(RB,GPIO.OUT)
    GPIO.setup(LF,GPIO.OUT)
    GPIO.setup(LB,GPIO.OUT)

    # Initilises motor speed conts as OUTPUT pwms
    GPIO.setup(p_RF,GPIO.OUT)
    pwm_RF=GPIO.PWM(p_RF,207)
    pwm_RF.start(0)

    GPIO.setup(p_RB,GPIO.OUT)
    pwm_RB=GPIO.PWM(p_RB,207)
    pwm_RB.start(0)

    GPIO.setup(p_LF,GPIO.OUT)
    pwm_LF=GPIO.PWM(p_LF,207)
    pwm_LF.start(0)

    GPIO.setup(p_LB,GPIO.OUT)
    pwm_LB=GPIO.PWM(p_LB,207)
    pwm_LB.start(0)

#cleanup Sets GPIOs at standead valus
def cleanup():

    stop()
    time.sleep(1)
    pwm_LB.stop()
    pwm_LF.stop()
    pwm_RB.stop()
    pwm_RF.stop()
    GPIO.cleanup()

#End of General Functions
#=============================================
#
#
#=============================================
# Motor Functions
# stop(): Stops all four motors
def stop():
    pwm_RF.ChangeDutyCycle(0)
    pwm_RB.ChangeDutyCycle(0)
    pwm_LF.ChangeDutyCycle(0)
    pwm_LB.ChangeDutyCycle(0)

# forward(speed): Sets all four motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    GPIO.output(RF,False) # Motor Direction
    GPIO.output(LF,False)
    GPIO.output(RB,False)
    GPIO.output(LB,False)
    # Motors speed
    pwm_RF.ChangeDutyCycle(speed)
    pwm_LF.ChangeDutyCycle(speed)
    pwm_RB.ChangeDutyCycle(speed)
    pwm_LB.ChangeDutyCycle(speed)

# reverse(speed): Sets all four motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    GPIO.output(RF,True) # Motor Direction
    GPIO.output(LF,True)
    GPIO.output(RB,True)
    GPIO.output(LB,True)
    # Motors speed
    pwm_RF.ChangeDutyCycle(speed)
    pwm_LF.ChangeDutyCycle(speed)
    pwm_RB.ChangeDutyCycle(speed)
    pwm_LB.ChangeDutyCycle(speed)

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    GPIO.output(RF,False) # Motor Direction
    GPIO.output(LF,True)
    GPIO.output(RB,False)
    GPIO.output(LB,True)
    # Motors speed
    pwm_RF.ChangeDutyCycle(speed)
    pwm_LF.ChangeDutyCycle(speed)
    pwm_RB.ChangeDutyCycle(speed)
    pwm_LB.ChangeDutyCycle(speed)

# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    GPIO.output(RF,True) # Motor Direction
    GPIO.output(LF,False)
    GPIO.output(RB,True)
    GPIO.output(LB,False)
    # Motors speed
    pwm_RF.ChangeDutyCycle(min(speed*1.5, 100))
    pwm_LF.ChangeDutyCycle(speed)
    pwm_RB.ChangeDutyCycle(min(speed*1.5, 100))
    pwm_LB.ChangeDutyCycle(speed)

# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    GPIO.output(RF,False) # Motor Direction
    GPIO.output(LF,False)
    GPIO.output(RB,False)
    GPIO.output(LB,False)
    # Motors speed
    pwm_RF.ChangeDutyCycle(rightSpeed)
    pwm_LF.ChangeDutyCycle(leftSpeed)
    pwm_RB.ChangeDutyCycle(rightSpeed)
    pwm_LB.ChangeDutyCycle(leftSpeed)


# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    GPIO.output(RF,True) # Motor Direction
    GPIO.output(LF,True)
    GPIO.output(RB,True)
    GPIO.output(LB,True)
    # Motors speed
    pwm_RF.ChangeDutyCycle(rightSpeed)
    pwm_LF.ChangeDutyCycle(leftSpeed)
    pwm_RB.ChangeDutyCycle(rightSpeed)
    pwm_LB.ChangeDutyCycle(leftSpeed)

# go(leftSpeed, rightSpeed): controls motors in both directions independently using different positive/negative speeds. -100<= leftSpeed,rightSpeed <= 100
def go(leftSpeed, rightSpeed):
    if leftSpeed<0:
       GPIO.output(LF,True)
       GPIO.output(LB,True)
       pwm_LB.ChangeDutyCycle(abs(leftSpeed))
       pwm_LF.ChangeDutyCycle(abs(leftSpeed))
    else:
       GPIO.output(LF,False)
       GPIO.output(LB,False)
       pwm_LB.ChangeDutyCycle(leftSpeed)
       pwm_LF.ChangeDutyCycle(leftSpeed)
    if rightSpeed<0:
       GPIO.output(RF,True)
       GPIO.output(RB,True)
       pwm_RF.ChangeDutyCycle(abs(rightSpeed))
       pwm_RB.ChangeDutyCycle(abs(rightSpeed))
    else:
       GPIO.output(RF,False)
       GPIO.output(RB,False)
       pwm_RB.ChangeDutyCycle(rightSpeed)
       pwm_RF.ChangeDutyCycle(rightSpeed)

# End of Motor Functions
#======================================================================
#
#
#======================================================================
#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object in three angles. 0 == no object
#
def getDistance():
#UltraSonic Sensor 1
    GPIO.setup(sonar1_out, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar1_out, True)
    time.sleep(0.00001)
    GPIO.output(sonar1_out, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar1_in,GPIO.IN)
    while GPIO.input(sonar1_in)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar1_in)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    S[0] = distance
#UltraSonic Sensor 2
    GPIO.setup(sonar2_out, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar2_out, True)
    time.sleep(0.00001)
    GPIO.output(sonar2_out, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar2_in,GPIO.IN)
    while GPIO.input(sonar2_in)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar2_in)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    S[1] = distance

#UltraSonic Sensor 3

    GPIO.setup(sonar3_out, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar3_out, True)
    time.sleep(0.00001)
    GPIO.output(sonar3_out, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar3_in,GPIO.IN)
    while GPIO.input(sonar3_in)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar3_in)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    S[2] = distance
    return S

# End of UltraSonic Functions
#======================================================================
#======================================================================
# GyroScope Functions
# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1)
address = 0x1e      # This is the address value read via the i2cdetect command

def read_word_2c(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

# Now wake the 6050 up as it starts in sleep mode
#bus.write_byte_data(address, power_mgmt_1, 0)

def gyro_xyz():
   gyro_xout = read_word_2c(0x43)
   gyro_yout = read_word_2c(0x45)
   gyro_zout = read_word_2c(0x47)
   return gyro_xout, gyro_yout, gyro_zout

def accel_xyz():
   accel_xout = (read_word_2c(0x3b)*3.6/1023-1.65)/.478
   accel_yout = (read_word_2c(0x3d)*3.6/1023-1.65)/.478
   accel_zout = (read_word_2c(0x3f)*3.6/1023-1.65)/.478
   return accel_xout, accel_yout, accel_zout

#End of Gyroscope Function
#=============================================================================
#=============================================================================
#Kinect Sensor Functions
def pretty_depth(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        A numpy array that has been processed whos datatype is unspecified
    """
    np.clip(depth, 0, 2**10 - 1, depth)
    depth >>= 2
    depth = depth.astype(np.uint8)
    return depth


def pretty_depth_cv(depth):
    """Converts depth into a 'nicer' format for display

    This is abstracted to allow for experimentation with normalization

    Args:
        depth: A numpy array with 2 bytes per pixel

    Returns:
        An opencv image who's datatype is unspecified
    """
    import cv
    depth = pretty_depth(depth)
    image = cv.CreateImageHeader((depth.shape[1], depth.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 1)
    cv.SetData(image, depth.tostring(),
               depth.dtype.itemsize * depth.shape[1])
    return image


def video_cv(video):
    """Converts video into a BGR format for opencv

    This is abstracted out to allow for experimentation

    Args:
        video: A numpy array with 1 byte per pixel, 3 channels RGB

    Returns:
        An opencv image who's datatype is 1 byte, 3 channel BGR
    """
    import cv
    video = video[:, :, ::-1]  # RGB -> BGR
    image = cv.CreateImageHeader((video.shape[1], video.shape[0]),
                                 cv.IPL_DEPTH_8U,
                                 3)
    cv.SetData(image, video.tostring(),
               video.dtype.itemsize * 3 * video.shape[1])
    return image
#End of Kinect function
#======================================================================
#======================================================================


