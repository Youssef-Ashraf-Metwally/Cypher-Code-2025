#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.robotics import Stop
import math
from threading import Thread
import time
import sys
import threading

# Objects.
ev3 = EV3Brick()
Gyro_Sensor = GyroSensor(Port.S2)
Right_Motor_L = Motor(Port.B)
Left_Motor_L = Motor(Port.C)
Left_Motor_M = Motor(Port.D)
Right_Motor_M = Motor(Port.A)
R_Color_Sensor = ColorSensor(Port.S3)
L_Color_Sensor = ColorSensor(Port.S1)
Robot = DriveBase(Left_Motor_L, Right_Motor_L, 54.5, 140)

# Variables
interrupt_flag = False
Running = False
Integral = 0
LastError = 0
Current_Speed = 0
RUN = True
TIME = 0

# Header.
def Move_Steering (Speed, Distance, Steering):
    global interrupt_flag
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    while abs(float(Robot.distance())) < abs(Distance):
        Robot.drive(Speed, Steering)
    else:
        Robot.stop()
def Get_Direction(T1, T2):
    if T1 > T2:
        Direction = 1
    elif T1 < T2:
        Direction = -1
    else:
        Direction = 0
    return (Direction)
def Robot_Break():
    Left_Motor_L.brake()
    Right_Motor_L.brake()
def Move_Tank(Left, Right):
    Left_Motor_L.run(Left)
    Right_Motor_L.run(Right)
def Gyro_Check(Target):
    global interrupt_flag
    while Gyro_Sensor.angle() < Target:
        if interrupt_flag:
            break
        Move_Tank(-100,100)
    else:
        while Gyro_Sensor.angle() > Target:
            if interrupt_flag:
                break
            Move_Tank(100,-100)
        else:
            Robot_Break()
def Motor_Check(Target):
    global interrupt_flag
    
    while float(Robot.distance()) < Target:
        if interrupt_flag:
            break
        Move_Tank(100,100)
    else:
        while float(Robot.distance()) > Target:
            if interrupt_flag:
                break
            Move_Tank(-100,-100)
        else:
            Robot_Break()
def P_Gyro_Turn(Target, LP, RP, min_speed=50, max_speed=300):
    global interrupt_flag
    while Gyro_Sensor.angle() != Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
            #break
        Error = Target - Gyro_Sensor.angle()
        left_speed = -Error * LP
        right_speed = Error * RP
        left_speed = max(min_speed, min(max_speed, abs(left_speed))) * (1 if left_speed > 0 else -1)
        right_speed = max(min_speed, min(max_speed, abs(right_speed))) * (1 if right_speed > 0 else -1)
        Move_Tank (left_speed, right_speed)
        time.sleep(0.01)
    else:
        Robot_Break()

    Gyro_Check(Target)
def Gyro_Turn(Target, L_Speed, R_Speed):
    global interrupt_flag
    Direction= Get_Direction(Target, Gyro_Sensor.angle())
    while Direction* Gyro_Sensor.angle() < Direction* Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
            #break
        Error = Target - Gyro_Sensor.angle()
        Move_Tank (L_Speed, R_Speed)
    else:
        Robot_Break()

    Gyro_Check(Target)
def PID_WALK (Speed, TA, KP, KI, KD):
    global Integral, LastError
    Error = TA - Gyro_Sensor.angle()
    Proportional = KP*Error
    Integral = (Integral + Error)*KI
    Derrivative= (Error - LastError)*KD
    Steering = Proportional + Integral + Derrivative
    Robot.drive(Speed, Steering)
    LastError =  Error
def Speed_Control(Speed, KA, Decceleration, DF):
    global Current_Speed
    Initial_Speed = (Speed*(0.001*KA))
    if (abs(Current_Speed) < abs(Speed)) and Decceleration== False:
        Current_Speed = Current_Speed + Initial_Speed
    elif Decceleration:
        Current_Speed = Current_Speed - Initial_Speed*DF
        if abs(Current_Speed) < 50:
            Current_Speed = 50 if Speed > 0 else -50
    else:
        Current_Speed = Speed
    return(Current_Speed)
def PID_COLOR_R (Speed, TA, TC, Margin, KP, KI, KD, KA):
    global interrupt_flag
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    while not(int(R_Color_Sensor.reflection()) > int(TC-Margin) and int(R_Color_Sensor.reflection()) < int(TC+Margin)):
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
        PID_WALK(Speed_Control(Speed, KA, False, 3), TA, KP, KI, KD)
    else:
        Robot.stop(Stop.BRAKE)
def PID_COLOR_L (Speed, TA, TC, Margin, KP, KI, KD, KA):
    global interrupt_flag
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    while not(int(L_Color_Sensor.reflection()) > int(TC-Margin) and int(L_Color_Sensor.reflection()) < int(TC+Margin)):
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
        PID_WALK(Speed_Control(Speed, KA, False, 3), TA, KP, KI, KD)
    else:
        Robot.stop(Stop.BRAKE)
def PID (Speed, TA, TD, KP, KI, KD, KA, DF=3.0, doMcheck = True):
    global interrupt_flag, Integral
    Decceleration= False
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    TD = TD*-1 if Speed < 0 else TD
    while abs(Robot.distance()) < abs(TD):
        Decceleration= abs(float(Robot.distance())) > abs(TD*0.7)
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = 1/0
        PID_WALK(Speed_Control(Speed, KA, Decceleration, DF), TA, KP, KI, KD)
    else:
        Robot.stop(Stop.BRAKE)
        if (doMcheck):
            Motor_Check(TD)
        ev3.screen.clear()
        ev3.screen.print(Robot.distance())
def CHECK_TIME():
    global RUN, TIME
    wait(TIME)
    RUN = False
def PID_TIME (Speed, TA, Time, KP, KI, KD, KA):
    global interrupt_flag, Integral, RUN, TIME
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    RUN = True
    TIME = Time
    time_thread = Thread(target=CHECK_TIME)
    time_thread.start()
    while RUN:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = 1/0
        PID_WALK(Speed_Control(Speed, KA, False, 3), TA, KP, KI, KD)
    else:
        Robot.stop(Stop.BRAKE)
        ev3.screen.clear()
        ev3.screen.print(Robot.distance())
def check_interrupt():
    global interrupt_flag
    while True:
        # Check if the center button is pressed
        if Button.LEFT in ev3.buttons.pressed():
            Running = False
            interrupt_flag = True
            # mission_thread.join()
            Robot.stop(Stop.BRAKE)
            Right_Motor_M.stop()
            Left_Motor_M.stop()
            ev3.screen.clear()
            ev3.screen.print("Interrupted")
            break
        wait(100)
def reset_all():
    Gyro_Sensor.reset_angle(0)
    Left_Motor_L.reset_angle(0)
    Right_Motor_L.reset_angle(0)
    Left_Motor_M.reset_angle(0)
    Right_Motor_M.reset_angle(0)