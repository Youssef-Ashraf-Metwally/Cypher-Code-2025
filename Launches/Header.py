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
GS = GyroSensor(Port.S2)
RM = Motor(Port.B)
LM = Motor(Port.C)
MML = Motor(Port.D)
MMR = Motor(Port.A)
RCS = ColorSensor(Port.S3)
LCS = ColorSensor(Port.S1)
Robot = DriveBase(LM, RM, 54.5, 140)

# Variables
interrupt_flag = False
Running = False
I = 0
LastError = 0
Current_Speed = 0

# Header.
def Move_Steering (Speed, Distance, Steering):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
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
    LM.brake()
    RM.brake()
def LineSquare (Value, Speed):
    while RCS.reflection() > Value and LCS.reflection() > Value:
        Robot.drive(Speed, 0)
    else:
        Robot.stop()
    
    if RCS.reflection() < Value:
        while LCS.reflection() > Value:
            LM.run(Speed)
        else:
            LM.brake()
    elif LCS.reflection() < Value:
        while RCS.reflection() > Value:
            RM.run(Speed)
        else:
            RM.brake()
    else: 
        Robot.stop()
def Move_Tank(Left, Right):
    LM.run(Left)
    RM.run(Right)
def Gyro_Check(Target):
    global interrupt_flag
    while GS.angle() < Target:
        if interrupt_flag:
            break
        Move_Tank(-100,100)
    else:
        while GS.angle() > Target:
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
    while GS.angle() != Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
            #break
        Error = Target - GS.angle()
        left_speed = -Error * LP
        right_speed = Error * RP
        left_speed = max(min_speed, min(max_speed, abs(left_speed))) * (1 if left_speed > 0 else -1)
        right_speed = max(min_speed, min(max_speed, abs(right_speed))) * (1 if right_speed > 0 else -1)
        Move_Tank (left_speed, right_speed)
        time.sleep(0.01)
    else:
        Robot_Break()

    Gyro_Check(Target)
def Gyro_Turn(Target, LSpeed, RSpeed):
    global interrupt_flag
    Direction= Get_Direction(Target, GS.angle())
    while Direction* GS.angle() < Direction* Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
            #break
        Error = Target - GS.angle()
        Move_Tank (LSpeed, RSpeed)
    else:
        Robot_Break()

    Gyro_Check(Target)
def PID_WALK (Speed, TA, KP, KI, KD):   
    global I, LastError
    Error = TA - GS.angle()
    P = KP*Error
    I = (I + Error)*KI
    Derrivative= (Error - LastError)*KD
    Steering = P + I + Derrivative
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

def PID_COLOR (Speed, TA, TC, Margin, KP, KI, KD, KA):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    while not(int(RCS.reflection()) > int(TC-Margin) and int(RCS.reflection()) < int(TC+Margin)):
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
        Robot.drive(Speed, 0)
    else:
        Robot.stop(Stop.BRAKE)
def PID_COLOR_L (Speed, TA, TC, Margin, KP, KI, KD, KA):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    while not(int(LCS.reflection()) > int(TC-Margin) and int(LCS.reflection()) < int(TC+Margin)):
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = ColorSensor(Port.S4)
        Robot.drive(Speed, 0)
    else:
        Robot.stop(Stop.BRAKE)
def PID (Speed, TA, TD, KP, KI, KD, KA, DF=3.0, doMcheck = True):
    global interrupt_flag, I
    Decceleration= False
    LM.reset_angle(0)
    RM.reset_angle(0)
    TD = TD*-1 if Speed < 0 else TD
    while abs(float(Robot.distance())) < abs(TD):
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

def PID_TIME (Speed, TA, Td, KP, KI, KD, KA, DF=3.0):
    global interrupt_flag, I
    Decceleration= False
    LM.reset_angle(0)
    RM.reset_angle(0)
    TD = TD*-1 if Speed < 0 else TD
    while abs(float(Robot.distance())) < abs(TD):
        Decceleration= abs(float(Robot.distance())) > abs(TD*0.7)
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            X = 1/0
        PID_WALK(Speed_Control(Speed, KA, Decceleration, DF), TA, KP, KI, KD)
    else:
        Robot.stop(Stop.BRAKE)
        Motor_Check(TD)
        ev3.screen.clear()
        ev3.screen.print(Robot.distance())


def Line_Follow (Speed, C1, C2, TD, KP):
    Robot.reset()
    TC = (C1 + C2)/2
    Direction= Get_Direction(C1, C2)
    while Robot.distance() > TD:
        Error = TC - RCS.reflection()
        P = KP*Error*D
        Robot.drive(Speed, P)
    else:
        Robot.stop()
def check_interrupt():
    global interrupt_flag
    while True:
        # Check if the center button is pressed
        if Button.LEFT in ev3.buttons.pressed():
            Running = False
            interrupt_flag = True
            # mission_thread.join()
            Robot.stop(Stop.BRAKE)
            MMR.stop()
            MML.stop()
            ev3.screen.clear()
            ev3.screen.print("Interrupted")
            break
        wait(100)
def reset_all():
    GS.reset_angle(0)
    LM.reset_angle(0)
    RM.reset_angle(0)
    MML.reset_angle(0)
    MMR.reset_angle(0)
