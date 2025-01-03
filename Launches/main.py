#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import math
from threading import Thread

# Objects.
ev3 = EV3Brick()
GS = GyroSensor(Port.S2)
RM = Motor(Port.B)
LM = Motor(Port.C)
MML = Motor(Port.D)
MMR = Motor(Port.A)
LCS = ColorSensor(Port.S3)
RCS = ColorSensor(Port.S1)

Robot = DriveBase(LM, RM, 54.5, 140)

# Variables
interrupt_flag = False

# Header.
def Get_Direction(T1, T2):
    if T1 > T2:
        Direction = 1
    elif T1 < T2:
        Direction = -1
    else:
        Direction = 0
    return (Direction)
def Robot_Break():
    LM.hold()
    RM.hold()
def LineSquare (Value, Speed):
    while RCS.reflection() > Value and LCS.reflection() > Value:
        Robot.drive(Speed, 0)
    else:
        Robot.stop()
    
    if RCS.reflection() < Value:
        print("right")
        while LCS.reflection() > Value:
            LM.run(Speed)
        else:
            LM.brake()
    elif LCS.reflection() < Value:
        print("left")
        while RCS.reflection() > Value:
            RM.run(Speed)
        else:
            RM.brake()
    else: 
        print("None")
        Robot.stop()
    print(LCS.reflection())
    print(RCS.reflection())
def Move_Tank(Left, Right):
    LM.run(Left)
    RM.run(Right)
def Gyro_Check(Target):
    global interrupt_flag
    D = Get_Direction(Target, GS.angle())

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
    print ("D = ", D)
    print ("__________________________________")
def P_Gyro_Turn(Target, LP, RP):
    global interrupt_flag
    while GS.angle() != Target:
        if interrupt_flag:
            break
        Error = Target - GS.angle()
        LSpeed = Error * LP * -1
        RSpeed = Error * RP
        Move_Tank (LSpeed, RSpeed)
    else:
        Robot_Break()

    Gyro_Check(Target)
    Gyro_Check(Target)
def Gyro_Turn(Target, LSpeed, RSpeed):
    D = Get_Direction(Target, GS.angle())

    while D* GS.angle() < D* Target:
        Error = Target - GS.angle()
        Move_Tank (LSpeed, RSpeed)
    else:
        Robot_Break()

    Gyro_Check(Target)
    print (GS.angle())
    Gyro_Check(Target)
    print (GS.angle())
    print ("__________________________________")
    print("D = ", D)            
def PID (Speed, TA, TD, KP, KI, KD):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    LastError = 0
    I = 0
    while abs(float(Robot.distance())) < abs(TD):
        if interrupt_flag:
            break
        Error = TA - GS.angle()
        P = KP*Error
        I = (I + Error)*KI
        D = (Error - LastError)*KD
        Steering = P + I + D
        Robot.drive(Speed, Steering)
        print(GS.angle())
        LastError =  Error
    else:
        Robot.stop()
        #Robot_Break()
def Line_Follow (Speed, C1, C2, TD, KP):
    Robot.reset()
    TC = (C1 + C2)/2
    D = Get_Direction(C1, C2)
    while Robot.distance() > TD:
        Error = TC - RCS.reflection()
        P = KP*Error*D
        Robot.drive(Speed, P)
        print(RCS.reflection())
    else:
        Robot.stop()
        print("N")
def check_interrupt():
    global interrupt_flag
    while True:
        # Check if the center button is pressed
        if Button.CENTER in ev3.buttons.pressed():
            interrupt_flag = True
            Robot_Break()
            ev3.screen.clear()
            ev3.screen.print("Interrupted")
            break
        wait(100)

#Launches.
def Launch_1():
    global interrupt_flag
    ev3.screen.print("Mission 1")
    while True:
        if interrupt_flag:
            break
        GS.reset_angle(0)
        P_Gyro_Turn(-5, 7, 7)
        PID (-1000, -5, 420, -5, 0, 0)
        P_Gyro_Turn(0, 7, 7)
        MMR.run_time (-1000,1700,then=Stop.HOLD)
        PID (-30, 0, 50, -5, 0, 0)
        MMR.run_time (1000,1700,then=Stop.HOLD)
        P_Gyro_Turn(10, 7, 7)
        PID (-100, 10, 100, -5, 0, 0)
        P_Gyro_Turn(-73, 9, 9)
        PID (-1000, -73, 290, -5, 0, 0)
        P_Gyro_Turn(-90, 9, 9)
        Gyro_Check(-90)
        PID (-1000, -90, 240, -5, 0, 0)
        MMR.run_time (-1000,1600,then=Stop.HOLD)
        PID (-500, -90, 105, -5, 0, 0)
        MMR.run_time (1000,1600,then=Stop.HOLD)
        PID (-500, -90, 160, -5, 0, 0)
        P_Gyro_Turn(-130, 7, 7)
        MMR.run_time (-1000,500,then=Stop.HOLD)
        PID (-500, -130, 10000, -5, 0, 0)
        break
    Robot_Break()
def Launch_2():
    global interrupt_flag
    ev3.screen.print("Mission 2")
    while True:
        if interrupt_flag:
            break
        GS.reset_angle(0)
        PID (1000, 0, 3000, -10, 0, 0)
        P_Gyro_Turn(-90, 10, 10)
        break
    Robot_Break()
def Launch_3():
    global interrupt_flag
    ev3.screen.print("Mission 3")
    while True:
        if interrupt_flag:
            break
        PID (-1000, 0, 5000, -10, 0, 0)
        P_Gyro_Turn(180, 10, 10)
        PID (-1000, 180, 5000, -10, 0, 0)
        break
    Robot_Break()
def Launch_4():
    global interrupt_flag
    ev3.screen.print("Mission 4")
    while True:
        if interrupt_flag:
            break
        for i in range(4):
            if interrupt_flag:
                break
            i += 1
            GS.reset_angle(0)
            PID (-1000, 0, 5000, 2, 0, 0)
            P_Gyro_Turn(90, 10, 10)
        break
    Robot_Break()
def Launch_5():
    global interrupt_flag
    ev3.screen.print("Mission 5")
    while True:
        if interrupt_flag:
            break
        GS.reset_angle(0)
        P_Gyro_Turn(90, 10, 10)
        wait(1000)
        P_Gyro_Turn(180, 10, 10)
        wait(1000)
        P_Gyro_Turn(270, 10, 10)
        break
    Robot_Break()

def run_miassion():
    global interrupt_flag
    secondary = False
    while True:
        ev3.screen.clear()
        ev3.screen.print(secondary)

        # Wait for a button press
        while True:
            buttons = ev3.buttons.pressed()
            
            # Mission 1 with Left Button
            if Button.LEFT in buttons:
                interrupt_flag = False
                interrupt_thread = Thread(target=check_interrupt)
                interrupt_thread.start()
                if secondary:
                    Launch_4()
                    secondary = False
                else:
                    Launch_1()
                break

            # Mission 2 with Right Button
            elif Button.RIGHT in buttons:
                interrupt_flag = False
                interrupt_thread = Thread(target=check_interrupt)
                interrupt_thread.start()
                if secondary:
                    Launch_5()
                    secondary = False
                else:
                    Launch_2()
                break

            elif Button.UP in buttons:
                interrupt_flag = False
                interrupt_thread = Thread(target=check_interrupt)
                interrupt_thread.start()
                Launch_3()
                break

            elif Button.DOWN in buttons:
                if secondary:
                    secondary = False
                else:
                    secondary = True
                wait(50)
                break

        # If interrupted, reset and wait for the next mission
        if interrupt_flag:
            wait(500)  # Wait a little before re-choosing mission
            interrupt_flag = False
