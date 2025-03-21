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
    LM.hold()
    RM.hold()
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
def P_Gyro_Turn(Target, LP, RP):
    global interrupt_flag
    while GS.angle() != Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            x = 1/0
            #break
        Error = Target - GS.angle()
        LSpeed = Error * LP * -1
        RSpeed = Error * RP
        Move_Tank (LSpeed, RSpeed)
    else:
        Robot_Break()

    Gyro_Check(Target)
def Gyro_Turn(Target, LSpeed, RSpeed):
    global interrupt_flag
    D = Get_Direction(Target, GS.angle())
    while D* GS.angle() < D* Target:
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            x = 1/0
            #break
        Error = Target - GS.angle()
        Move_Tank (LSpeed, RSpeed)
    else:
        Robot_Break()

    Gyro_Check(Target)
def PID_WALK (Speed, TA, KP, KI, KD):
    global I
    global LastError
    
    Error = TA - GS.angle()
    P = KP*Error
    I = (I + Error)*KI
    D = (Error - LastError)*KD
    Steering = P + I + D
    Robot.drive(Speed, Steering)
    LastError =  Error
def Speed_Control(Speed, KA, D):
    global Current_Speed

    Initial_Speed = (Speed*(0.001*KA))
    if abs(Current_Speed) < abs(Speed) and D == False:
        Current_Speed = Current_Speed + Initial_Speed
    elif D:
        Current_Speed = Current_Speed - Speed*0.12
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
            x = 1/0
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
            x = 1/0
        Robot.drive(Speed, 0)
    else:
        Robot.stop(Stop.BRAKE)
def PID (Speed, TA, TD, KP, KI, KD, KA):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    TD *= -1
    while abs(float(Robot.distance())) < abs(TD):
        D = not(abs(float(Robot.distance())) < abs(TD - 50))
        if interrupt_flag:
            Robot.stop(Stop.BRAKE)
            x = 1/0
            #break
        PID_WALK(Speed_Control(Speed, KA, D), TA, KP, KI, KD)
    else:
        Robot.drive(Speed/-10, 0)
        wait(125)
        Robot.stop(Stop.BRAKE)
        #Robot.stop(brake = True)
def Line_Follow (Speed, C1, C2, TD, KP):
    Robot.reset()
    TC = (C1 + C2)/2
    D = Get_Direction(C1, C2)
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
            interrupt_flag = True
            mission_thread.join()
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

#Launches.
def Launch_1():
    global interrupt_flag
    ev3.screen.clear()
    ev3.screen.print("Mission 1")
    while True:
        if interrupt_flag:
            break
        reset_all()
        GS.reset_angle(0)
        P_Gyro_Turn(-5, 10, 10)
        PID(-1000, -5, 390, -5, -0.1, -10, 15)
        P_Gyro_Turn(0, 10, 10)
        MMR.run_time (-1000,1500,then=Stop.HOLD)
        PID(-30, 0, 50, -5, -0.1, -10, 15)
        MMR.run_time (1000,1500,then=Stop.HOLD)
        P_Gyro_Turn(10, 10, 10)
        PID(-100, 10, 100, -5, -0.1, -10, 15)
        MML.run_time (-1000,1500,then=Stop.HOLD)
        P_Gyro_Turn(-69, 7, 7)
        PID(-300, -69, 310, -4, -0.1, -10, 15)
        P_Gyro_Turn(-90, 10, 10)
        Gyro_Check(-90)
        PID(-1000, -90, 243, -5, -0.1, -10, 15)
        MMR.run_time (-1000,1600,then=Stop.HOLD)
        PID(50, -90, 25, -5, -0.1, -10, 15)
        MML.run_time (1000,1500,then=Stop.HOLD)
        PID(-500, -90, 120, -5, -0.1, -10, 15)
        MMR.run_time (1000,1600,then=Stop.HOLD)
        PID(-500, -90, 180, -5, -0.1, -10, 15)
        P_Gyro_Turn(-145, 12, 12)
        MMR.run_time (-1000,700,then=Stop.HOLD)
        PID(-500, -145, 10000, -5, -0.1, -10, 15)
    Robot_Break()
def Launch_2():
    global interrupt_flag
    ev3.screen.clear()
    ev3.screen.print("Mission 2")
    while True:
        if interrupt_flag:
            break
        reset_all()
        P_Gyro_Turn(0, 7, 7)
        PID(-200, 0, 200, -5, -0.1, -10, 15)
        PID_COLOR(-50, 0, 81, 2, -5, -0.1, -10, 15)
        MML.run_target(650, 1600, then=Stop.HOLD, wait=False)
        PID(-60, 1, 200, -5, -0.1, -10, 15)
        MMR.run_target(1000, -800, then=Stop.HOLD)
        wait(500)
        MMR.run_target(1400, 1500, then=Stop.HOLD)
        MMR.run_target(600, 400, then=Stop.HOLD)
        MML.run_target(300, 1000, then=Stop.HOLD, wait=False)
        PID(1000, 0, 150, -5, -0.1, -10, 15)
        reset_all()
        PID(1000, -70, 3000, -15, -0.1, -10, 15)
        break
    Robot_Break()
def Launch_3():
    global interrupt_flag
    ev3.screen.clear()
    ev3.screen.print("Mission 3")
    while True:
        if interrupt_flag:
            break
        reset_all()
        P_Gyro_Turn(-55, 7, 7)
        PID(-150, -55, 430, -5, -0.1, -10, 15)
        P_Gyro_Turn(0, 5, 15)
        Gyro_Check(0)
        MMR.run_target(-1000, -3700, then=Stop.HOLD, wait=False)
        MML.run_target(-1000, -1300, then=Stop.HOLD, wait=False)
        PID(-30, 0, 165, -6, -0.1, -10, 15)
        wait(300)
        MML.reset_angle(0)
        MML.run_target(-1300, -1600, then=Stop.HOLD)
        MML.run_target(-5000, 1100, then=Stop.HOLD, wait=False)
        wait(300)
        PID(-50, 0, 15, -5, -0.1, -10, 15)
        PID(200, 0, 150, -5, -0.1, -10, 15)
        P_Gyro_Turn(130, 7, 7)
        PID(-500, 130, 1000, -5, -0.1, -10, 15)
    Robot_Break()
def Launch_4():
    global interrupt_flag
    ev3.screen.clear()
    ev3.screen.print("Mission 4")
    while True:
        if interrupt_flag:
            break
        reset_all()
        P_Gyro_Turn(0, 7, 7)
        reset_all()
        GS.reset_angle(0)
        MMR.run(-50)
        PID(-400, 0, 900, -5, -0.1, -10, 15)
        MMR.stop()
        wait(500)
        MMR.reset_angle(0)
        MMR.run_target(1600, 140, then=Stop.HOLD, wait=True)
        MMR.run_target(-700, -100, then=Stop.HOLD, wait=True)
        MMR.run_time(1600, 1200, then=Stop.HOLD, wait=True)
        wait(500)
        MMR.run_time(-500, 1200, then=Stop.HOLD, wait=True)
        MML.run_target(1000, 650, then=Stop.HOLD, wait=True)
        PID(400, 0, 350, -5, -0.1, -10, 15)
        MML.run_target(-1000, -300, then=Stop.HOLD, wait=True)
        MMR.run_time(1600, 1000, then=Stop.HOLD, wait=False)
        P_Gyro_Turn(-23, 10, 10)
        PID(-300, -23, 150, -5, -0.1, -10, 15)
        MMR.run_target(-1600, -100, then=Stop.HOLD, wait=True)
        PID(-300, -23, 270, -5, -0.1, -10, 15)
        P_Gyro_Turn(0, 10, 10)
        PID(-400, 0, 400, -5, -0.1, -10, 15)
        P_Gyro_Turn(45, 10, 10)
        MML.run_target(1000, 200, then=Stop.HOLD, wait=True)
        PID(500, 45, 250, -5, -0.1, -10, 15)
        PID(-450, 35, 400, -5, -0.1, -10, 15)
    Robot_Break()
def Launch_5():
    global interrupt_flag
    ev3.screen.clear()
    ev3.screen.print("Mission 5")
    while True:
        if interrupt_flag:
            break
        reset_all()
        PID_COLOR(-50, 0, 82, 2, -5, -0.1, -10, 15)
        PID(-200, 0, 170, -5, -0.1, -10, 15)
        P_Gyro_Turn(90, 7, 7)
        reset_all()
        MML.run_target(-500, -340, then=Stop.HOLD, wait=False)
        PID(-400, 0, 750, -5, -0.1, -10, 15)
        GS.reset_angle(0)
        LM.reset_angle(0)
        RM.reset_angle(0)
        MML.run_target(-500, 450, then=Stop.HOLD)
        wait(1000)
        PID(200, 0, 110, -5, -0.1, -10, 15)
        P_Gyro_Turn(45, 12, 12)
        PID(-100, 45, 120, -5, -0.1, -10, 15)
        wait(500)
        PID(-1000, 45, 65, -5, -0.1, -10, 15)
        MMR.run_target(-10000, 3500, then=Stop.HOLD, wait=False)
        PID(100, 45, 205, -5, -0.1, -10, 15)
        P_Gyro_Turn(-45, 15, 10)
        PID(50, -45, 145, -5, -0.1, -10, 15)
        MMR.run_target(-10000, -500, then=Stop.HOLD)
        PID(-300, -45, 195, -5, -0.1, -10, 15)
        P_Gyro_Turn(-75, 0, 12)
        PID(-200, -75, 350, -5, -0.1, -10, 15)
        P_Gyro_Turn(-180, 12, 12)
        MMR.run_target(10000, 350, then=Stop.HOLD)
        PID(200, -180, 30, -5, -0.1, -10, 15)
        MMR.run_target(-10000, -2000, then=Stop.HOLD)
        PID(-200, -180, 20, -5, -0.1, -10, 15)
        wait(50000)
    Robot_Break()

def menu():

    global interrupt_flag
    Current_Launch = 0
    Launch_Names = ["Launch 1", "Launch 2", "Launch 3", "Launch 4", "Launch 5"]
    Launches = [Launch_1, Launch_2, Launch_3, Launch_4, Launch_5]
    Running = False

    while True:
            buttons = ev3.buttons.pressed()
            
            if Button.UP in buttons:
                if Current_Launch == 0:
                    Current_Launch = 4
                else:
                    Current_Launch -= 1
            elif Button.DOWN in buttons:
                if Current_Launch == 4:
                    Current_Launch = 0
                else:
                    Current_Launch += 1
            
            if not Running:
                ev3.screen.clear()
                ev3.screen.print("Anything")
                for i, name in enumerate(Launch_Names):
                    if i == Current_Launch:
                        ev3.screen.print("<> " + name)
                    else:
                        ev3.screen.print(name)
                
            if Button.CENTER in buttons:
                interrupt_flag = False
                interrupt_thread = Thread(target=check_interrupt)
                interrupt_thread.start()
                mission_thread = Thread(target=Launches[Current_Launch])
                mission_thread.start()
                Running = True

            wait(100)
            
            # If interrupted, reset and wait for the next mission
            if interrupt_flag:
                wait(500)  # Wait a little before re-choosing mission
                # interrupt_flag = False
                Running = False
                Robot.stop(Stop.BRAKE)
                MMR.stop()
                MML.stop()

menu()