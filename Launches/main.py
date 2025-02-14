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
    # print(GS.angle()) 
def Speed_Control(Speed, KA, D):
    global Current_Speed

    Initial_Speed = (Speed*(0.001*KA))
    print(Current_Speed, Speed)
    if abs(Current_Speed) < abs(Speed) and D == False:
        Current_Speed = Current_Speed + Initial_Speed
    elif D:
        # print("Decel")
        Current_Speed = Current_Speed - Speed*0.12
    else:
        Current_Speed = Speed
    return(Current_Speed)

def PID_COLOR (Speed, TA, TC, KP, KI, KD, KA):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    P_color = str(RCS.color())
    while str(RCS.color()) != str(TC):
        if interrupt_flag:
            break
        PID_WALK(Speed, TA, KP, KI, KD, KA)
    else:
        Robot.stop()
        #Robot_Break()
def PID (Speed, TA, TD, KP, KI, KD, KA):
    global interrupt_flag
    LM.reset_angle(0)
    RM.reset_angle(0)
    TD *= -1
    while abs(float(Robot.distance())) < abs(TD):
        D = not(abs(float(Robot.distance())) < abs(TD - 50))
        if interrupt_flag:
            break
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
def reset_all():
    GS.reset_angle(0)
    LM.reset_angle(0)
    RM.reset_angle(0)
    MML.reset_angle(0)
    MMR.reset_angle(0)

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
        PID(-200, 0, 430, -5, 0, 0)
        MMR.run_target(1000, -800, then=Stop.HOLD, wait=False)
        MML.run_target(550, 260, then=Stop.HOLD)
        wait(500)
        MML.run_target(2000, 0, then=Stop.HOLD, wait=False)
        MMR.run_target(1400, 1500, then=Stop.HOLD)
        MMR.run_target(500, 500, then=Stop.HOLD)
        PID(1000, 0, 200, -5, 0, 0)
        Gyro_Turn(-45, 1000, -1000)
        break
    Robot_Break()
def Launch_3():
    global interrupt_flag
    ev3.screen.print("Mission 3")
    while True:
        if interrupt_flag:
            break
        P_Gyro_Turn(15, 7, 7)
        PID(-1000, 15, 560, -5, 0, 0)
        P_Gyro_Turn(90, 0, 11)
        MMR.run_target(-1000, -3700, then=Stop.HOLD, wait=False)
        MML.run_target(-1000, -1500, then=Stop.HOLD, wait=False)
        PID(-50, 90, 190, -5, 0, 0)
        MML.reset_angle(0)
        MML.run_target(-1300, -1300, then=Stop.HOLD)
        MML.run_target(-5000, 1100, then=Stop.HOLD, wait=False)
        PID(200, 90, 150, -5, 0, 0)
        P_Gyro_Turn(30, 7, 7)
        PID(500, 30, 1000, -5, 0, 0)
        break
    Robot_Break()
def Launch_4():
    global interrupt_flag
    ev3.screen.print("Mission 3")
    while True:
        if interrupt_flag:
            break
        GS.reset_angle(0)
        PID(200, 0, 150, -5, -0.1, -10, 15)
        PID(-200, 0, 100, -5, -0.1, -10, 15)
        P_Gyro_Turn(92, 10, 10)
        PID(-300, 92, 175, -5, -0.1, -10, 15)
        wait(500)
        P_Gyro_Turn(182, 10, 10)
        PID(-1000, 182, 335, -5, -0.1, -10, 15)
        wait(500)
        P_Gyro_Turn(132, 10, 10)
        PID(-80, 132, 125, -5, -0.1, -10, 15)
        MMR.run_target(-80, -200, then=Stop.HOLD, wait=False)
        MML.run_target(-1000, -2500, then=Stop.HOLD, wait=True)
        PID(200, 132, 100, -5, -0.1, -10, 15)
        P_Gyro_Turn(182, 7, 7)
        PID(-500, 182, 230, -5, -0.1, -10, 15)
        # MMR.run_target(-50, 270, then=Stop.HOLD, wait=True)
        # PID(500, 180, 150, -5, -0.1, -10, 15)
        # MMR.run_target(-50, 130, then=Stop.HOLD, wait=True)
        # P_Gyro_Turn(160, 7, 7)
        # PID(-200, 160, 260, -5, -0.1, -10, 15)
        # P_Gyro_Turn(180, 7, 7)
        # PID(-500, 180, 435, -5, -0.1, -10, 15)
        # P_Gyro_Turn(225, 7, 7)
        # PID(200, 225, 350, -5, -0.1, -10, 15)
        # wait(500)
        # PID(-500, 225, 400, -5, -0.1, -10, 15)
        break
    Robot_Break()
def Launch_5():
    global interrupt_flag
    ev3.screen.print("Mission 5")
    while True:
        if interrupt_flag:
            break
        reset_all()
        P_Gyro_Turn(-15, 7, 7)
        PID(-1000, -15, 395, -5, 0, 0)
        MML.run_target(-1000, -340, then=Stop.HOLD, wait=False)
        P_Gyro_Turn(0, 7, 7)
        Gyro_Check(0)
        PID(-500, 0, 500, -5, 0, 0)
        MML.run_target(-500, 450, then=Stop.HOLD)
        PID(500, 0, 60, -5, 0, 0)
        P_Gyro_Turn(45, 7, 7)
        PID_COLOR(-50, 45, Color.BLACK, -5, 0, 0)
        wait(500)
        LM.run_time(-1000, 1000, then=Stop.HOLD, wait=False)
        RM.run_time(-1000, 1000, then=Stop.HOLD)
        MMR.run_target(-10000, 3500, then=Stop.HOLD, wait=False)
        PID(100, 45, 180, -5, 0, 0)
        P_Gyro_Turn(-45, 7, 7)
        PID(50, -45, 140, -5, 0, 0)
        MMR.run_target(-10000, -1000, then=Stop.HOLD)
        PID(-100, -45, 260, -5, 0, 0)
        P_Gyro_Turn(-75, 0, 7)
        PID_COLOR(-500, -75, Color.WHITE, -5, 0, 0)
        P_Gyro_Turn(-180, 7, 7)
        MMR.run_target(-10000, -2000, then=Stop.HOLD)
        break
    Robot_Break()

def run_mission():
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

reset_all()
Launch_4()