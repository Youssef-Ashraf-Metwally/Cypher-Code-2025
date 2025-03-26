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
import Header
import Launch_1
import Launch_2
import Launch_3
import Launch_4
import Launch_5

ev3 = EV3Brick()
GS = GyroSensor(Port.S2)
RM = Motor(Port.B)
LM = Motor(Port.C)
MML = Motor(Port.D)
MMR = Motor(Port.A)
RCS = ColorSensor(Port.S3)
LCS = ColorSensor(Port.S1)
Robot = DriveBase(LM, RM, 54.5, 140)


def menu():
    Current_Launch = 0
    Launch_Names = ["Launch 1", "Launch 2", "Launch 3", "Launch 4", "Launch 5"]
    Launches = [Launch_1.Mission, Launch_2.Mission, Launch_3.Mission, Launch_4.Mission, Launch_5.Mission]
    Header.Running = False
    while True:
            buttons = ev3.buttons.pressed()
            if not Header.Running:
                ev3.screen.clear()
                ev3.screen.print("Anything")
                for i, name in enumerate(Launch_Names):
                    if i == Current_Launch:
                        ev3.screen.print("<> " + name)
                    else:
                        ev3.screen.print(name)  
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
            if Button.CENTER in buttons:
                mission_thread = Thread(target=Launches[Current_Launch])
                mission_thread.start()
                Header.Running = True           
            if Button.LEFT in buttons:
                Header.Running = False
            wait(100)
menu()