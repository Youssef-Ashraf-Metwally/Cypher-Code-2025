#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
M1 = Motor(Port.A)
M2 = Motor(Port.B)
M3 = Motor(Port.C)
M4 = Motor(Port.D)

Motor = 1

# Write your program here.
def Stop_Motors(All):
    global Motor
    if Motor != 1 or All:
        M1.stop()
    if Motor != 2 or All:
        M2.stop()
    if Motor != 3 or All:
        M3.stop()
    if Motor != 4 or All:
        M4.stop()
def Control():
    Speed = 1000
    global Motor
    while True:
        buttons = ev3.buttons.pressed()
        ev3.screen.clear()
        ev3.screen.print(Motor)
        ev3.screen.print(Speed)
        if Button.CENTER in buttons:
            if Motor < 4:
                Motor += 1
            else:
                Motor = 1
            wait(50)
        if Button.RIGHT in buttons:
            if Motor == 1:
                M1.run(Speed)
                Stop_Motors(False)
            elif Motor == 2:
                M2.run(Speed)
                Stop_Motors(False)
            elif Motor == 3:
                M3.run(Speed)
                Stop_Motors(False)
            elif Motor == 4:
                M4.run(Speed)
                Stop_Motors(False)
        elif Button.LEFT in buttons:
            if Motor == 1:
                M1.run(Speed*-1)
                Stop_Motors(False)
            elif Motor == 2:
                M2.run(Speed*-1)
                Stop_Motors(False)
            elif Motor == 3:
                M3.run(Speed*-1)
                Stop_Motors(False)
            elif Motor == 4:
                M4.run(Speed*-1)
                Stop_Motors(False)
        else:
            Stop_Motors(True)
        if Button.UP in buttons:
            Speed += 100
        if Button.DOWN in buttons:
            if Speed > 100:
                Speed -= 100
        wait(100)

Control()   
