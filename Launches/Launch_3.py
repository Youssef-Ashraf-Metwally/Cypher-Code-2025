#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 3")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.P_Gyro_Turn(-56, 7, 7)
    Header.wait(200)
    Header.PID(-150, -56, 300, -5, -0.1, -10, 15)
    Header.PID_COLOR_L(-50, -56, 70, 2, -5, -0.1, -10, 15)
    Header.PID(-50, -56, 25, -5, -0.1, -10, 15)
    Header.wait(300)
    Header.P_Gyro_Turn(0, 5, 15)
    Header.wait(500)
    Header.MMR.run_target(-1000, -3700, then=Stop.HOLD, wait=False)
    Header.MML.run_target(-1000, -1300, then=Stop.HOLD, wait=False)
    Header.PID(-50, 0, 150, -6, -0.1, -10, 15)
    Header.PID_COLOR(-50, 0, 40, 3, -5, -0.1, -10, 15)
    Header.PID(-50, 0, 20, -6, -0.1, -10, 15)
    Header.MML.reset_angle(0)
    Header.MML.run_target(-1300, -1600, then=Stop.HOLD)
    Header.PID(-20, 0, 15, -6, -0.1, -10, 15)
    Header.wait(300)
    Header.MML.run_target(-5000, 1100, then=Stop.HOLD, wait=False)
    Header.PID(200, 0, 150, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(130, 7, 7)
    Header.PID(-500, 130, 1000, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False