#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 1")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.GS.reset_angle(0)
    Header.P_Gyro_Turn(-7, 10, 10)
    Header.PID(-1000, -7, 410, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(0, 10, 10)
    Header.PID(-30, 0, 10, -5, -0.1, -10, 15)
    Header.MMR.run_time (-1000,1500,then=Stop.HOLD)
    Header.PID(-30, 0, 30, -5, -0.1, -10, 15)
    Header.MMR.run_time (1000,1500,then=Stop.HOLD)
    Header.P_Gyro_Turn(10, 10, 10)
    Header.PID(-100, 10, 100, -5, -0.1, -10, 15)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD)
    Header.PID(50, 10, 70, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-61, 7, 7)
    Header.MML.run_time (1000,1500,then=Stop.HOLD)
    Header.PID_COLOR(-300, -61, 15, 2, -4, -0.1, -10, 15)
    Header.PID(-50, -61, 140, -5, -0.1, -10, 15)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-90, 10, 10)
    Header.Gyro_Check(-90)
    Header.PID(-1000, -90, 260, -5, -0.1, -10, 15)
    Header.MMR.run_time (-1000,1600,then=Stop.HOLD)
    Header.PID(50, -90, 35, -5, -0.1, -10, 15)
    Header.MML.run_time (1000,1500,then=Stop.HOLD)
    Header.PID(-500, -90, 130, -5, -0.1, -10, 15)
    Header.MMR.run_time (1000,2000,then=Stop.HOLD)
    Header.wait(500)
    Header.PID(-500, -90, 260, -5, -0.1, -10, 15)
    Header.wait(500)
    Header.P_Gyro_Turn(-145, 12, 12)
    Header.MMR.run_time (-1000,1200,then=Stop.HOLD)
    Header.PID(-500, -145, 10000, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False