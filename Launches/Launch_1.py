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
    Header.P_Gyro_Turn(-8, 10, 10)
    Header.PID(-500, -8, 450, -5, -0.1, -10, 20, 4)
    Header.Gyro_Check(-8)
    Header.P_Gyro_Turn(0, 10, 10)
    Header.PID(-30, 0, 65, -5, -0.1, -10, 15)
    Header.MMR.run_time (-1000,1550,then=Stop.HOLD)
    Header.PID(-30, 0, 35, -5, -0.1, -10, 15)
    Header.MMR.run_time (1000,900,then=Stop.HOLD)
    Header.PID(-200, 0, 100, -5, -0.1, -10, 15)
    Header.Gyro_Check(0)
    Header.MML.run_time (-1000,1400,then=Stop.HOLD)
    Header.MMR.run_time (1000,800,then=Stop.HOLD, wait=True)
    Header.P_Gyro_Turn(-74, 6, 0)
    Header.Gyro_Check(-74)
    Header.MML.run_time (1000,1400,then=Stop.HOLD)
    Header.PID(-400, -74, 380, -5, -0.1, -10, 15)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-90, 6, 6)
    Header.Gyro_Check(-90)
    Header.PID(-500, -90, 235, -5, -0.1, -10, 20, 4)
    Header.MMR.run_time (-1000,1650,then=Stop.HOLD)
    Header.MMR.run_time (1000,2000,then=Stop.HOLD, wait=False)
    Header.PID(-400, -90, 290, -5, -0.1, -10, 15)
    Header.MML.run_time (1000,1500,then=Stop.HOLD)
    Header.P_Gyro_Turn(-99, 6, 6)
    Header.PID(-400, -99, 200, -5, -0.1, -10, 5)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD)
    Header.MMR.run_time (-1000,1200,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-145, 12, 12)
    Header.PID(-500, -145, 900, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False