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
    Header.PID(-500, -8, 500, -5, -0.1, -10, 20)
    Header.P_Gyro_Turn(0, 10, 10)
    Header.PID(-30, 0, 20, -5, -0.1, -10, 15)
    Header.MMR.run_time (-1000,1550,then=Stop.HOLD)
    Header.PID(-30, 0, 35, -5, -0.1, -10, 15)
    Header.MMR.run_time (1000,1450,then=Stop.HOLD)
    Header.P_Gyro_Turn(15, 10, 10)
    Header.PID(-100, 15, 120, -5, -0.1, -10, 15)
    Header.MML.run_time (-1000,1400,then=Stop.HOLD)
    Header.PID(50, 15, 105, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-61, 7, 7)
    Header.MML.run_time (1000,1400,then=Stop.HOLD)
    # Header.PID_COLOR(-300, -61, 15, 2, -4, -0.1, -10, 15)
    Header.PID(-300, -61, 390, -5, -0.1, -10, 15)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-90, 10, 10)
    Header.PID(-500, -90, 280, -5, -0.1, -10, 5)
    Header.MMR.run_time (-1000,1500,then=Stop.HOLD)
    Header.PID(-100, -90, 40, -5, -0.1, -10, 5)
    Header.PID(50, -90, 40, -5, -0.1, -10, 15)
    # Header.MMR.run_time (200,1500,then=Stop.HOLD, wait=False)
    Header.MML.run_time (1000,1500,then=Stop.HOLD)
    Header.PID(-300, -90, 260, -5, -0.1, -10, 15)
    Header.MMR.run_time (1000,2000,then=Stop.HOLD)
    Header.wait(500)
    Header.Gyro_Check(-87)
    Header.PID(-300, -87, 300, -5, -0.1, -10, 5)
    Header.wait(500)
    Header.MML.run_time (-1000,1500,then=Stop.HOLD)
    Header.MMR.run_time (-1000,1200,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-145, 12, 12)
    Header.PID(-500, -145, 900, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False

# Header.MMR.run_time (-1000,1500,then=Stop.HOLD)
# Header.wait(500)
# Header.MMR.run_time (1000,1500,then=Stop.HOLD)
