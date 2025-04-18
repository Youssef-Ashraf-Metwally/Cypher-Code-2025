#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Init():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 1.5")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.P_Gyro_Turn(-45, 7, 7)
    Header.wait(200)
    Header.PID(-500, -45, 550, -5, -0.1, -10, 10)
    Header.Left_Motor_M.run_target (1000,-500, then=Stop.HOLD)
    Header.PID(500, -45, 700, -5, -0.1, -10, 10, DF=1.5)

def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 3")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.P_Gyro_Turn(-60, 7, 7)
    Header.wait(200)
    Header.PID(-500, -60, 435, -5, -0.1, -10, 10)
    Header.wait(300)
    Header.P_Gyro_Turn(0, 5, 15)
    Header.wait(500)
    Header.Right_Motor_M.run_target(-1000, -3700, then=Stop.HOLD, wait=False)
    Header.Left_Motor_M.run_target(-1000, 325, then=Stop.HOLD, wait=False)
    Header.PID_TIME(-50, 0, 4500, -5, -0.1, -10, 15)
    Header.Left_Motor_M.reset_angle(0)
    Header.Left_Motor_M.run_time(1300, 2000, then=Stop.HOLD)
    Header.wait(1000)
    Header.PID(50, 0, 20, -5, -0.1, -10, 10)
    Header.Left_Motor_M.run_target(-5000, -300, then=Stop.HOLD, wait=True)
    Header.PID(200, 0, 170, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(130, 15, 15)
    Header.PID(-500, 130, 500, -5, -0.1, -10, 20)
    Header.Robot_Break()
    Header.Running = False