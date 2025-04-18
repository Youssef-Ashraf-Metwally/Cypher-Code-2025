#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.

def Mission_1():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 1")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.Gyro_Sensor.reset_angle(0)
    Header.P_Gyro_Turn(-13, 10, 10)
    Header.PID(-500, -13, 340, -5, -0.1, -10, 20)
    # Header.Gyro_Check(-8)
    Header.P_Gyro_Turn(0, 10, 10)
    Header.PID(-200, 0, 270, -5, -0.1, -10, 15)
    Header.Right_Motor_M.run_target (1000,-1100,then=Stop.HOLD)
    Header.Left_Motor_M.run_target (1000,-900,then=Stop.HOLD, wait=False)
    Header.PID(-10, 0, 40, -5, -0.1, -10, 15)
    # Header.PID(-30, 0, 35, -5, -0.1, -10, 15)
    Header.Right_Motor_M.run_target (1000,0,then=Stop.HOLD)
    # Header.PID(-200, 0, 100, -5, -0.1, -10, 15)
    # Header.Gyro_Check(0)
    # Header.Right_Motor_M.run_time (1000,800,then=Stop.HOLD, wait=True)
    Header.Right_Motor_M.run_target (1000,-200,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-76, 6, 6)
    # Header.Gyro_Check(-74)
    # Header.Left_Motor_M.run_time (1000,1400,then=Stop.HOLD)
    Header.PID(-400, -76, 380, -5, -0.1, -10, 15)
    # Header.Left_Motor_M.run_time (-1000,1500,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-90, 6, 6)
    Header.Gyro_Check(-90)
    Header.PID(-500, -90, 290, -5, -0.1, -10, 7)
    Header.Right_Motor_M.run_target (1000,-1100,then=Stop.HOLD)
    # Header.Left_Motor_M.run_time (1000,1500,then=Stop.HOLD, wait=False)
    Header.Right_Motor_M.run_target (1000,0,then=Stop.HOLD, wait=False)
    Header.PID(-400, -90, 305, -5, -0.1, -10, 15)
    Header.wait(1000)
    Header.P_Gyro_Turn(-92, 6, 6)
    Header.PID(-400, -92, 200, -5, -0.1, -10, 5)
    Header.Left_Motor_M.run_time (-1000,1500,then=Stop.HOLD)
    Header.Right_Motor_M.run_time (-1000,1200,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-145, 12, 12)
    Header.PID(-500, -145, 900, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False

def Init():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 1.5")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.PID(-100, 0, 50, -5, -0.1, -10, 10)
    Header.wait(200)
    Header.PID(100, 0, 50, -5, -0.1, -10, 10)



def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 1")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.Left_Motor_M.reset_angle(-70)
    Header.Right_Motor_M.run_target (1000,500,then=Stop.HOLD, wait=False)
    Header.Left_Motor_M.run_target (1000,-1040,then=Stop.HOLD, wait=False)
    Header.PID(-500, 0, 760, -4, -0.1, -10, 7)
    Header.Left_Motor_M.run_target (500,0,then=Stop.HOLD, wait=False)
    Header.Right_Motor_M.run_target (500,-450,then=Stop.HOLD)
    Header.Right_Motor_M.run_target (500,500,then=Stop.HOLD)
    Header.P_Gyro_Turn(-90, 7, 7)
    Header.Gyro_Check(-90)
    Header.Right_Motor_M.run_target (150,0,then=Stop.HOLD, wait=False)
    Header.Left_Motor_M.run_target (1000,-1050,then=Stop.HOLD, wait=False)
    Header.PID(-500, -90, 520, -3.5, -0.1, -10, 5)
    Header.Left_Motor_M.run_target (1000,-850,then=Stop.HOLD, wait=False)
    Header.PID(-300, -90, 120, -5, -0.1, -10, 10)
    Header.Right_Motor_M.run_target (500,-610,then=Stop.HOLD)
    Header.PID(-300, -90, 150, -5, -0.1, -10, 10)
    Header.Right_Motor_M.run_target (1000,200,then=Stop.HOLD)
    Header.Left_Motor_M.run_target (1000,-1000,then=Stop.HOLD, wait=False)
    Header.Right_Motor_M.run_target (1000,350,then=Stop.HOLD, wait=False)
    Header.PID(-500, -88, 390, -5, -0.1, -10, 20, DF=2)
    # Header.Right_Motor_M.run_target (1000,0,then=Stop.HOLD, wait=False)
    Header.Left_Motor_M.run_target (1000,0,then=Stop.HOLD)
    # Header.Gyro_Turn(-150, 500, -100)
    Header.Right_Motor_M.run_target (1000,500,then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-150, 8, 8)
    Header.Left_Motor_M.run_target (1500,-900,then=Stop.HOLD)
    Header.PID(-500, -150, 650, -5, -0.1, -10, 20, 1.5)
