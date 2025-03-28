#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 5")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.PID(-300, 0, 200, -5, -0.1, -10, 10)
    # Header.PID_COLOR_L(-50, 0, 82, 2, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-35, 7, 7)
    Header.wait(200)
    Header.PID(-300, -35, 190, -5, -0.1, -10, 10)
    # Header.PID_COLOR_L(-100, -35, 19, 2, -5, -0.1, -10, 15)
    Header.wait(200)
    Header.P_Gyro_Turn(0, 5, 20)
    Header.Gyro_Check(0)
    # Header.MML.run_target(500, 340, then=Stop.HOLD, wait=False)
    Header.PID(-500, 0, 500, -5, -0, -10, 7, DF=5)
    Header.GS.reset_angle(0)
    Header.LM.reset_angle(0)
    Header.RM.reset_angle(0)
    Header.MML.run_target(300, -800, then=Stop.HOLD)
    Header.PID(200, 0, 124, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(45, 7, 7)
    Header.wait(500)
    Header.ev3.screen.print("Waiting")
    Header.PID(-50, 45, 90, -1.5, 0, -10, 1)
    # Header.Move_Steering(-50, 90, 0)
    Header.Robot.straight(-90)
    Header.wait(500)
    Header.Robot.stop()
    Header.PID(-700, 45, 40, -5, -0.1, -10, 30)
    Header.MMR.run_target(-10000, 3700, then=Stop.HOLD, wait=False)
    # Header.PID_COLOR(50, 45, 9, 2, -5, -0.1, -10, 15)
    # Header.PID_COLOR(50, 45, 88, 2, -5, -0.1, -10, 15)
    Header.PID(50, Header.GS.angle(), 200, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-43, 7, 7)
    Header.wait(300)
    Header.PID(45, -43, 160, -5, -0.1, -10, 15)
    Header.MMR.run_target(-10000, 1000, then=Stop.HOLD)
    Header.PID(-100, -43, 250, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-65, 7, 7)
    Header.PID(-150, -65, 330, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-180, 7, 7)
    Header.Gyro_Check(-180)
    Header.MMR.run_target(10000, 2500, then=Stop.HOLD)
    Header.PID(200, -180, 130, -5, -0.1, -10, 15)
    Header.MMR.run_target(-10000, 0, then=Stop.HOLD)
    Header.PID(-200, -180, 50, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False