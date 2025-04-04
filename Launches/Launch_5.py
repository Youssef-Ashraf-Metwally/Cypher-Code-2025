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
    Header.PID(-600, 0, 650, -5, -0.1, -10, 10, DF=2)
    Header.MML.run_target(1000, 850, then=Stop.HOLD, wait=True)
    Header.PID(-100, 0, 150, -5, -0.1, -10, 10, DF=2)
    Header.Robot.drive(-300, 0)
    Header.wait(1000)
    Header.Robot.stop()
    Header.MML.run_target(1000, 400, then=Stop.HOLD)
    Header.PID(100, 0, 73, -5, -0.1, -10, 10)
    Header.P_Gyro_Turn(40, 7, 7)
    Header.Gyro_Check(40)
    Header.ev3.screen.print(Header.GS.angle())
    Header.PID(-700, 38, 60, -5, -0.1, -10, 30, doMcheck=False)
    Header.MMR.run_target(-10000, 3700, then=Stop.HOLD, wait=False)
    Header.MML.run_target(1000, 0, then=Stop.HOLD, wait=False)
    Header.PID(50, Header.GS.angle(), 170, -5, -0.1, -10, 15)
    Header.Motor_Check(170)
    Header.P_Gyro_Turn(-43, 7, 7)
    Header.PID(45, -43, 130, -5, -0.1, -10, 15)
    Header.MMR.run_target(-10000, 1000, then=Stop.HOLD)
    Header.PID(-100, -43, 190, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-65, 7, 7)
    Header.PID(-150, -65, 420, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(-180, 7, 7)
    Header.Gyro_Check(-180)
    Header.MMR.run_target(10000, 2500, then=Stop.HOLD)
    Header.PID(200, -180, 130, -5, -0.1, -10, 15)
    Header.PID(-200, -180, 50, -5, -0.1, -10, 15)
    Header.MMR.run_target(-10000, 0, then=Stop.HOLD)
    Header.Robot_Break()