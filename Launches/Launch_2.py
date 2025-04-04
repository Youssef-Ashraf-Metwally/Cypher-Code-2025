#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 2")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.P_Gyro_Turn(0, 7, 7)
    Header.PID(-200, 0, 200, -5, -0.1, -10, 15)
    Header.PID_COLOR(-50, 0, 81, 2, -5, -0.1, -10, 15)
    Header.MML.run_target(700, 1600, then=Stop.HOLD, wait=False)
    Header.PID(-60, 1, 200, -5, -0.1, -10, 15)
    Header.MMR.run_target(1000, -800, then=Stop.HOLD)
    Header.wait(500)
    Header.MMR.run_target(1400, 1500, then=Stop.HOLD)
    Header.MMR.run_target(600, 400, then=Stop.HOLD)
    Header.MML.run_target(-300, 1350, then=Stop.HOLD)
    Header.Move_Steering(1000, 250, 0)
    Header.reset_all()
    Header.Gyro_Turn(-70, 500, -500)
    Header.Robot_Break()
    Header.Running = False