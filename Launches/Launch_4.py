#!/usr/bin/env pybricks-micropython
import Header
import threading
from threading import Thread
from pybricks.robotics import Stop

#Launches.
def Mission():
    Header.ev3.screen.clear()
    Header.ev3.screen.print("Mission 4")
    Header.interrupt_flag = False
    interrupt_thread = Thread(target=Header.check_interrupt)
    interrupt_thread.start()
    Header.reset_all()
    Header.P_Gyro_Turn(0, 7, 7)
    Header.reset_all()
    Header.GS.reset_angle(0)
    Header.MMR.run(-50)
    Header.PID(-400, 0, 900, -5, -0.1, -10, 15)
    Header.MMR.stop()
    Header.wait(500)
    Header.MMR.reset_angle(0)
    Header.MMR.run_target(1600, 140, then=Stop.HOLD, wait=True)
    Header.MMR.run_target(-700, -100, then=Stop.HOLD, wait=True)
    Header.MMR.run_time(1600, 1200, then=Stop.HOLD, wait=True)
    Header.wait(500)
    Header.MMR.run_time(-500, 1200, then=Stop.HOLD, wait=True)
    Header.MML.run_target(1000, 650, then=Stop.HOLD, wait=True)
    Header.PID(400, 0, 350, -5, -0.1, -10, 15)
    Header.MML.run_target(-1000, -300, then=Stop.HOLD, wait=True)
    Header.MMR.run_time(1600, 1000, then=Stop.HOLD, wait=False)
    Header.P_Gyro_Turn(-23, 10, 10)
    Header.PID(-300, -23, 150, -5, -0.1, -10, 15)
    Header.MMR.run_target(-1600, -100, then=Stop.HOLD, wait=True)
    Header.PID(-300, -23, 270, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(0, 10, 10)
    Header.PID(-400, 0, 400, -5, -0.1, -10, 15)
    Header.P_Gyro_Turn(45, 10, 10)
    Header.MML.run_target(1000, 200, then=Stop.HOLD, wait=True)
    Header.PID(500, 45, 250, -5, -0.1, -10, 15)
    Header.PID(-450, 35, 400, -5, -0.1, -10, 15)
    Header.Robot_Break()
    Header.Running = False