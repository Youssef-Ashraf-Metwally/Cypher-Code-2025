GS.reset_angle(0)
P_Gyro_Turn(-5, 7, 7)
PID (-1000, -5, 420, -5, 0, 0)
P_Gyro_Turn(0, 7, 7)
MMR.run_time (-1000,1700,then=Stop.HOLD)
PID (-30, 0, 50, -5, 0, 0)
MMR.run_time (1000,1700,then=Stop.HOLD)
P_Gyro_Turn(10, 7, 7)
PID (-100, 10, 100, -5, 0, 0)
P_Gyro_Turn(-73, 9, 9)
PID (-1000, -73, 290, -5, 0, 0)
P_Gyro_Turn(-90, 9, 9)
Gyro_Check(-90)
PID (-1000, -90, 240, -5, 0, 0)
MMR.run_time (-1000,1600,then=Stop.HOLD)
PID (-500, -90, 105, -5, 0, 0)
MMR.run_time (1000,1600,then=Stop.HOLD)
PID (-500, -90, 160, -5, 0, 0)
P_Gyro_Turn(-130, 7, 7)
MMR.run_time (-1000,500,then=Stop.HOLD)
PID (-500, -130, 10000, -5, 0, 0)