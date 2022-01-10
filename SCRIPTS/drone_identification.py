import logging
import math
import threading
import time
import numpy as np
from DroneManager import DroneManager
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
#from vicon_dssdk import ViconDataStream
import target_class as tar_c
import matplotlib.pyplot as plt


def chirp(t, f1, f0, T):
    c = (f1 - f0)/T
    return math.sin(2*math.pi*(c/2*t*t))

# #
# x = np.arange(0,10,0.0001)
# y=np.array([chirp(x[i],20,0,10) for i in range(len(x))])
#
# plt.figure(1)
# plt.plot(x,y)
# plt.show()
# #
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.0, -1.0, 1.0, -1.0]))
    ome_comm = np.array([])
    times = 0
    dt = 0.05
    vx = 0.0
    t_tot = 20
    drone.start(0.0, 0.3)
    for i in range(20):
        drone.scf.cf.commander.send_position_setpoint(0.0,0.0,sc_v.DEFAULT_HEIGHT,90)
        time.sleep(0.1)
    while times < 20:
        # drone.send_command(0.5, 0.0, 0.0, 0.05)
        omega_comm = 90 * chirp(times, 20, 0, t_tot)
        print(omega_comm)
        drone.send_command(vx, 0.0, omega_comm, dt)
        times += dt
    drone.landing()
    print('Main Finished')
