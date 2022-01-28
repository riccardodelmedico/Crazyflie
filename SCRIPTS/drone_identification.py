import logging
import math
from Model_Identify import Model_Compensation
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
import Target as tar_c
import matplotlib.pyplot as plt


def chirp(t, f1, f0, T):
    c = (f1 - f0)/T
    return c*t * math.sin(2*math.pi*(c/2*t*t))
def step(t,val_ini,val_fin,T):
    if(t<T/2):
        return val_ini
    else:
        return val_fin
def ramp(t,m):
    return t*m
#50*c*t *
# #
# x = np.arange(0,10,0.001)
# y=np.array([chirp(x[i],10,0,10) for i in range(len(x))])
#
# plt.figure(1)
# plt.plot(x,y)
# plt.show()
# #
# with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
#     print('Main Start')
#     drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
#                          box=np.array([1.5, -2.0, 1.5, -1.0]))
#     ome_comm = np.array([])
#     times = 0
#     dt = 0.01
#     vx = 0.0
#     t_tot = 10
#     drone.start(0.0, 0.3)
#     for i in range(20):
#         drone.scf.cf.commander.send_position_setpoint(-0.5,0.0,sc_v.DEFAULT_HEIGHT,90)
#         time.sleep(0.1)
#     while times < t_tot:
#         # drone.send_command(0.5, 0.0, 0.0, 0.05)
#         #omega_comm = 60 * chirp(times, 10, 0, t_tot)
#         #omega_comm = step(times,0,120,t_tot)
#         omega_comm = ramp(times,36)
#         # print(omega_comm)
#         drone.send_command(vx, 0.0, omega_comm, dt)
#         times += dt
#
#
#     drone.landing()
#     print('Main Finished')
modello_p = Model_Compensation(np.array([0.0428651354557580,-0.0258394740829223,2.58351875693767e-06]),np.array([1,-1.73923437373173,0.756007018857389]))
time = np.arange(0,10,0.01)
all_input = np.array([])
all_output = np.array([])
for i in time:
    omega_in = step(i,0,30,10)
    omega_out = modello_p.compensation(omega_in)
    all_input = np.append(all_input,omega_in)
    all_output = np.append(all_output,omega_out)

plt.figure(1)
plt.plot(time,all_input)
plt.plot(time,all_output)
plt.show()
