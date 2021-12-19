import logging
import math
import threading
import time
import numpy as np

import target_class as tg
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
#from own_module import crazyfun as crazy, script_setup as sc_s, \
  #  script_variables as sc_v
#from vicon_dssdk import ViconDataStream
import target_class as tar_c

import matplotlib.pyplot as plt
import DroneManager

def drone_guidance(pursuer,dt,N):
    # get the target position and velocity to calculate R Vc and \dot{Sigma}
    (t_pos,t_vel) = pursuer.target.get_target()
    pursuer.get_state()
    R = np.linalg.norm(t_pos[0:2]-pursuer.p[0:2],2)
    pursuer.R = np.append(pursuer.R, np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line, time.time())
    if R < 1e-2 and pursuer.ka_boom == None:

        pursuer.ka_boom = pursuer.time_line[-1] - pursuer.time_line[0]
        print(f"interncettazione avvenuta al tempo { pursuer.time_line[- 1]- pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}" )
        tar_c.run = False
        pursuer.drone.landing()

    Vc =  - ((t_vel[0]-pursuer.v[0])*(t_pos[0]-pursuer.p[0])+(t_vel[1]-pursuer.v[1])*(t_pos[1]-pursuer.p[1]))/R
    dotSigma = ((t_vel[1] - pursuer.v[1]) * (t_pos[0] - pursuer.p[0]) - (t_vel[0] - pursuer.v[0]) * (t_pos[1] - pursuer.p[1])) /( R * R )

    # append these values to numpy to plot at the end their behavior on time
    pursuer.dotSigma = np.append(pursuer.dotSigma,np.array([dotSigma]))
    pursuer.Vc = np.append(pursuer.Vc,np.array([Vc]))
    #calculate PNG acceleration
    Ac = N * Vc * dotSigma

    if len(pursuer.time_line) <2:
        deltat=dt
    else:
        deltat = pursuer.time_line[-1] - pursuer.time_line[-2]
   #integrate velociticy and acceleration with forward euler
    if not pursuer.send_command():
        pursuer.drone.landing()
    pursuer.list_pos = np.concatenate((pursuer.list_pos,pursuer.p.reshape((1,3))),axis=0)

 #interception condition

class DroneGuidance:
    def __init__(self,target,droneM,initial_velocity=np.array([0.0,0.1,0.0]),dt= 0.05,N=3):
        self.ka_boom = None
        self.update_thread=threading.Thread(target= tar_c.repeat_fun,args= (dt,drone_guidance,self,N))
        self.pos_list = np.array([])
        self.p = np.array([])
        self.v = initial_velocity
        self.drone = droneM
        self.target = target
        self.R = np.array([])
        self.Vc = np.array([])
        self.dotSigma = np.array([])
        self.time_line = np.array([])


    def start(self):
        self.drone.take_off(self.v[0:2])
        #take off deve eseguire finche non rispetta le condizioni di lancio
        #della guida, quindi V circa self.v ed il drone si trova dentro la box
        tar_c.run = True
        self.target.update_thread.start()
        self.update_thread.start()

    #def stop(self):
    #    tar_c.run = False
    #    self.update_thread.join()
    #    self.target.update_thread.join()

    def send_command(self,vx,vy,omega):
        ret = self.drone.send_command(vx,vy,omega)
        return ret
    def get_state(self):
        #crazy.callback_mutex.acquire(blocking=True)
        #self.v = sc_v.vel_estimate[0:3]
        #self.p = sc_v.pos_estimate[0:3]
        #crazy.callback_mutex.release()
        pass