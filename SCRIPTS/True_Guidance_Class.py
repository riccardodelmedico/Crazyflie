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
def sim_guidance(pursuer,target,dt,N):
    (t_pos,t_vel) = target.get_target()
    R = np.linalg.norm(t_pos[0:2]-pursuer.p[0:2],2)
    Vc = ((t_vel[0]-pursuer.v[0])*(t_pos[0]-pursuer.p[0])+(t_vel[1]-pursuer.v[1])*(t_pos[1]-pursuer.p[1]))/R
    dotSigma = ((t_vel[1] - pursuer.v[1]) * (t_pos[0] - pursuer.p[0]) - (t_vel[0] - pursuer.v[0]) * (t_pos[1] - pursuer.p[1])) /( R * R )
    Ac = N * Vc * dotSigma
    if np.linalg.norm(pursuer.v,2) != 0:
        pursuer.p += pursuer.v * dt + target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))*pursuer.a* dt * dt /2
        pursuer.v += target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))*pursuer.a* dt
    pursuer.list_pos = np.concatenate((pursuer.list_pos,pursuer.p.reshape((1,3))),axis=0)


class guidance():
    def __init__(self,target,inital_pose=np.array([0.0,0.0,0.5,90]),chase_vel=0.2,N=3,Simulation_Guidance=True, dt=0.05):
        if Simulation_Guidance:
            self.p = np.array(inital_pose[0:3])
            self.v = np.array([math.cos(inital_pose[3]),math.sin(inital_pose[3]),0.0])*chase_vel
            self.a = 0
            self.list_pos = self.p.reshape((1,3))
            self.update_thread = threading.Thread(target= tar_c.repeat_fun, args = (dt,sim_guidance,self,target,dt,N))
        else:
            self.pose0 = inital_pose
            self.chase_vel = chase_vel
        self.target = target
        self.N = N
    def start(self):
        tar_c.run=True
        self.update_thread.start()
        self.target.start()
    def stop(self):
        tar_c.run=False
        self.update_thread.join()
        self.target.stop()
    def plot(self):
        plt.figure(1)
        plt.plot(self.list_pos[0,0],self.list_pos[0,1],'o')
        plt.plot(self.list_pos[:,0], self.list_pos[:,1],'-')
        plt.plot(self.list_pos[-1,0] , self.list_pos[-1,1],'->')
        plt.plot(self.target.list_pos[0, 0], self.target.list_pos[0, 1], 'o')
        plt.plot(self.target.list_pos[:, 0], self.target.list_pos[:, 1], '-')
        plt.plot(self.target.list_pos[-1, 0], self.target.list_pos[-1, 1], '->')
        plt.show()
a=tar_c.target(initial_position=np.array([2.0,2.0,0.5]))
pur=guidance(a)
pur.start()
time.sleep(100)
pur.stop()
pur.plot()
print('finito il tutto ')


