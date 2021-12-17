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
    # get the target position and velocity to calculate R Vc and \dot{Sigma}
    (t_pos,t_vel) = target.get_target()
    R = np.linalg.norm(t_pos[0:2]-pursuer.p[0:2],2)
    if R < 1e-3 and pursuer.ka_boom == None:
        pursuer.ka_boom = pursuer.time_line[-1] - pursuer.time_line[0]
        print(f'interncettazione avvenuta al tempo { pursuer.ka_boom}')

    Vc = - ((t_vel[0]-pursuer.v[0])*(t_pos[0]-pursuer.p[0])+(t_vel[1]-pursuer.v[1])*(t_pos[1]-pursuer.p[1]))/R
    dotSigma = ((t_vel[1] - pursuer.v[1]) * (t_pos[0] - pursuer.p[0]) - (t_vel[0] - pursuer.v[0]) * (t_pos[1] - pursuer.p[1])) /( R * R )

    # append these values to numpy to plot at the end their behavior on time
    pursuer.dotSigma = np.append(pursuer.dotSigma,np.array([dotSigma]))
    pursuer.Vc = np.append(pursuer.Vc,np.array([Vc]))
    pursuer.R = np.append(pursuer.R,np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line,time.time())

    #calculate PNG acceleration
    Ac = N * Vc * dotSigma

   #integrate velociticy and acceleration with forward euler
    if np.linalg.norm(pursuer.v,2) != 0:
        pursuer.p += pursuer.v * dt + target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))* Ac * dt * dt /2
        pursuer.v += target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))* Ac* dt
    pursuer.list_pos = np.concatenate((pursuer.list_pos,pursuer.p.reshape((1,3))),axis=0)

    #interception condition





class guidance():
    def __init__(self,target,inital_pose=np.array([0.0,0.0,0.5,math.pi/2]),chase_vel=0.2,N=3,Simulation_Guidance=True, dt=0.05):
        if Simulation_Guidance:
            # pursuer position  and velocity
            self.p = np.array(inital_pose[0:3])
            print(inital_pose[3])
            self.v = np.array([math.cos(inital_pose[3]),math.sin(inital_pose[3]),0.0])*chase_vel
            print(f'pursuer initial Pos:{self.p},initial Vel:{self.v}')

            #list of element for plot
            self.list_pos = self.p.reshape((1,3))
            self.update_thread = threading.Thread(target= tar_c.repeat_fun, args = (dt,sim_guidance,self,target,dt,N))
            self.dotSigma = np.array([])
            self.Vc = np.array([])
            self.R = np.array([])
            self.time_line = np.array([])
            # istant of interception
            self.ka_boom=None
        else:
            #section to code to guide the real drone
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
        self.time_line-=self.time_line[0]
    def plot_chase(self):

        if self.ka_boom != None:
            end = np.where(self.time_line == self.ka_boom)[0][0]
            print(end)
            self.list_pos = self.list_pos[0:end,:]
            self.target.list_pos = self.target.list_pos[0:end,:]
        plt.figure(1)
        plt.plot(self.list_pos[0,0],self.list_pos[0,1],'ro')
        plt.plot(self.list_pos[:,0], self.list_pos[:,1],'r-')
        plt.plot(self.list_pos[-1,0] , self.list_pos[-1,1],'r->')
        plt.plot(self.target.list_pos[0, 0], self.target.list_pos[0, 1], 'go')
        plt.plot(self.target.list_pos[:, 0], self.target.list_pos[:, 1], 'g-')
        plt.plot(self.target.list_pos[-1, 0], self.target.list_pos[-1, 1], 'g->')
        plt.show()
    def plot_chase_info(self):
        if self.ka_boom != None:
            end = np.where(self.time_line == self.ka_boom)[0][0]
            print(end)
            self.time_line = self.time_line[0:end]
            self.R = self.R[0:end]
            self.dotSigma = self.dotSigma[0:end]
            self.Vc = self.Vc[0:end]
        plt.figure(1)
        plt.plot(self.time_line, self.dotSigma[:], '-')
        plt.show()
        plt.figure(2)
        plt.plot(self.time_line, self.Vc[:], '-')
        plt.show()
        plt.figure(3)
        plt.plot(self.time_line, self.R[:], '-')
        plt.show()


a=tar_c.target(initial_position=np.array([2.0,2.0,0.5]),initial_velocity=np.array((0.1,0.1,0.0)),dt=0.005)
pur=guidance(a,chase_vel=0.5,dt=0.005,N=4)
pur.start()
time.sleep(10)
pur.stop()
pur.plot_chase()
pur.plot_chase_info()

print('finito il tutto ')


