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


def drone_guidance(pursuer,dt,N):
    # get the target position and velocity to calculate R Vc and \dot{Sigma}
    (t_pos,t_vel) = pursuer.target.get_target()
    pursuer.drone.get_state()
    R = np.linalg.norm(t_pos[0:2]-pursuer.drone.position[0:2],2)
    pursuer.R = np.append(pursuer.R, np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line, time.time())
    if R < 5e-2 and pursuer.ka_boom == None:

        # salva ka_boom come true guidance
        pursuer.ka_boom = len(pursuer.R)
        print(
            f"intercettazione avvenuta al tempo {pursuer.time_line[pursuer.ka_boom - 1] - pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}")
        pursuer.drone.landing()

    Vc = - ((t_vel[0]-pursuer.drone.velocity[0])*(t_pos[0]-pursuer.drone.position[0])+(t_vel[1]-pursuer.drone.velocity[1])*(t_pos[1]-pursuer.drone.position[1]))/R
    dotSigma = ((t_vel[1] - pursuer.drone.velocity[1]) * (t_pos[0] - pursuer.drone.position[0]) - (t_vel[0] - pursuer.drone.velocity[0]) * (t_pos[1] - pursuer.drone.position[1])) /( R * R )

    # append these values to numpy to plot at the end their behavior on time
    pursuer.dotSigma = np.append(pursuer.dotSigma,np.array([dotSigma]))
    pursuer.Vc = np.append(pursuer.Vc,np.array([Vc]))
    #calculate PNG acceleration
    Ac = N * Vc * dotSigma
    omega = math.degrees(Ac/np.linalg.norm(pursuer.drone.velocity[0:2],2))
    #print(f'la velocità angolare vale {omega}, ')
    pursuer.send_command(pursuer.v[0],0.0,omega)
    #pursuer.list_pos = np.concatenate((pursuer.list_pos,pursuer.drone.position.reshape((1,2))),axis=0)

 #interception condition

class DroneGuidance:
    def __init__(self,target,droneM,initial_velocity=np.array([0.1,0.0,0.0]),dt= 0.05,N=3):
        self.ka_boom = None
        self.update_thread=threading.Thread(target= crazy.repeat_fun,args= (dt,drone_guidance,self,dt,N))
        self.list_pos = np.array([])
        self.p = np.array([])
        self.v = initial_velocity
        self.drone = droneM
        self.target = target
        self.R = np.array([])
        self.Vc = np.array([])
        self.dotSigma = np.array([])
        self.time_line = np.array([])


    def start(self,vx,vy):
        self.drone.start(vx,vy)
        #take off deve eseguire finche non rispetta le condizioni di lancio
        #della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')
        crazy.run = True
        self.target.update_thread.start()
        self.update_thread.start()

    def stop(self):
       # tar_c.run = False
       self.update_thread.join()
       self.target.update_thread.join()

    def send_command(self, vx, vy, omega):
        self.drone.send_command(vx, vy, omega)

    def plot_chase(self):
        if self.ka_boom != None:
            self.target.time_line-=self.target.time_line[0]
            #print(self.target.time_line)
            stop = self.time_line[self.ka_boom]
            #print(self.time_line[self.ka_boom])
            len_of_time=np.where(self.target.time_line<= self.time_line[self.ka_boom],self.target.time_line,0)
            i=0
            #print(len_of_time)
            for j in range(len(len_of_time)):
                if len_of_time[j]!=0:
                    i+=1
            #print(f'la i vale{i}')
            self.list_pos = self.list_pos[0:self.ka_boom, :]
            self.target.list_pos = self.target.list_pos[0:i, :]
        plt.figure(1)
        plt.plot(self.list_pos[0,0],self.list_pos[0,1],'ro')
        plt.plot(self.list_pos[:,0], self.list_pos[:,1],'r-')
        plt.plot(self.list_pos[-1,0] , self.list_pos[-1,1],'r->')
        plt.plot(self.target.list_pos[0, 0], self.target.list_pos[0, 1], 'go')
        plt.plot(self.target.list_pos[:, 0], self.target.list_pos[:, 1], 'g-')
        plt.plot(self.target.list_pos[-1, 0], self.target.list_pos[-1, 1], 'g->')
        plt.show()


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    target = tar_c.target(dt=0.01)
    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0)
    guidance = DroneGuidance(target, drone,initial_velocity= np.array([1.0, 0.0, 0.0]),N=4)
    guidance.start(0.0, 0.4)
    guidance.stop()
    print(np.min(guidance.R))
    print('Main Finished')