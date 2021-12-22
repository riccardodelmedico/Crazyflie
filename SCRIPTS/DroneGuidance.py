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

first_iter = True

def drone_guidance_v3(pursuer, dt, N):
    global first_iter,old_px,old_py
    (t_pos, t_vel) = pursuer.target.get_target()
    pursuer.drone.get_state()
    if first_iter:
        old_px = t_pos[0]
        old_py = t_pos[1]
        first_iter = False
    else:
        cos_yaw = math.cos(math.radians(pursuer.drone.yaw))
        sin_yaw = math.sin(math.radians(pursuer.drone.yaw))
        rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        print(f'le velocità reali sono {t_vel}')
        t_velx = (t_pos[0] - old_px)/dt
        t_vely = (t_pos[1] - old_py)/dt
        old_px = t_pos[0]
        old_py = t_pos[1]
        print(f'le velocità stimate sono {t_velx,t_vely}')
        pursuer.drone.velocity = rot_yaw.dot(pursuer.drone.velocity)
        R = np.linalg.norm(t_pos[0:2] - pursuer.drone.position[0:2], 2)
        pursuer.R = np.append(pursuer.R, np.array([R]))
        pursuer.time_line = np.append(pursuer.time_line, time.time())
        if R < 5e-2 and pursuer.ka_boom == None:
            pursuer.drone.datalog.stop()
            # salva ka_boom come true guidance
            pursuer.ka_boom = len(pursuer.R)
            print(pursuer.ka_boom, len(pursuer.time_line))
            print(
                f"intercettazione avvenuta al tempo {pursuer.time_line[pursuer.ka_boom - 1] - pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}")
            pursuer.drone.landing()
        sigma = math.atan2(t_pos[1] - pursuer.drone.position[1],
                           t_pos[0] - pursuer.drone.position[0])
        Vc = - ((t_velx - pursuer.drone.velocity[0]) * (
                    t_pos[0] - pursuer.drone.position[0]) + (
                            t_vely - pursuer.drone.velocity[1]) * (
                            t_pos[1] - pursuer.drone.position[1])) / R
        dotSigma = ((t_vely - pursuer.drone.velocity[1]) * (
                    t_pos[0] - pursuer.drone.position[0]) - (
                                t_vely - pursuer.drone.velocity[0]) * (
                                t_pos[1] - pursuer.drone.position[1])) / (R * R)

        # append these values to numpy to plot at the end their behavior on time
        pursuer.dotSigma = np.append(pursuer.dotSigma, np.array([dotSigma]))
        pursuer.Vc = np.append(pursuer.Vc, np.array([Vc]))
        N_tv = N / math.cos(sigma - math.radians(pursuer.drone.yaw))
        # calculate PNG acceleration
        Ac = N_tv * Vc * dotSigma
        omega = - math.degrees(Ac / np.linalg.norm(pursuer.drone.velocity[0:2], 2))
        # omega saturation
        if omega > 90:
            omega = 90
        elif omega < -90:
            omega = -90
        # print(f'la velocità angolare vale {omega}, ')
        crazy.guidance_matlab.write(t_pos[0], t_pos[1], R, Vc, dotSigma)
        pursuer.send_command(pursuer.v, 0.0, omega, dt=dt)
        # if pursuer.list_pos[0] == -100.0:
        #     pursuer.list_pos = pursuer.drone.position.reshape((1,2))
        # else:
        pursuer.list_pos = np.concatenate(
            (pursuer.list_pos, pursuer.drone.position.reshape((1, 2))), axis=0)


def drone_guidance_v2(pursuer, dt, N):
    global first_iter, old_R, old_sigma
    (t_pos, t_vel) = pursuer.target.get_target()
    pursuer.drone.get_state()
    R = np.linalg.norm(t_pos[0:2] - pursuer.drone.position[0:2], 2)
    sigma = math.atan2(t_pos[1] - pursuer.drone.position[1],
                       t_pos[0] - pursuer.drone.position[0])
    pursuer.R = np.append(pursuer.R, np.array([R]))
    if first_iter:
        old_R = R
        old_sigma = sigma
        first_iter = False
    else:
        if R < 5e-2 and pursuer.ka_boom == None:
            pursuer.drone.datalog.stop()
            # salva ka_boom come true guidance
            pursuer.ka_boom = len(pursuer.R)
            print(
                f"intercettazione avvenuta, il valore di R all' intercettazione vale {R}")
            pursuer.drone.landing()

        dotSigma = (sigma - old_sigma)/dt
        Vc = - (R - old_R) / dt
        old_sigma = sigma
        old_R = R

        # calculate PNG acceleration
        N_tv = N/math.cos(sigma - math.radians(pursuer.drone.yaw))
        Ac = N * Vc * dotSigma
        omega = - math.degrees(Ac/np.linalg.norm(pursuer.drone.velocity[0:2], 2))
        # omega saturation
        if omega > 90:
            omega = 90
        elif omega < -90:
            omega = -90
        #print(f'la velocità angolare vale {omega}, ')
        crazy.guidance_matlab.write(t_pos[0], t_pos[1], R, Vc, dotSigma)
        pursuer.send_command(pursuer.v, 0.0, omega, dt=dt)

def drone_guidance(pursuer, dt, N):
    (t_pos, t_vel) = pursuer.target.get_target()
    pursuer.drone.get_state()
    cos_yaw = math.cos(math.radians(pursuer.drone.yaw))
    sin_yaw = math.sin(math.radians(pursuer.drone.yaw))
    rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

    pursuer.drone.velocity = rot_yaw.dot(pursuer.drone.velocity)
    R = np.linalg.norm(t_pos[0:2] - pursuer.drone.position[0:2], 2)
    pursuer.R = np.append(pursuer.R, np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line, time.time())
    if R < 2e-2 and pursuer.ka_boom == None:
        pursuer.drone.datalog.stop()
        # salva ka_boom come true guidance
        pursuer.ka_boom = len(pursuer.R)
        print(pursuer.ka_boom, len(pursuer.time_line))
        print(
            f"intercettazione avvenuta al tempo {pursuer.time_line[pursuer.ka_boom - 1] - pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}")
        pursuer.drone.landing()
    sigma = math.atan2(t_pos[1] - pursuer.drone.position[1],
                       t_pos[0] - pursuer.drone.position[0])
    Vc = - ((t_vel[0] - pursuer.drone.velocity[0]) * (
                t_pos[0] - pursuer.drone.position[0]) + (
                        t_vel[1] - pursuer.drone.velocity[1]) * (
                        t_pos[1] - pursuer.drone.position[1])) / R
    dotSigma = ((t_vel[1] - pursuer.drone.velocity[1]) * (
                t_pos[0] - pursuer.drone.position[0]) - (
                            t_vel[0] - pursuer.drone.velocity[0]) * (
                            t_pos[1] - pursuer.drone.position[1])) / (R * R)

    # append these values to numpy to plot at the end their behavior on time
    pursuer.dotSigma = np.append(pursuer.dotSigma, np.array([dotSigma]))
    pursuer.Vc = np.append(pursuer.Vc, np.array([Vc]))
    N_tv = N / math.cos(sigma - math.radians(pursuer.drone.yaw))
    # calculate PNG acceleration
    Ac = N_tv * Vc * dotSigma
    omega = - math.degrees(Ac / np.linalg.norm(pursuer.drone.velocity[0:2], 2))
    # omega saturation
    if omega > 90:
        omega = 90
    elif omega < -90:
        omega = -90
    # print(f'la velocità angolare vale {omega}, ')
    crazy.guidance_matlab.write(t_pos[0], t_pos[1], R, Vc, dotSigma)
    pursuer.send_command(pursuer.v, 0.0, omega, dt=dt)
    # if pursuer.list_pos[0] == -100.0:
    #     pursuer.list_pos = pursuer.drone.position.reshape((1,2))
    # else:
    pursuer.list_pos = np.concatenate(
        (pursuer.list_pos, pursuer.drone.position.reshape((1, 2))), axis=0)


class DroneGuidance:
    def __init__(self,target,droneM,guidance_velocity= 0.5,dt=0.05,N=3):
        self.ka_boom = None
        self.update_thread=threading.Thread(target=crazy.repeat_fun,
                                            args=(dt, drone_guidance_v3, self, dt, N))
        self.list_pos = np.array([-100.0,-100.0]).reshape((1,2))
        self.p = np.array([])
        self.v = guidance_velocity
        self.drone = droneM
        self.target = target
        self.R = np.array([])
        self.Vc = np.array([])
        self.dotSigma = np.array([])
        self.time_line = np.array([])

    def start(self, vx, vy):
        self.drone.start(vx, vy)
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

    def send_command(self, vx, vy, omega, dt= 0.05):
        self.drone.send_command(vx, vy, omega,dt)

    def plot_chase(self):
        self.list_pos = self.list_pos[1:-1,:]
        self.time_line-=self.time_line[0]
        # print(f' il kaboom {self.ka_boom}, mentre la lunghezza vale {len(self.time_line)}')
        if self.ka_boom != None:

            self.target.time_line-=self.target.time_line[0]
            #print(self.target.time_line)
            # self.ka_boom-=1
            # print(f' momento di collisione :{self.time_line[self.ka_boom]} e distanza di collisione{self.R[self.ka_boom]}')
            # print(self.time_line[self.ka_boom ])
            len_of_time = np.where(self.target.time_line<= self.time_line[self.ka_boom -1],self.target.time_line,0)
            i = 0
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

    def plot_chase_info(self):
        if self.ka_boom != None:

            self.time_line = self.time_line[0:self.ka_boom]
            self.R = self.R[0:self.ka_boom]
            self.dotSigma = self.dotSigma[0:self.ka_boom]
            self.Vc = self.Vc[0:self.ka_boom]
        plt.figure(1)
        plt.title('Derivative of Line of Sight')
        plt.plot(self.time_line, self.dotSigma[:], '-')
        plt.show()
        plt.figure(2)
        plt.title('Closing Velocity')
        plt.plot(self.time_line, self.Vc[:], '-')
        plt.show()
        plt.figure(3)
        plt.title('Pursuer/Target Distance')
        plt.plot(self.time_line, self.R[:], '-')
        plt.show()
        print(f'la distanza minima di intercettazione vale {np.min(self.R)}')
        dif_time = np.array([self.time_line[i]-self.time_line[i-1] for i in np.arange(1,len(self.time_line),1)])
        print(f'tempo minimo di esecuzione{np.min(dif_time)}')
        print(f'tempo medio di esecuzione {np.mean(dif_time)}')


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    target = tar_c.target(initial_position=np.array([0.5, 1.0, -0.5]),
                          initial_velocity=np.array([-0.25, 0.0, 0.0]),
                          dt=0.005)

    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.2, -1.0, 2.0, -1.5]))

    guidance = DroneGuidance(target, drone,
                             guidance_velocity=0.6,
                             N=3, dt=0.011)
    guidance.start(0.0, 0.4)
    guidance.stop()
    guidance.plot_chase()
    # guidance.plot_chase_info()
    print(f'la distanza minore raggiunta dal drone vale {np.min(guidance.R)}')
    print('Main Finished')
