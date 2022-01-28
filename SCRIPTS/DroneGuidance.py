import logging
import math
import threading
import time
import numpy as np
from DroneManager import DroneManager
from FadingFilter import Fading_Filter
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream
import target_class as tar_c
import matplotlib.pyplot as plt
from Model_Identify import Model_Compensation
from  Fading_Filter_Homing import FadingFilterHoming

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
    R = np.linalg.norm(pursuer.target.real_pos[0:2]- pursuer.drone.position[0:2], 2)
    #realR =  np.linalg.norm(pursuer.target.real_pos[0:2] - pursuer.drone.position[0:2], 2)
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
def sim_APNG_guidance_filtered(pursuer,dt,N):
    # get the target position and velocity to calculate R Vc and \dot{Sigma}
    (t_pos,t_vel,t_acc) = pursuer.target.get_estimation()
    pursuer.drone.get_state()
    cos_yaw = math.cos(math.radians(pursuer.drone.yaw))
    sin_yaw = math.sin(math.radians(pursuer.drone.yaw))
    rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    pursuer.drone.velocity = rot_yaw.dot(pursuer.drone.velocity)

    R = np.linalg.norm(t_pos[0:2]-pursuer.drone.position[0:2],2)
    sigma = math.atan2(t_pos[1] - pursuer.drone.position[1],t_pos[0] - pursuer.drone.position[0])
    #pursuer.sigma = np.append(pursuer.sigma, np.array([sigma]))
    pursuer.R = np.append(pursuer.R, np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line, time.time())
    if R < 5e-2 and pursuer.ka_boom == None:
        pursuer.drone.datalog.stop()
        pursuer.ka_boom = len(pursuer.R)
        crazy.run=False
        print(f"interncettazione avvenuta al tempo { pursuer.time_line[pursuer.ka_boom - 1]- pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}" )
        pursuer.drone.landing()

    Vc = - ((t_vel[0] - pursuer.drone.velocity[0]) * (
            t_pos[0] - pursuer.drone.position[0]) + (
                    t_vel[1] - pursuer.drone.velocity[1]) * (
                    t_pos[1] - pursuer.drone.position[1])) / R
    dotSigma = ((t_vel[1] - pursuer.drone.velocity[1]) * (
            t_pos[0] - pursuer.drone.position[0]) - (
                        t_vel[0] - pursuer.drone.velocity[0]) * (
                        t_pos[1] - pursuer.drone.position[1])) / (R * R)

    # append these values to numpy to plot at the end their behavior on time
    pursuer.dotSigma = np.append(pursuer.dotSigma,np.array([dotSigma]))
    pursuer.Vc = np.append(pursuer.Vc,np.array([Vc]))




    R_v = np.append(t_pos[0:2]-pursuer.drone.position[0:2],0)/R

    Ort_R = pursuer.target.target.omega_vers_hat.dot(R_v)

    AcAPNG = np.transpose(Ort_R).dot(t_acc)/math.cos(sigma)
    AcAPNG_vett= AcAPNG*Ort_R
    #calculate PNG acceleration
    #print( AcAPNG,N * Vc * dotSigma)
    # N_tv = N / math.cos(sigma - math.atan2(pursuer.p[1],pursuer.p[0]))
    # Ac = N *( Vc * dotSigma+ AcAPNG/2)

    N_tv = N / math.cos(sigma - math.radians(pursuer.drone.yaw))
    # calculate PNG acceleration
    Ac = N_tv * ( Vc * dotSigma+ AcAPNG/2)
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
   # Ac+= AcAPNG
   #  if len(pursuer.time_line) <2:
   #      deltat=dt
   #  else:
   #      deltat = pursuer.time_line[-1] - pursuer.time_line[-2]
   #integrate velociticy and acceleration with forward euler
    # if np.linalg.norm(pursuer.v,2) != 0:
    #     pursuer.p += pursuer.v * deltat + target.target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))* Ac * deltat * deltat /2 #+ N*AcAPNG_vett*math.pow(deltat,2)/2
    #     pursuer.v += target.target.omega_vers_hat.dot(pursuer.v/np.linalg.norm(pursuer.v,2))* Ac* deltat
    # pursuer.list_pos = np.concatenate((pursuer.list_pos,pursuer.p.reshape((1,3))),axis=0)


def sim_APNG_guidance_filtered_comp(pursuer, dt, N):
    # get the target position and velocity to calculate R Vc and \dot{Sigma}
    (t_pos, t_vel, t_acc) = pursuer.target.get_estimation()
    pursuer.drone.get_state()

    (homing_guidance, homing_guidance_dot, los_rate) = pursuer.homing_ff.get_estimation(
        pursuer.drone.position, math.radians(pursuer.drone.yaw),
        pursuer.drone.yawrate)
    pursuer.homing_ff.list_homing_dot[-1, 0] += pursuer.drone.yawrate
    cos_yaw = math.cos(math.radians(pursuer.drone.yaw))
    sin_yaw = math.sin(math.radians(pursuer.drone.yaw))
    rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    pursuer.drone.velocity = rot_yaw.dot(pursuer.drone.velocity)

    R = np.linalg.norm(t_pos[0:2]-pursuer.drone.position[0:2], 2)
    sigma = math.atan2(t_pos[1] - pursuer.drone.position[1], t_pos[0] - pursuer.drone.position[0])
    #pursuer.sigma = np.append(pursuer.sigma, np.array([sigma]))
    pursuer.R = np.append(pursuer.R, np.array([R]))
    pursuer.time_line = np.append(pursuer.time_line, time.time())
    if R < 5e-2 and pursuer.ka_boom == None:
        pursuer.drone.datalog.stop()
        pursuer.ka_boom = len(pursuer.R)
        print(f"interncettazione avvenuta al tempo { pursuer.time_line[pursuer.ka_boom - 1]- pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}" )
        pursuer.drone.landing()

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
    R_v = np.append(t_pos[0:2]-pursuer.drone.position[0:2], 0) / R

    # Ort_R = pursuer.target.target.omega_vers_hat.dot(R_v)
    #
    # AcAPNG = np.transpose(Ort_R).dot(t_acc)/math.cos(sigma)
    # AcAPNG_vett= AcAPNG*Ort_R

    N_tv = N / math.cos(sigma - math.radians(pursuer.drone.yaw))
    # calculate PNG acceleration
    Ac = N_tv * (Vc * dotSigma)#+ AcAPNG/2)
    omega = - math.degrees(Ac / np.linalg.norm(pursuer.drone.velocity[0:2], 2))
    omega = pursuer.compensator.compensation(omega)
    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120
    # crazy.guidance_matlab.write(t_pos[0], t_pos[1], R, Vc, dotSigma)
    crazy.guidance_matlab.write(t_pos[0], t_pos[1], R, Vc, dotSigma, sigma,
                                homing_guidance[0] + math.radians(pursuer.drone.yaw), homing_guidance[1],
                                homing_guidance_dot[0] + pursuer.drone.yawrate,
                                homing_guidance_dot[1])

    pursuer.send_command(pursuer.v, 0.0, omega, dt=dt)
    pursuer.list_pos = np.concatenate(
        (pursuer.list_pos, pursuer.drone.position.reshape((1, 2))), axis=0)


class DroneGuidance:
    def __init__(self, HFF, compensator, target, droneM, guidance_velocity=0.5, dt=0.05, N=3, fading=True):
        self.ka_boom = None
        if not fading:
            self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                                  args=(dt, drone_guidance_v3, self, dt, N))
        else:
            self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                                  args=(dt, sim_APNG_guidance_filtered_comp, self, dt, N))
        self.list_pos = np.array([-100.0, -100.0]).reshape((1, 2))
        self.p = np.array([])
        self.v = guidance_velocity
        self.drone = droneM
        self.target = target
        self.R = np.array([])
        self.Vc = np.array([])
        self.dotSigma = np.array([])
        self.time_line = np.array([])
        self.compensator = compensator
        self.homing_ff = HFF

    def start(self, vx, vy):
        self.drone.start(vx, vy)
        #take off deve eseguire finche non rispetta le condizioni di lancio
        #della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')

        try:
            sc_s.vicon.GetFrame()
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", str(exc))

            # Get current drone position and orientation in Vicon
        sc_v.wand_pos = sc_s.vicon. \
            GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

        # Converts in meters

        in_p = np.array([float(sc_v.wand_pos[0] / 1000),
                         float(sc_v.wand_pos[1] / 1000),
                         float(sc_v.wand_pos[2] / 1000)])
        print("terminata l'inizializzaizone reale")
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)

        self.target.target.initial_wand(in_p)
        self.target.initial(np.array([in_p, in_v, in_a]))

        #inizialization of Homing Fading Filter
        self.drone.get_state()
        cos_yaw = math.cos(math.radians(self.drone.yaw))
        sin_yaw = math.sin(math.radians(self.drone.yaw))
        rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        self.drone.velocity = rot_yaw.dot(self.drone.velocity)
        print(
            f'Posizioni e velocita iniziali in terna World {self.drone.position},{self.drone.velocity}')
        self.homing_ff.initialize(self.drone.position, self.drone.velocity)


        crazy.run = True
        self.target.start()
        self.update_thread.start()

    def stop(self):
       # tar_c.run = False
       self.update_thread.join()
       self.target.stop()

    def send_command(self, vx, vy, omega, dt=0.05):
        self.drone.send_command(vx, vy, omega, dt)

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
            self.target.list_vel = self.target.list_vel[0:i, :]
            self.target.list_acc = self.target.list_acc[0:i, :]
            self.target.real_pos = self.target.real_pos[0:i, :]
            self.target.time_line = self.target.time_line[0:i]
        plt.figure(1)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.plot(self.list_pos[0, 0], self.list_pos[0, 1], 'ro')
        plt.plot(self.list_pos[:, 0], self.list_pos[:, 1], 'r-')
        plt.plot(self.list_pos[-1, 0], self.list_pos[-1, 1], 'r->')
        plt.plot(self.target.list_pos[0, 0], self.target.list_pos[0, 1], 'bo')
        plt.plot(self.target.list_pos[:, 0], self.target.list_pos[:, 1], 'b--')
        plt.plot(self.target.list_pos[-1, 0], self.target.list_pos[-1, 1],'b->')
        plt.plot(self.target.real_pos[0, 0], self.target.real_pos[0, 1], 'go')
        plt.plot(self.target.real_pos[:, 0], self.target.real_pos[:, 1], 'g-')
        plt.plot(self.target.real_pos[-1, 0], self.target.real_pos[-1, 1],'g->')
        plt.show()

    def plot_chase_info(self):
        if self.ka_boom != None:

            self.time_line = self.time_line[0:self.ka_boom]
            self.R = self.R[0:self.ka_boom]
            self.dotSigma = self.dotSigma[0:self.ka_boom]
            self.Vc = self.Vc[0:self.ka_boom]
        plt.figure(1)
        plt.title('Derivative of Line of Sight')
        plt.plot(self.time_line, self.dotSigma[:], 'b-')
        plt.plot(self.time_line, self.homing_ff.list_homing_dot[1:,0], 'r-')
        # plt.plot(self.time_line, self.homing_ff.los_rate[1:] + self.homing_ff.gamma_dot[1:], 'g-')
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


print('inizializzo il vettore delle posizioni iniziali a zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

        # Get current drone position and orientation in Vicon
    sc_v.wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters


    in_p = np.array([float(sc_v.wand_pos[0] / 1000),
                     float(sc_v.wand_pos[1] / 1000),
                     float(sc_v.wand_pos[2] / 1000)])
print("terminata l'inizializzaizone reale")
in_v = np.array([0.0, 0.0, 0.0])
in_a = np.zeros(3)
delta = 2e-2
vc = 0.6
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    # comp = Model_Compensation(np.array([0.0299875609518963, 0.0300702032877777, 3.00672045316825e-06, 0]),
    #                           np.array([1, -0.516503065398502, -0.970604433271622, 0.545898677743298]))
    comp = Model_Compensation()
    target = tar_c.target(initial_position=in_p,
                          dt=0.1, use_wand_target=True)
    ff = Fading_Filter(target, Nstd=0, Dimensions=2, Order=3, Beta=0.5,
                       dt=delta)
    hff = FadingFilterHoming(target, chase_vel=vc, std=3e-3, beta=0.1, dt=delta)
    ff.initial(np.array([in_p, in_v, in_a]))

    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.5, -1.8, 2.0, -0.8]))

    guidance = DroneGuidance(hff, comp, ff, drone,
                             guidance_velocity=vc,
                             N=5, dt=delta)
    guidance.start(0.0, 1.0)
    guidance.stop()
    guidance.plot_chase()

    print(ff.real_pos.shape, ff.list_pos.shape, ff.time_line.shape)
    ff.plot_Filter()
    guidance.plot_chase_info()
    print(f'la distanza minore raggiunta dal drone vale {np.min(guidance.R)}')
    print('Main Finished')
#test venuti egregiamente :
#crazyfun__20220112_151923