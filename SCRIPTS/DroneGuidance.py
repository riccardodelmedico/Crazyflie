import logging
import math
import threading
import time
import numpy as np
import DroneManager
import FadingFilter
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream
import Target as tar_c
import matplotlib.pyplot as plt
from Model_Identify import Model_Compensation


first_iter = True


def compute_sigma():
    pass


def compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = (target_vel[1] - pursuer_vel[1]) * (target_pos[0] - pursuer_pos[0]) - \
          (target_vel[0] - pursuer_vel[0]) * (target_pos[1] - pursuer_pos[1])
    den = r * r
    return num / den


def compute_r():
    pass


def compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = ((target_vel[0] - pursuer_vel[0]) * (target_pos[0] - pursuer_pos[0]) +
           (target_vel[1] - pursuer_vel[1]) * (target_pos[1] - pursuer_pos[1]))
    den = r
    return num / den


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
    if R < 5e-2 and pursuer.interception == None:
        pursuer.drone.datalog.stop()
        pursuer.interception = len(pursuer.R)
        print(f"interncettazione avvenuta al tempo { pursuer.time_line[pursuer.interception - 1]- pursuer.time_line[0]}, il valore di R all' intercettazione vale {R}" )
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
    def __init__(self, guidance_ff, compensator, target_ff, target, drone_manager, guidance_velocity=0.5, dt=0.05, N=3):
        self.interception = None
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(dt, sim_APNG_guidance_filtered_comp, self, dt, N))
        self.v = guidance_velocity
        self.drone = drone_manager
        self.target_ff = target_ff
        self.target = target
        self.compensator = compensator
        self.guidance_ff = guidance_ff

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
        print("terminata l'inizializzazione reale")
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)
        # init wand position
        self.target.init_wand(in_p)
        # init target fading filter with initial state
        self.target_ff.init(np.array([in_p, in_v, in_a]))

        #inizialization of Homing Fading Filter:
        # TODO: spostare calcolo velocità in world in DroneManager
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
        # dt: send_hover_setpoint frequency
        self.drone.send_command(vx, vy, omega, dt)


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
    ff = FadingFilter(target, Nstd=0, Dimensions=2, Order=3, Beta=0.5,
                       dt=delta)
    hff = FadingFilter(target, chase_vel=vc, std=3e-3, beta=0.1, dt=delta)
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