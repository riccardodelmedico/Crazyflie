import logging
import math
import threading
import time
import numpy as np
from FadingFilter import FadingFilter
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream




def compute_sigma(pursuer_pos, target_pos):
    r_v = pursuer_pos[0:2] - target_pos[0:2]
    sigma = math.atan2(r_v[1], r_v[0])
    if sigma < 0:
        sigma += 2 * math.pi
    return sigma


def compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = (target_vel[1] - pursuer_vel[1]) * (target_pos[0] - pursuer_pos[0]) - \
          (target_vel[0] - pursuer_vel[0]) * (target_pos[1] - pursuer_pos[1])
    den = r * r
    return num / den


def compute_r(pursuer_pos, target_pos):
    r_v = pursuer_pos[0:2] - target_pos[0:2]
    r = np.linalg.norm(r_v, 2)
    return r


def compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = ((target_vel[0] - pursuer_vel[0]) * (target_pos[0] - pursuer_pos[0]) +
           (target_vel[1] - pursuer_vel[1]) * (target_pos[1] - pursuer_pos[1]))
    den = r
    return num / den


def get_wand_position():
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
    return in_p


def rot_z(yaw, radians=False):
    if not radians:
        rad_yaw = math.radians(yaw)
    else:
        rad_yaw = yaw
    cos_yaw = math.cos(math.radians(rad_yaw))
    sin_yaw = math.sin(math.radians(rad_yaw))
    rot_yaw = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    return rot_yaw


def guidance_png(guidance, N, dt,r_incterception):

    #get the state of target
    (tar_pos,_,_,tar_time) = guidance.target.get_state()
    
    #evaluate the estimation of target quantities
    (est_t_pos,est_t_vel,ext_t_acc)=guidance.target_ff.update(tar_pos,tar_time)


    #get the state of drone in world frame
    guidance.drone.get_state()
    rot_yaw = rot_z(guidance.drone.yaw)
    guidance.drone.velocity = rot_yaw.dot(guidance.drone.velocity)
    
    #closed form guindance quantities evaluate
    r = compute_r(guidance.drone.position,est_t_pos)
    sigma = compute_sigma(guidance.drone.position,est_t_pos)
    r_dot = compute_r_dot(guidance.drone.position,guidance.drone.velocity,est_t_pos,est_t_vel,r)
    sigma_dot = compute_sigma_dot(guidance.drone.position,guidance.drone.velocity,est_t_pos,est_t_vel,r)
    
    if r < r_incterception and guidance.interception == None:
        guidance.drone.datalog.stop()
        guidance.interception = r
        print(
            f"il valore di R all' intercettazione vale {r}")
        guidance.drone.landing()
    # get guidance quatities estimation 
    (est_r,est_dot_r)=guidance.r_ff.update(r,tar_time)
    (est_sigma,est_dot_sigma) = guidance.sigma_ff.update(sigma,tar_time)
    #write in Log [closed_form quantities, estimated quantities and the time of data acquisition
    crazy.guidance_matlab.write(r,sigma,r_dot,sigma_dot,est_r,est_sigma,est_dot_r,est_dot_sigma,tar_time)
   

    N_tv = N / math.cos(sigma - math.radians(guidance.drone.yaw))
    # calculate PNG acceleration with closed form quantities
    Ac = - N_tv * (r_dot * sigma_dot)  # + AcAPNG/2)
    omega = - math.degrees(Ac / np.linalg.norm(guidance.drone.velocity[0:2], 2))
    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120

    guidance.send_command(guidance.v, 0.0, omega, dt=dt)



class DroneGuidance:
    def __init__(self, guidance_ff_beta, target_ff_beta, target, drone_manager, guidance_velocity=0.5,
                 dt=0.05, N=3):
        self.interception = None
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(dt,guidance_png, self, dt, N, 0.05))
        self.v = guidance_velocity
        self.drone = drone_manager
        self.target_ff = FadingFilter(dimensions=3, order=3, beta=target_ff_beta)
        self.target = target
        self.r_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)
        self.sigma_ff = FadingFilter(order=2, dimensions=1, beta=guidance_ff_beta)

    def start(self, vx, vy):
        self.drone.start(vx, vy)
        # take off deve eseguire finche non rispetta le condizioni di lancio
        # della guida, quindi V circa self.v ed il drone si trova dentro la box
        print('Start Guidance')

        in_p = get_wand_position()
        #
        in_v = np.array([0.0, 0.0, 0.0])
        in_a = np.zeros(3)
        # init wand position
        self.target.init_wand(in_p)
        # init target fading filter with initial state
        in_time = time.time()
        self.target_ff.init(np.array([in_p, in_v, in_a]), in_time)

        # inizialization of Homing Fading Filter:
        self.drone.get_state()
        rot_yaw = rot_z(self.drone.yaw)
        self.drone.velocity = rot_yaw.dot(self.drone.velocity)
        print(
            f'Posizioni e velocita iniziali in terna World {self.drone.position},{self.drone.velocity}')

        in_sigma = compute_sigma(self.drone.position, in_p)
        in_r = compute_r(self.drone.position, in_p)
        in_sigma_dot = compute_sigma_dot(self.drone.position, self.drone.velocity, in_p, in_v, in_r)
        in_r_dot = compute_r_dot(self.drone.position, self.drone.velocity, in_p, in_v, in_r)
        self.r_ff.init(np.array([[in_sigma], [in_sigma_dot]]), in_time)
        self.sigma_ff.init(np.array([[in_r], [in_r_dot]]), in_time)
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



