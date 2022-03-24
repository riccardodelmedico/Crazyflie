import logging
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from DroneGuidance import *
from own_module import script_setup as sc_s, \
    script_variables as sc_v
from DroneManager import DroneManager
from Target import Target
from Seeker import Seeker
from DataCore import DataCore
from vicon_dssdk import ViconDataStream


def get_wand_position():
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

    # Get current drone position and orientation in Vicon
    wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters
    wand_pos = np.array([float(wand_pos[0] / 1000),
                         float(wand_pos[1] / 1000),
                         float(wand_pos[2] / 1000)])
    return wand_pos


# ------------------MAIN-------------------------- #
print('Waiting for Wand to be turned on')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    in_p = get_wand_position()
print("Wand position initialized with 'real' values")

wand = False
in_p = np.array([0.8, 2.0, 0])
in_v = np.array([-0.33, -0.33, 0])
in_a = 0#1/6

guidance_beta = np.array([0.35, 0.3])
core_beta = np.array([0.45, 0.45])
yr_ff_beta = 0.65

delta = 0.02
initial_drone_pos = np.array([0.8, -1.5])
chase_velocity = np.array([0, 0.60])
data_list = ["r", "sigma", "t_acc_x", "t_acc_y"]

virtual_box = np.array([1.2, -1.5, 2.0, -1.0])
guidance_gain = 5

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    target = Target(initial_pos=in_p, initial_vel=in_v, initial_acc_module=in_a,
                    use_wand_target=wand)
    core = DataCore(200, target, core_beta, chase_velocity, scf)
    core.start()
    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=virtual_box)
    seeker = Seeker(data_list)
    guidance = DroneGuidance(guidance_beta, yr_ff_beta, seeker,
                             drone, guidance_velocity=np.linalg.norm(chase_velocity, 2),
                             dt=delta, N=guidance_gain, guidance_data_length=3)
    guidance.start(initial_drone_pos, chase_velocity)
    guidance.stop()
    core.logger.save_all()
    guidance.logger.save_all()
    print('Main Finished')
