from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from DroneGuidance import *
from own_module import script_setup as sc_s, \
    script_variables as sc_v
from DroneManager import DroneManager
from Target import Target
from Seeker import Seeker
from DataCore import DataCore


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
print('Wand initial position initialized to zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    in_p = get_wand_position()
print("Wand position initialized with 'real' values")
#
# in_p = np.array([-1.5, 1.5, 0])
# in_v = np.array([0.3, -0.3, 0])
# in_a = 0.0
in_p = np.array([-1.5, 1.5, 0])
in_v = np.array([0.0, 0.0, 0])
in_a = 0.0
guidance_beta = np.array([0.35, 0.3])

yr_ff_beta = 0.85
delta = 0.02
beta_core = np.array([0.5, 0.5])
pos = np.array([-1, -1.5])
vc = np.array([0, 0.7])
list_data = ["r", "sigma", "t_acc_x", "t_acc_y"]

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    target = Target(initial_pos=in_p, initial_vel=in_v, use_wand_target=True)
    core = DataCore(200, target, beta_core, vc, scf)
    core.start()
    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.2, -1.5, 2.0, -1.0]))
    seeker = Seeker(list_data)
    guidance = DroneGuidance(guidance_beta, yr_ff_beta, seeker,
                             drone, guidance_velocity=np.linalg.norm(vc, 2),
                             dt=delta, N=5)
    guidance.start(pos, vc)
    guidance.stop()
    core.logger.save_all()
    guidance.logger.save_all()
    print('Main Finished')

# test venuti egregiamente :
# crazyfun__20220112_151923
