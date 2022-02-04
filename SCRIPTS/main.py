from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from Model_Identify import Model_Compensation
from DroneGuidance import *
from own_module import script_setup as sc_s, \
    script_variables as sc_v
from DroneManager import DroneManager
from Target import Target

#
print('Wand initial position initialized to zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    in_p = get_wand_position()
print("Wand position initialized with 'real' values")

in_p = np.array([-1.0, 1.5, 0])
in_v = np.array([0.0, -0.5, 0])
in_a = 0.2
guidance_beta = 0.1
target_beta = 0.5
delta = 0.02
vc = 0.7

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    comp = Model_Compensation()
    target = Target(initial_pos=in_p, initial_vel=in_v, initial_acc_module=in_a,
                    dt=0.02, use_wand_target=True)

    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.0, -1.5, 2.0, -1.0]))

    guidance = DroneGuidance(guidance_beta, target_beta, target,
                             drone, guidance_velocity=vc, dt=delta, N=5)
    guidance.start(0.0, vc)
    guidance.stop()
    print('Main Finished')

# test venuti egregiamente :
# crazyfun__20220112_151923