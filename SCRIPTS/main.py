import logging

import numpy as np
import DroneManager
from FadingFilter import FadingFilter
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from Model_Identify import Model_Compensation
from DroneGuidance import *
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from vicon_dssdk import ViconDataStream
from DroneManager import DroneManager
from Target import Target



print('inizializzo il vettore delle posizioni iniziali a zero')
in_p = np.zeros(3)
while len(np.argwhere(in_p == 0.0)) == 3:
    in_p = get_wand_position()
print("terminata l'inizializzaizone reale")

guidance_beta = 0.1
target_beta = 0.9
delta = 2e-2
vc = 0.6
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    print('Main Start')
    # comp = Model_Compensation(np.array([0.0299875609518963, 0.0300702032877777, 3.00672045316825e-06, 0]),
    #                           np.array([1, -0.516503065398502, -0.970604433271622, 0.545898677743298]))
    comp = Model_Compensation()
    target = Target(initial_pos=in_p,
                          dt=0.1, use_wand_target=True)


    drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0,
                         box=np.array([1.5, -1.8, 2.0, -0.8]))

    guidance = DroneGuidance(target_beta,guidance_beta, drone,
                             guidance_velocity=vc,
                             N=5, dt=delta)
    guidance.start(0.0, 1.0)
    guidance.stop()
    print('Main Finished')
# test venuti egregiamente :
# crazyfun__20220112_151923