import logging
import math
import threading
import time
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


class DroneManager:
    def __init__(self, cf, AccZPn = 0.1):
        self.crazyflie = cf
        self.crazyflie.param.set_value('AccZPn', 0.1)
        self.crazyflie.reset_estimator()
        # self.position  = ..;
        # self.velocity = ..;

    def set_value(self, string, value):
        self.crazyflie.param.set_value(string, value)

    def start(self):
        pass
        # datalog
        # vicon_pose_sending
        # take_off(vx, vy): start from outside virtual box and reach vx and vy

    def take_off(self, vx, vy):
        pass
    #     self.crazyflie.

    def check_virtual_box(self):
        pass
    #   guardo se la posizione attuale del drone è all'itnerno della virtual box
    #   ritorna un booleano
    #   false => atterro, true => mando il comando
    #   self.get_kinematic_variables
    #   guarda self.position e self.velocity
    #
    #
    #

    def send_command(self, vx, vy, yawrate):
        pass
    # prendo gli input dalla classe guida e mando l'hover setpoint soltanto se check_virtual_box
    # torna true, altrimenti faccio il landing
    #     flag = self.check_virtual_box()
    #     if(flag):
    #         self.crazyflie.send_hover_setpoint(vx, vy, 0.0, yawrate)
    #     else:
    #         self.landing()
    #

    def landing(self):
       pass
    #   atterraggio nella posizione attuale
    #
    # def get_kinematic_variables():
    # entra in sezione critica e legge le informazioni che vengono aggiornate
    # nella callback
    # aggiorno self.position e self.velocity

    def stop(self):
        pass












