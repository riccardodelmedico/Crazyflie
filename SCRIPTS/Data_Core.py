import logging
import threading
import time
import numpy as np
from own_module import crazyfun as crazy
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s
from FadingFilter import FadingFilter

variable_dict = {
    "t_get":1

}
#(est_t_pos[0], est_t_pos[1],est_pur_pos[0], est_pur_pos[1], && stessa cosa con le velocità, (4)
# r, sigma,
# r_dot,sigma_dot, (calcollate con V dai FF)
# r_dot,sigma_dot, (calcollate con V dai FF e Kalman fillter)
# matlab_time,
# est_t_acc[0], est_t_acc[1],est_p_acc[0], est_p_acc[1])
def update_data_core(data):
    if data.drone_check == 0:
        pass
    elif data.drone_check == 1:
        pass
    else:
        pass
class DataCore:
    def __init__(self,vicon_freq, target,beta ,pur_vel):
        self.vicon_freq = vicon_freq
        # mutex per accedere alle variabili globali contententi il filtro del drone ed il flag si sincronizzazione
        self.mutex = threading.Semaphore(value=1)
        self.counter = 0
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(1/vicon_freq, update_data_core,self))
        self.pursuer_vel = pur_vel
        # se lo drone_check è a zero deve girare solo le correzioni e controlla quando avviene il fronte in salita
        # se lo drone_check è 1 inizializza i filtri e poi passa allo stato due
        # se lo drone_check è 2 fa l'update delle sue variabili
        self.drone_check = 0
        self.target = target
        self.target_ff = FadingFilter(dimensions=3, order=3, beta=beta[0])
        self.pursuer_ff = FadingFilter(dimensions=3, order=3, beta=beta[1])
      #  self.logger =  definire ill logger per la scrittura asincrona
        self.guidance_variable = np.zeros((19))
#(est_t_pos[0], est_t_pos[1],est_pur_pos[0], est_pur_pos[1], && stessa cosa con le velocità, (4)
# r, sigma,
# r_dot,sigma_dot, (calcollate con V dai FF)
# r_dot,sigma_dot, (calcollate con V dai FF e Kalman fillter)
# matlab_time,
# est_t_acc[0], est_t_acc[1],est_p_acc[0], est_p_acc[1])
    def start(self):
        self.update_thread.start()

    def stop(self):
        self.update_thread.join()
    def get_data(self, list_param):
        dim = len(list_param)
        ret = np.zeros((dim))
        for i in range(dim):

            ret[i] = self.guidance_variable(variable_dict[list_param(i)])