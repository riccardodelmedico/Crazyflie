import numpy as np
from own_module import script_variables as sc_v, script_setup as sc_s
class Target:
    def __init__(self, initial_pos,
                 initial_vel=np.array([0.0, 0.0, 0.0]),
                 initial_acc_module=0, use_wand_target=False):
        self.pos = initial_pos
        self.vel = initial_vel
        self.acc = initial_acc_module
        self.wand = use_wand_target
        self.omega_vers_hat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])


    def init_wand(self, init_pos):
        if self.wand:
            self.pos = init_pos
    # funzione che ritona la posizione della wand o aggiorna lo stato interno del target virtuale
    def update(self,dt):
        if self.wand:
            sc_v.wand_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
            self.pos = np.array([float(sc_v.wand_pos[0] / 1000),
                                   float(sc_v.wand_pos[1] / 1000),
                                   float(sc_v.wand_pos[2] / 1000)])
            return (self.pos, None)
        else:
            self.pos += self.vel * dt + self.omega_vers_hat.dot(
                self.vel / np.linalg.norm(self.vel, 2)) * self.acc * dt * dt / 2
            self.vel += self.omega_vers_hat.dot(self.vel / np.linalg.norm(self.vel, 2)) * self.acc * dt
            return (self.pos,self.vel)



