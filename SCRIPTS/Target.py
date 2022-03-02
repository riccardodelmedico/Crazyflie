import numpy as np
from own_module import script_variables as sc_v, script_setup as sc_s


class Target:
    """
    Target class manages the update of target position.
    The target can be virtual or can be the Wand. The Constructor parameters are
    meaningful only for virtual target since the Wand is managed online

    Constructor:
    :param initial_pos: initial target position
    :type initial_pos: float np.array[3]
    :param initial_vel: initial target velocity
    :type initial_vel: float np.array[3]
    :param initial_acc_module: acceleration module (orthogonal to velocity)
    :type initial_acc_module: float
    :param use_wand_target: flag for choosing type of target
    :type use_wand_target: bool

    """

    def __init__(self, initial_pos,
                 initial_vel=np.array([0.0, 0.0, 0.0]),
                 initial_acc_module=0, use_wand_target=False):
        self.pos = initial_pos
        self.vel = initial_vel
        self.acc = initial_acc_module
        self.wand = use_wand_target
        self.omega_vers_hat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])

    # def init_wand(self, init_pos):
    #     if self.wand:
    #         self.pos = init_pos

    def update(self, dt):
        """
        update target position: if target is virtual, its position is updated
        with forward Euler method and returns both position and velocity.
        If target is the Wand, its position is updated simply getting the actual
        position from last frame

        :param dt: used only for virtual target; time interval for updating
        :type dt: float
        """
        if self.wand:
            wand_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]
            self.pos = np.array([float(wand_pos[0] / 1000),
                                 float(wand_pos[1] / 1000),
                                 float(wand_pos[2] / 1000)])
            return self.pos, np.zeros(3)
        else:
            if np.linalg.norm(self.vel, 2) != 0:
                self.pos += self.vel * dt + self.omega_vers_hat.dot(
                    self.vel / np.linalg.norm(self.vel, 2)) * self.acc \
                            * dt * dt / 2
                self.vel += self.omega_vers_hat.dot(
                    self.vel / np.linalg.norm(self.vel, 2)) * self.acc * dt
            return self.pos, self.vel
