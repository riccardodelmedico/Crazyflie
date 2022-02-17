import math
import numpy as np
# This file is an utility module that compute all the guidance quantities which
# will be feed to fading filter.


def compute_sigma(pursuer_pos, target_pos):
    r_v = target_pos[0:2] - pursuer_pos[0:2]
    sigma = math.atan2(r_v[1], r_v[0])
    if sigma < 0:
        sigma += 2 * math.pi
    return sigma


def compute_sigma_homing(pursuer_pos, pursuer_yaw, target_pos):
    r_v = target_pos[0:2] - pursuer_pos[0:2]
    pursuer_yaw = math.radians(pursuer_yaw)
    sigma_homing = math.atan2(r_v[1], r_v[0]) - pursuer_yaw
    # if sigma_homing < 0:
    #     sigma_homing += 2 * math.pi
    return sigma_homing


def compute_sigma_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = (target_vel[1] - pursuer_vel[1]) * (target_pos[0] - pursuer_pos[0]) -\
          (target_vel[0] - pursuer_vel[0]) * (target_pos[1] - pursuer_pos[1])
    den = r * r
    return num / den


def compute_r(pursuer_pos, target_pos):
    r_v = target_pos[0:2] - pursuer_pos[0:2]
    r = np.linalg.norm(r_v, 2)
    return r


def compute_r_dot(pursuer_pos, pursuer_vel, target_pos, target_vel, r):
    num = ((target_vel[0] - pursuer_vel[0]) * (target_pos[0] - pursuer_pos[0]) +
           (target_vel[1] - pursuer_vel[1]) * (target_pos[1] - pursuer_pos[1]))
    den = r
    return num / den
