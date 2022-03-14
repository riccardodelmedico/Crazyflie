import threading
import time
from FadingFilter import FadingFilter
from GuidanceUtility import *
from own_module import crazyfun as crazy


def guidance_png_command(guidance, n, r_interception):
    """
    implementation of guidance

    :param guidance: instance of DroneGuidance
    :type guidance: DroneGuidance
    :param n: guidance constant
    :type n: float in [3,5]
    :param r_interception: distance below which there is interception
    :type r_interception: float
    """

    measures = guidance.seeker.get_measures()

    r = measures[0]
    sigma = measures[1]
    yr = r * sigma
    t_acc_x = measures[2]
    t_acc_y = measures[3]
    new_time = measures[-1]

    if r < r_interception and guidance.interception is None:
        guidance.drone.datalog.stop()
        crazy.run = False
        guidance.interception = r
        print(f"R value at interception: {r}")
        guidance.drone.landing()

    (_, est_dot_r) = guidance.r_ff.update(np.array([r]), new_time)
    (_, est_dot_sigma) = guidance.sigma_ff.update(np.array([sigma]), new_time)
    (_, est_dot_yr, est_ddot_yr) = guidance.yr_ff.update(np.array([yr]),
                                                         new_time)

    acc_apng = t_acc_x * math.cos(sigma + math.pi / 2) + \
        t_acc_y * math.sin(sigma + math.pi / 2)
    # acc_apng = 0
    acc = n * est_dot_sigma * -est_dot_r + acc_apng/2
    guidance.drone.get_state()
    omega = - math.degrees(
        acc / np.linalg.norm(guidance.drone.velocity[0:2], 2))

    # omega saturation
    if omega > 120:
        omega = 120
    elif omega < -120:
        omega = -120
    guidance.guidance_data[0] = est_dot_r
    guidance.guidance_data[1] = est_dot_sigma
    guidance.guidance_data[2] = new_time
    guidance.logger.append(guidance.guidance_data)
    guidance.send_command(guidance.v, 0.0, omega)


class DroneGuidance:
    """
    DroneGuidance class contains guidance thread and fading filter used to
    implement guidance law

    Constructor:
    :param guidance_ff_beta: beta parameters for guidance FF
                             guidance_ff_beta[0] => r_ff
                             guidance_ff_beta[1] => sigma_ff
    :type guidance_ff_beta: float np.array[2]
    :param yr_ff_beta: beta parameters for yr FF
    :type yr_ff_beta:  float
    :param seeker: instance of Seeker class
    :type seeker: Seeker
    :param drone_manager: instance of DroneManager
    :type drone_manager: DroneManager
    :param guidance_data_length: number of guidance data that must be save
    :type guidance_data_length: int
    :param guidance_velocity: module of chasing velocity
    :type guidance_velocity: float
    :param dt: period of guidance thread
    :type dt: float
    :param N: guidance constant
    :type N: float in [3,5]
    """

    def __init__(self, guidance_ff_beta, yr_ff_beta, seeker, drone_manager,
                 guidance_data_length, guidance_velocity=0.5, dt=0.05, N=3):
        self.interception = None
        self.update_thread = threading.Thread(target=crazy.repeat_fun,
                                              args=(1, dt, guidance_png_command,
                                                    self, N, 0.05))
        self.v = guidance_velocity
        self.drone = drone_manager
        self.seeker = seeker
        self.r_ff = FadingFilter(order=2, dimensions=1,
                                 beta=guidance_ff_beta[0])
        self.sigma_ff = FadingFilter(order=2, dimensions=1,
                                     beta=guidance_ff_beta[1])
        self.yr_ff = FadingFilter(order=3, dimensions=1, beta=yr_ff_beta)
        self.logger = crazy.AsyncMatlabPrint(guidance_data_length, flag=5)
        self.guidance_data = np.zeros((guidance_data_length, 1))

    def start(self, pos, vel):
        """
        starts the guidance task from initial condition

        :param pos: initial world position from which the drone will enter
                    virtual box
        :type pos: float np.array[2]
        :param vel: initial world velocity to enter the box
        :type vel: float np.array[2]
        """

        self.drone.start(pos, vel)
        print('Start Guidance')
        time.sleep(0.02)
        # initialize filter inside DroneGuidance:
        init_measures = self.seeker.get_measures()
        print(f'init measures:{init_measures}')
        r = init_measures[0]
        sigma = init_measures[1]
        yr = r * sigma
        new_time = init_measures[-1]

        self.r_ff.init(np.array([[r], [0.0]]), new_time)
        self.sigma_ff.init(np.array([[sigma], [0.0]]), new_time)
        self.yr_ff.init(np.array([[yr], [0.0], [0.0]]), new_time)
        crazy.run = True
        time.sleep(0.02)
        self.update_thread.start()

    def stop(self):
        self.update_thread.join()

    def send_command(self, vx, vy, omega):
        self.drone.send_command(vx, vy, omega)
