import math
import threading
import time
import numpy as np
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


# box parameter defines a virtual box where crazyflie has to stay in order to
# implement the guidance law for safety reasons.
# box[0]: positive x boundary [m]
# box[1]: negative x boundary [m]
# box[2]: positive y boundary [m]
# box[3]: negative y boundary [m]
# limits are measured from world reference frame origin


class DroneManager:
    def __init__(self, scf, pnAcc_xy=1.0, pnAcc_z=0.5, pnPos=0.0, pnVel=0.0,
                 box=np.array([1.0, -1.0, 1.0, -1.0])):
        self.scf = scf
        self.scf.cf.param.set_value('stabilizer.estimator', 2)
        self.scf.cf.param.set_value('kalman.pNAcc_xy', pnAcc_xy)
        self.scf.cf.param.set_value('kalman.pNAcc_z', pnAcc_z)
        self.scf.cf.param.set_value('kalman.pNPos', pnPos)
        self.scf.cf.param.set_value('kalman.pNVel', pnVel)
        self.position = np.array([])
        self.velocity = np.array([])
        self.yawrate = np.array([])
        self.initial_position = np.array([])
        self.initial_orientation = np.array([])
        self.flying = False
        self.yaw = 0.0
        self.box = box
        self.vicon_thread = threading.Thread(target=crazy.repeat_fun,
                                             args=(1, crazy.vicon2drone_period,
                                                   crazy.pose_sending,
                                                   self.scf))
        self.datalog = crazy.datalog(self.scf)

    def set_value(self, string, value):
        self.scf.cf.param.set_value(string, value)

    def reset_estimator(self):
        cf = self.scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

    def start(self, vx, vy):
        self.position = np.array(sc_s.vicon.
                                 GetSegmentGlobalTranslation(sc_v.drone,
                                                             sc_v.drone)[0])
        self.position /= 1000
        self.initial_position = self.position
        self.initial_orientation = sc_s.vicon. \
            GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)[0]

        print(f'Initial position [m]: x{self.position[0]}, y:{self.position[1]}'
              f' z{self.position[2]}')
        print(
            f"Initial yaw [degree]: {math.degrees(self.initial_orientation[2])}")

        self.scf.cf.param.set_value('kalman.initialX', float(self.position[0]))
        self.scf.cf.param.set_value('kalman.initialY', float(self.position[1]))
        self.scf.cf.param.set_value('kalman.initialZ', float(self.position[2]))
        self.scf.cf.param.set_value('kalman.initialYaw',
                                    float(self.initial_orientation[2]))

        self.reset_estimator()
        print('End Kalman Filter Reset')

        # vicon_pose_sending
        self.vicon_thread.daemon = True
        self.vicon_thread.start()

        # datalog
        self.datalog.start()

        # Take-off
        self.take_off(vx, vy)

    def take_off(self, vx, vy):
        # yaw is computed in order to direct the x-body axis in the same
        # direction of atan(vy, vx)
        yaw = math.atan2(vy, vx)
        yaw = math.degrees(yaw)
        vel_norm = np.linalg.norm(np.array([vx, vy]), 2)

        for i in np.arange(0.0, sc_v.DEFAULT_HEIGHT + 0.01, 0.1):
            self.scf.cf.commander.send_position_setpoint(self.position[0],
                                                         self.position[1],
                                                         i,
                                                         math.degrees(self.initial_orientation[2]))
            time.sleep(0.2)
        print('Take-off done')

        # redirecting yaw
        init_yaw_degree = math.degrees(self.initial_orientation[2])
        if init_yaw_degree < 0:
            init_yaw_degree += 360.0
        if init_yaw_degree - yaw > 0:
            align = np.arange(init_yaw_degree, yaw, -15)
        else:
            align = np.arange(init_yaw_degree, yaw, 15)
        for j in align:
            for i in range(2):
                self.scf.cf.commander.send_position_setpoint(self.position[0],
                                                             self.position[1],
                                                             sc_v.DEFAULT_HEIGHT,
                                                             j)
                time.sleep(0.1)
        for i in range(10):
            self.scf.cf.commander.send_position_setpoint(self.position[0],
                                                         self.position[1],
                                                         sc_v.DEFAULT_HEIGHT,
                                                         yaw)
            time.sleep(0.1)
        self.flying = True
        while not self.check_virtual_box():
            self.scf.cf.commander.send_hover_setpoint(vel_norm, 0.0, 0.0,
                                                      sc_v.DEFAULT_HEIGHT)

        self.get_state()
        print('Inside Virtual Box')

    def get_state(self):
        crazy.callback_mutex.acquire(blocking=True)
        self.position = sc_v.pos_estimate[0:2]
        self.velocity = sc_v.vel_estimate[0:2]
        self.yaw = sc_v.pos_estimate[2]
        self.yawrate = sc_v.vel_estimate[2] / 1000
        crazy.callback_mutex.release()

    def check_virtual_box(self):
        self.get_state()
        if self.position[0] > self.box[0] or self.position[0] < self.box[1]:
            return False
        elif self.position[1] > self.box[2] or self.position[1] < self.box[3]:
            return False
        return True

    def send_command(self, vx, vy, yawrate, dt=0.05):
        if self.flying:
            flag = self.check_virtual_box()
            if flag:
                self.scf.cf.commander.send_hover_setpoint(vx, vy,
                                                          yawrate,
                                                          sc_v.DEFAULT_HEIGHT)
                crazy.command_matlab.write(vx, vy, yawrate)
                time.sleep(dt)
                return True
            else:
                print('Out of Virtual Box')
                self.landing()
                return False

    def landing(self):
        self.flying = False
        self.get_state()
        land_pos = self.initial_position
        land_yaw = self.yaw
        print(f'Landing position:{land_pos}, landing yaw:{land_yaw}')
        for i in range(20):
            self.scf.cf.commander.send_position_setpoint(land_pos[0],
                                                         land_pos[1],
                                                         sc_v.DEFAULT_HEIGHT, land_yaw)
            time.sleep(0.2)
        for i in np.arange(0.5, 0, -0.05):
            self.scf.cf.commander.send_position_setpoint(land_pos[0],
                                                         land_pos[1],
                                                         i, land_yaw)
            time.sleep(0.2)
        self.scf.cf.commander.send_position_setpoint(land_pos[0],
                                                     land_pos[1],
                                                     -0.1, land_yaw)
        #time.sleep(0.1)
        crazy.run = False
        crazy.run_data = False
