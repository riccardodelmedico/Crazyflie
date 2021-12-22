import logging
#from cflib.positioning.motion_commander import MotionCommander
#from cflib.positioning.position_hl_commander import PositionHlCommander
import math
import threading
import time
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v

# il parametro box di inizializzazione definisce una virtual box all'interno
# della quale il drone deve rimanere durante la guida, per motivi di sicurezza.
# box[0]: limite su x positiva
# box[1]: limite su x negativa
# box[2]: limite su y positiva
# box[3]: limite su y negativa
# i limiti sono da intendere a partire dall'origine del world frame


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
        self.flying =False
        self.yaw = 0.0
        self.box = box
        self.vicon_thread = threading.Thread(target=crazy.repeat_fun,
                                             args=(crazy.vicon2drone_period,
                                             crazy.pose_sending, self.scf))
        self.datalog = crazy.datalog(self.scf)

    def set_value(self, string, value):
        self.scf.cf.param.set_value(string, value)

    def reset_estimator(self):
        cf = self.scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')

    def start(self, vx, vy):
        self.position = np.array(sc_s.vicon. \
            GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0])
        self.position /= 1000
        self.initial_orientation = sc_s.vicon. \
            GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)[0]
        print(f'init_pos: x{self.position[0]}, y:{self.position[1]}')

        self.scf.cf.param.set_value('kalman.initialX', float(self.position[0]))
        self.scf.cf.param.set_value('kalman.initialY', float(self.position[1]))
        self.scf.cf.param.set_value('kalman.initialZ', float(self.position[2]))
        self.scf.cf.param.set_value('kalman.initialYaw', float(self.initial_orientation[2]))
        print('Start Reset Kalman Filter')
        self.reset_estimator()
        print('End Reset Kalman Filter')
        # vicon_pose_sending
        self.vicon_thread.daemon = True
        self.vicon_thread.start()

        # datalog
        self.datalog.start()

        # Take-off
        self.take_off(vx, vy)

    def take_off(self, vx, vy):
        # calcolo yaw in modo da essere orientato con asse x (body) nella
        # direzione data da vx e vy
        yaw = math.atan2(vy, vx)
        yaw = math.degrees(yaw)
        vel_norm = np.linalg.norm(np.array([vx, vy]), 2)

        for i in np.arange(0.0, sc_v.DEFAULT_HEIGHT + 0.01, 0.1):
            self.scf.cf.commander.send_position_setpoint(self.position[0],
                                                         self.position[1],
                                                         i,
                                                         math.degrees(self.initial_orientation[2]))
            time.sleep(0.2)
        for j in np.arange(math.degrees(self.initial_orientation[2]),yaw,-10):
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
            self.scf.cf.commander.send_hover_setpoint(vel_norm, 0.0,
                                                      0.0,
                                                      sc_v.DEFAULT_HEIGHT)

        self.get_state()
        print(f'Guidance Velocity reached. Pos:{self.position}')

    def get_state(self):
        crazy.callback_mutex.acquire(blocking=True)
        self.position = sc_v.pos_estimate[0:2]
        self.velocity = sc_v.vel_estimate[0:2]
        self.yaw = sc_v.pos_estimate[2]
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
                self.landing()
                return  False

    def landing(self):
        self.flying = False
        for i in np.arange(0.5, -0.11, -0.1):
            self.get_state()
            self.scf.cf.commander.send_position_setpoint(self.position[0],
                                                         self.position[1],
                                                         i, self.yaw)
            time.sleep(0.2)
        crazy.run = False

    # def stop(self):
    #     pass

#
# with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
#     print('Main Start')
#     drone = DroneManager(scf, 1.5, 2.0, 0.025, 1.0)
#     drone.start(0.0, 0.4)
#     while drone.send_command(1.0, 0.0,  -0.25016528840485525):
#         continue
#     print('Main Finished')











