import math
import threading
import time
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, script_variables as sc_v

sequence = [
    [-0.5, 0.0, 0.5],
    [-0.5, 1.0, 0.5],
    [0.5, 1.0, 0.5],
    [0.5, 1.0, 0.75],
    [0.5, 1.0, 1.0],
    [-0.5, 1.0, 1.0],
    [0.0, 0.0, 1.0],
    [0.0, 0.0, 0.75]
    ]   # This sequence is utilized to test the upgrade of the Kalman filter estimation

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

    scf.cf.param.set_value('stabilizer.estimator', 2)  # Set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')
    scf.cf.param.set_value('kalman.pNAcc_xy', 1.5) # Set the value for the KF
    scf.cf.param.set_value('kalman.pNAcc_z', 2.0)
    scf.cf.param.set_value('kalman.pNPos', 0.025)
    scf.cf.param.set_value('kalman.pNVel', 1.0)
    crazy.int_matlab.write("% x y z vx vy vz")

    # Kalman filter initialization with the initial position and the initial yaw
    InitialPos = np.array([0.0, 0.0, 0.0])
    InitialPos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    InitialOrient = sc_s.vicon. \
        GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)[0]
    # Conversion in meter
    scf.cf.param.set_value('kalman.initialX', float(InitialPos[0]/1000))
    scf.cf.param.set_value('kalman.initialY', float(InitialPos[1]/1000))
    scf.cf.param.set_value('kalman.initialZ', float(InitialPos[2]/1000))
    scf.cf.param.set_value('kalman.initialYaw', float(InitialOrient[2]))

    # crazy.reset_estimator(scf)
    scf.cf.param.set_value('kalman.resetEstimation', 1)
    time.sleep(1)
    scf.cf.param.set_value('kalman.resetEstimation', 0)
    time.sleep(0.5)

    print(f'Initial position and yaw of the drone: [x:{scf.cf.param.get_value("kalman.initialX")},'
          f'y:{scf.cf.param.get_value("kalman.initialY")},'
          f'z:{scf.cf.param.get_value("kalman.initialZ")},'
          f'yaw:{math.degrees(float(scf.cf.param.get_value("kalman.initialYaw")))}') # Yaw angle in degrees

    crazy.run = True
    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))

    # Set threads as daemon: this way they will terminate as soon as the main program terminates
    est_thread.daemon = True
    est_thread.start()
    datalog = crazy.datalog(scf)
    datalog.start()     # Starts the data collection for MATLAB analysis

    with PositionHlCommander(
                    scf,
                    x=InitialPos[0]/1000, y=InitialPos[1]/1000, z=InitialPos[2]/1000,
                    default_velocity=0.3,
                    default_height=0.5,
                    controller=PositionHlCommander.CONTROLLER_PID) as pc:

        print('Take Off') # Take off is done automatically inside the PositionHlCommander, to an height of 0.5 m

        # # Test utilized to check how the Kalman filter works
        # for j in sequence:
        #     for i in range(10):
        #         scf.cf.commander.send_position_setpoint(j[0],
        #                                                 j[1],
        #                                                 j[2], 0.0)
        #         time.sleep(0.5)

        # # Test utilized to check how a yaw rate command works
        # for j in np.arange(0, 361, 90): # In order to achieve a circumference
        #     for i in range(8):
        #         scf.cf.commander.send_position_setpoint(InitialPos[0]/1000,InitialPos[1]/1000,0.5,j)
        #         time.sleep(0.5)

        # Manual landing above the initial position, from an height of 0.5 m
        for i in np.arange(-0.5, 0.12, 0.1):
            for j in range(1):
                scf.cf.commander.send_position_setpoint(InitialPos[0] / 1000,
                                                        InitialPos[1] / 1000,
                                                        -i, 0.0)
                time.sleep(0.5)

    print("LANDING!")
    datalog.stop()