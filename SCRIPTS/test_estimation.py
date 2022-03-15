import logging
import math
import threading
import time
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, script_variables as sc_v
from vicon_dssdk import ViconDataStream


def pose_sending(sync_cf):
    """
    Sends the Crazyflie pose taken from the Vicon system to the drone
    estimator.
    :param sync_cf: Synchronization wrapper of the Crazyflie object.
    :type sync_cf: SyncCrazyflie object
    :return: None.
    :rtype: None
    """

    # Get a new frame from Vicon
    try:
        sc_s.vicon.GetFrame()
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error while getting a frame in the core! "
                      "--> %s", str(exc))

    # Get current drone position and orientation in Vicon
    sc_v.drone_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    sc_v.drone_or = sc_s.vicon. \
        GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                           sc_v.drone)[0]

    # Converts in meters
    sc_v.drone_pos = (float(sc_v.drone_pos[0] / 1000),
                      float(sc_v.drone_pos[1] / 1000),
                      float(sc_v.drone_pos[2] / 1000))

    # # Send to drone estimator
    # sync_cf.cf.extpos.send_extpose(sc_v.drone_pos[0], sc_v.drone_pos[1],
    #                                sc_v.drone_pos[2],
    #                                sc_v.drone_or[0], sc_v.drone_or[1],
    #                                sc_v.drone_or[2], sc_v.drone_or[3])
    #
    sync_cf.cf.extpos.send_extpos(sc_v.drone_pos[0], sc_v.drone_pos[1],
                                  sc_v.drone_pos[2])
    logging.debug("sent pose: %s %s",
                  str(sc_v.drone_pos), str(sc_v.drone_or))

    # Log to file
    crazy.vicon_matlab.write(sc_v.drone_pos[0], sc_v.drone_pos[1],
                       sc_v.drone_pos[2],
                       sc_v.drone_or[0], sc_v.drone_or[1],
                       sc_v.drone_or[2], sc_v.drone_or[3], time.time())


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
    scf.cf.param.set_value('kalman.pNAcc_xy', 1.5)  # Set the value for the KF
    scf.cf.param.set_value('kalman.pNAcc_z', 2.0)
    scf.cf.param.set_value('kalman.pNPos', 0.025)
    scf.cf.param.set_value('kalman.pNVel', 1.0)

    # Kalman filter initialization with the initial position and the initial yaw
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
                                  args=(1, crazy.vicon2drone_period,
                                        pose_sending, scf))

    # Set threads as daemon: this way they will terminate as soon as the
    # main program terminates
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

        print('Take Off')

        # # Test utilized to check how the Kalman filter works
        for j in sequence:
            for i in range(10):
                scf.cf.commander.send_position_setpoint(j[0],
                                                        j[1],
                                                        j[2], 0.0)
                time.sleep(0.5)

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
