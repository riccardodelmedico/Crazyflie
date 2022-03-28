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
    sc_v.drone_pose = sc_s.vicon. \
        GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)[0]

    # Converts in meters
    sc_v.drone_pos = (float(sc_v.drone_pos[0] / 1000),
                      float(sc_v.drone_pos[1] / 1000),
                      float(sc_v.drone_pos[2] / 1000))

    # ------- USER CAN DECIDE WHETHER USE POSE OR POS CORRECTION ------- #
    # # Send to drone estimator
    # sync_cf.cf.extpos.send_extpose(sc_v.drone_pos[0], sc_v.drone_pos[1],
    #                                sc_v.drone_pos[2],
    #                                sc_v.drone_or[0], sc_v.drone_or[1],
    #                                sc_v.drone_or[2], sc_v.drone_or[3])
    #
    sync_cf.cf.extpos.send_extpos(sc_v.drone_pos[0], sc_v.drone_pos[1],
                                  sc_v.drone_pos[2])
    # ------------------------------------------------------------------ #

    logging.debug("sent pose: %s %s",
                  str(sc_v.drone_pos), str(sc_v.drone_or))

    # Log to file
    crazy.vicon_matlab.write(sc_v.drone_pos[0], sc_v.drone_pos[1],
                             sc_v.drone_pos[2],
                             sc_v.drone_or[0], sc_v.drone_or[1],
                             sc_v.drone_or[2], sc_v.drone_or[3], time.time())


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

    scf.cf.param.set_value('stabilizer.estimator', 2)  # Set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    # ------- USER CAN DECIDE KALMAN FILTER PARAMETERS ------- #
    scf.cf.param.set_value('kalman.pNAcc_xy', 1.5)  # Set the value for the KF
    scf.cf.param.set_value('kalman.pNAcc_z', 2.0)
    scf.cf.param.set_value('kalman.pNPos', 0.025)
    scf.cf.param.set_value('kalman.pNVel', 1.0)
    # -------------------------------------------------------- #

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

    init_posx = InitialPos[0]/1000
    init_posy = InitialPos[1]/1000
    init_posz = InitialPos[2]/1000
    init_yaw = InitialOrient[2]
    init_yaw_degree = 0
    yaw = 90
    starting_posx = -1.25
    starting_posy = -1.5
    starting_vel = 1.0
    starting_omega = 3.0
    starting_omega_degree = math.degrees(starting_omega)

    # crazy.reset_estimator(scf)
    scf.cf.param.set_value('kalman.resetEstimation', 1)
    time.sleep(1)
    scf.cf.param.set_value('kalman.resetEstimation', 0)
    time.sleep(0.5)

    print(f'Initial position and yaw of the drone: [x:{scf.cf.param.get_value("kalman.initialX")},'
          f'y:{scf.cf.param.get_value("kalman.initialY")},'
          f'z:{scf.cf.param.get_value("kalman.initialZ")},'
          f'yaw:{math.degrees(float(scf.cf.param.get_value("kalman.initialYaw")))}')  # Yaw angle in degrees

    crazy.run = True
    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(1, crazy.vicon2drone_period,
                                        pose_sending, scf))

    # Set threads as daemon: this way they will terminate as soon as the
    # main program terminates
    est_thread.daemon = True
    est_thread.start()
    datalog = crazy.datalog_acc(scf)

    with PositionHlCommander(
                    scf,
                    x=InitialPos[0]/1000, y=InitialPos[1]/1000, z=InitialPos[2]/1000,
                    default_velocity=0.3,
                    default_height=0.5,
                    controller=PositionHlCommander.CONTROLLER_PID) as pc:

        for i in range(25):
            scf.cf.commander.send_position_setpoint(starting_posx, starting_posy, sc_v.DEFAULT_HEIGHT, init_yaw_degree)
            time.sleep(0.2)

        if init_yaw_degree < 0:
            init_yaw_degree += 360.0
        if init_yaw_degree - yaw > 0:
            align = np.arange(init_yaw_degree, yaw, -15)
        else:
            align = np.arange(init_yaw_degree, yaw, 15)
        for j in align:
            for i in range(10):
                scf.cf.commander.send_position_setpoint(starting_posx,
                                                        starting_posy,
                                                        sc_v.DEFAULT_HEIGHT,
                                                        j)
                print(j)
                time.sleep(0.1)

        datalog.start()

        for i in range(20):
            scf.cf.commander.send_hover_setpoint(starting_vel, 0.0, 0.0, sc_v.DEFAULT_HEIGHT)
            time.sleep(0.1)

        for i in range(100):
            scf.cf.commander.send_hover_setpoint(starting_vel, 0.0, starting_omega_degree, sc_v.DEFAULT_HEIGHT)
            time.sleep(0.1)
        datalog.stop()
        land_x = sc_v.drone_pos[0]
        land_y = sc_v.drone_pos[1]
        land_yaw = crazy.prova_yaw

        for i in range(30):
            scf.cf.commander.send_position_setpoint(land_x, land_y, sc_v.DEFAULT_HEIGHT, land_yaw)
            time.sleep(0.1)

        # Manual landing above the initial position, from a height of 0.5 m
        for i in np.arange(-0.5, 0.12, 0.1):
            for j in range(1):
                scf.cf.commander.send_position_setpoint(land_x, land_y, -i, land_yaw)
                time.sleep(0.5)

    print("LANDING!")

