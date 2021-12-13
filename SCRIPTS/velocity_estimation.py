import logging
import math
import threading
import time
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


#print("5s before taking off! Prepare to take a video!")
# time.sleep(5)

sequence = [
    [0.0, 0.0, 0.5],
    [0.0, 1.0, 0.5],
    [1.0, 1.0, 0.5],
    [1.0, 1.0, 1],
    [0.0, 1.0, 1],
    [0.0, 0.0, 1],
    [0.0, 0.0, 0.5]
]

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    # scf.cf.param.set_value('stabilizer.controller', 1)
    scf.cf.param.set_value('commander.enHighLevel', '1')
    scf.cf.param.set_value('kalman.pNAcc_xy', 1.5)
    scf.cf.param.set_value('kalman.pNAcc_z', 2.0)
    scf.cf.param.set_value('kalman.pNPos', 0.025)
    scf.cf.param.set_value('kalman.pNVel', 1)
    # scf.cf.param.set_value('velCtlPid.vxKd', 0.05)
    # scf.cf.param.set_value('velCtlPid.vyKd', 0.05)
    # scf.cf.param.set_value('velCtlPid.vzKd', 0.001)


    crazy.int_matlab.write("% x y z vx vy vz")

    # Kalman filter initialisation
    InitialPos = np.array([0.0, 0.0, 0.0])
    InitialPos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]

    scf.cf.param.set_value('kalman.initialX', float(InitialPos[0]/1000))
    scf.cf.param.set_value('kalman.initialY', float(InitialPos[1]/1000))
    scf.cf.param.set_value('kalman.initialZ', float(InitialPos[2]/1000))
    # scf.cf.param.set_value('velCtlPid.vxKp', 50.0)
    # scf.cf.param.set_value('velCtlPid.vyKp', 70.0)
    # scf.cf.param.set_value('velCtlPid.vxKi',2.0)
    # scf.cf.param.set_value('velCtlPid.vyKi', 3.0)
    #crazy.reset_estimator(scf)
    scf.cf.param.set_value('kalman.resetEstimation', 1)
    time.sleep(1)
    scf.cf.param.set_value('kalman.resetEstimation', 0)
    time.sleep(0.5)
    pnv = scf.cf.param.get_value('kalman.pNVel')
    pnpos = scf.cf.param.get_value('kalman.pNPos')
    pnatt = scf.cf.param.get_value('kalman.pNAtt')
    vel_p_x = scf.cf.param.get_value('velCtlPid.vxKp')
    vel_p_y = scf.cf.param.get_value('velCtlPid.vyKp')
    vel_p_z = scf.cf.param.get_value('velCtlPid.vzKp')

    vel_i_x = scf.cf.param.get_value('velCtlPid.vxKi')
    vel_i_y = scf.cf.param.get_value('velCtlPid.vyKi')
    vel_i_z = scf.cf.param.get_value('velCtlPid.vzKi')

    vel_d_x = scf.cf.param.get_value('velCtlPid.vxKd')
    vel_d_y = scf.cf.param.get_value('velCtlPid.vyKd')
    vel_d_z = scf.cf.param.get_value('velCtlPid.vzKd')

    vel_ffx = scf.cf.param.get_value('velCtlPid.vxKFF')
    vel_ffy = scf.cf.param.get_value('velCtlPid.vyKFF')

    print(f' parametri controllo PID proporzionale: x {vel_p_x}, y {vel_p_y}, z {vel_p_z}')
    print(f' parametri controllo PID integrale: x {vel_i_x}, y {vel_i_y}, z {vel_i_z}')
    print(f' parametri controllo PID derivato: x {vel_d_x}, y {vel_d_y}, z {vel_d_z}')
    print(f' parametri di FeedForward: x {vel_ffx}, y {vel_ffy}')

    print(f'processNoise: vel: {pnv}, pos: {pnpos}, att: {pnatt}')
    print(f'inital position param[x:{scf.cf.param.get_value("kalman.initialX")},y:{scf.cf.param.get_value("kalman.initialY")},z:{scf.cf.param.get_value("kalman.initialZ")}')


    crazy.run = True
    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))

    # Set threads as daemon: this way they will terminate as soon as
    # the main program terminates
    est_thread.daemon = True

    est_thread.start()
    datalog = crazy.datalog(scf)
    datalog.start()
    # time.sleep(5)
    # scf.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
    # time.sleep(0.01)
    #
    # for i in range(5):
    #     scf.cf.commander.send_setpoint(0.0, 0.0, 0, 47000)
    #     time.sleep(0.3)
    # for i in range(10):
    #     scf.cf.commander.send_setpoint(0.0, 0.0, 0, 40500)
    #     time.sleep(0.3)
    vel_ref = np.array([0.0, 0.0])
    vel_error = np.array([0.0, 0.0])
    k_x = 0.1
    k_y = 0.1
    k_vel = np.array([[k_x, 0.0], [0.0, k_y]])
    with PositionHlCommander(
                    scf,
                    x=InitialPos[0]/1000, y=InitialPos[1]/1000, z=InitialPos[2]/1000,
                    default_velocity=0.3,
                    default_height=0.5,
                    controller=PositionHlCommander.CONTROLLER_PID) as pc:

        print('Take Off ....')
        time.sleep(1)
        plot = True
        for i in range(1):
            for j in np.arange(0.0, 6.28, 0.05):
                vel_ref[0:2] = (math.sin(j)/4, math.sin(j)/4)
                if plot:
                    CommandPos = sc_s.vicon. \
                        GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
                    crazy.command_matlab.write(float(CommandPos[0] / 1000),
                                               float(CommandPos[1] / 1000),
                                               float(CommandPos[2] / 1000),
                                               0.0, 0.0)
                    plot = False


                crazy.callback_mutex.acquire(blocking=True)
                vel_error = vel_ref - sc_v.vel_estimate[0:2]
                crazy.callback_mutex.release()

                vel_comm = vel_ref[0:2] + k_vel.dot(vel_error)
                scf.cf.commander.send_hover_setpoint(vel_comm[0],
                                                     vel_comm[1],
                                                     0.0,
                                                     0.5)
                crazy.command_matlab.write(vel_ref[0], vel_ref[1], 0.0,
                                           vel_comm[0], vel_comm[1])
                time.sleep(0.1)
        for k in range(5):
            scf.cf.commander.send_position_setpoint(0.0,0.0, 0.5, 0.0)
            time.sleep(0.2)
        for i in range(3):
            scf.cf.commander.send_position_setpoint(0.0, 0.0, 0.2, 0.0)
            time.sleep(0.2)

        # pc.go_to(InitialPos[0]/1000, InitialPos[1]/1000, 0.5)

    #
    # scf.cf.commander.send_position_setpoint(0.0,0.0,0.5,0.0)
    # time.sleep(2)
    # scf.cf.commander.send_position_setpoint(0.0,0.0,0.05,0.0)
    #
    # with PositionHlCommander(
    #         scf,
    #         x=InitialPos[0]/1000, y=InitialPos[1]/1000, z=InitialPos[2]/1000,
    #         default_velocity=0.3,
    #         default_height=0.5,
    #         controller=PositionHlCommander.CONTROLLER_PID) as pc:
    #
    #     logging.info("Take-off!")
    #     #time.sleep(1)
    #     lowPowerCount = 0
    #
    #     for setpoint in sequence:
    #         pc.go_to(setpoint[0], setpoint[1], setpoint[2])
        # while lowPowerCount < 5:
        #     logging.info("Current battery state is %d", crazy.battery)
        #
        #     if crazy.battery == 3:
        #         lowPowerCount = lowPowerCount + 1
        #     else:
        #         lowPowerCount = 0

    # time.sleep(5)
    print("LANDING!")
    datalog.stop()