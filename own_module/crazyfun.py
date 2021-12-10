from __future__ import print_function
import logging
import os
import time
import numpy as np
import math
from pytictoc import TicToc
from datetime import datetime, timedelta
from pathlib import Path
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from vicon_dssdk import ViconDataStream
from own_module import script_variables as sc_v, script_setup as sc_s
from cflib.positioning.position_hl_commander import PositionHlCommander
import signal

t = TicToc()
first_iter = True

def datetime2matlabdatenum(dt):
    """
    Converts Python 'datetime' object into MATLAB 'datenum' format,
    including microseconds.
    source: https://stackoverflow.com/a/9391765/5627942

    :param dt: Python 'datetime' object.
    :type dt: datetime object
    :return: Time indication in MATLAB 'datenum' format.
    :rtype: float
    """

    # Synchronize reference start time (Python and MATLAB dates starts at
    # different point in time)
    mdn = dt + timedelta(days=366)
    # Computes seconds...
    frac_seconds = (dt - datetime(dt.year, dt.month, dt.day, 0, 0,
                                  0)).seconds / \
                   (24.0 * 60.0 * 60.0)
    # ... and microseconds
    frac_microseconds = dt.microsecond / (24.0 * 60.0 * 60.0 * 1000000.0)

    return mdn.toordinal() + frac_seconds + frac_microseconds


def print_callback(timestamp, data, log_conf):
    """
    Prints gathered data to a specific file.

    :param data: Data to be logged.
    :type data:
    :return: None.
    :rtype: None
    """

    pos_x = data['kalman.stateX']
    pos_y = data['kalman.stateY']
    pos_z = data['kalman.stateZ']
    roll = data['stabilizer.roll']
    pitch = data['stabilizer.pitch']
    yaw = data['stabilizer.yaw']
    global battery
    battery = data['pm.state']
    # Print state estimate to file

    int_matlab.write(pos_x, pos_y, pos_z, roll, pitch, yaw)


def print_callback_vel_est(timestamp, data, log_conf):
    """
    Prints gathered data to a specific file.

    :param data: Data to be logged.
    :type data:
    :return: None.
    :rtype: None
    """
    global first_iter, old_position

    # if first_iter:
    #     first_iter = False
    #
    #     # Get current drone position and orientation in Vicon
    #     old_position = sc_s.vicon. \
    #         GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    #
    #
    #     # Converts in meters
    #     old_position = np.array([float(old_position[0] / 1000),
    #                       float(old_position[1] / 1000),
    #                       float(old_position[2] / 1000)])
    # else:
    #     dt = t.tocvalue() #seconds
    #     new_position = sc_s.vicon. \
    #         GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
    #     new_position = np.array([float(new_position[0] / 1000),
    #                       float(new_position[1] / 1000),
    #                       float(new_position[2] / 1000)])
    #     print(f'new_pos:{new_position}, old_position:{old_position}, dt:{dt}')
    #     vicon_vel = (new_position-old_position)/dt
    #     old_position = new_position

    pos_x = data['kalman.stateX']
    pos_y = data['kalman.stateY']
    pos_z = data['kalman.stateZ']
    v_x = data['kalman.statePX']
    v_y = data['kalman.statePY']
    v_z = data['kalman.statePZ']

    global battery
    battery = data['pm.state']
    # Print state estimate to file
    int_matlab.write(pos_x, pos_y, pos_z, v_x, v_y, v_z)
    # int_matlab.write(pos_x, pos_y, pos_z, roll, pitch, yaw)
    # t.tic()


def datalog_async(sync_crazyflie, log_conf):
    """
    Adds a callback function to the LogTable of the drone, which will be
    executed every time new data have been received.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object.
    :type sync_crazyflie: SyncCrazyflie object
    :param log_conf: Representation of a log configuration.
    :type log_conf: LogConfig object
    :return: None.
    :rtype: None
    """

    crazyflie = sync_crazyflie.cf
    crazyflie.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(print_callback_vel_est)


def datalog(sync_crazyflie):
    """
    Prepares the call to "datalog_async" and achieves it.
    Adds a log configuration to the Crazyflie with desired variables
    logged every 'datalog_period' milliseconds.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object.
    :type sync_crazyflie: SyncCrazyflie object
    :return: Log configuration to get the drone internal estimate.
    :rtype: LogConfig object
    """

    global datalog_period

    measure_log = LogConfig(name='TotalEstimate', period_in_ms=datalog_period)
    measure_log.add_variable('kalman.stateX', 'float')
    measure_log.add_variable('kalman.stateY', 'float')
    measure_log.add_variable('kalman.stateZ', 'float')
    measure_log.add_variable('kalman.statePX', 'float')
    measure_log.add_variable('kalman.statePY', 'float')
    measure_log.add_variable('kalman.statePZ', 'float')
    # measure_log.add_variable('stabilizer.roll', 'float')
    # measure_log.add_variable('stabilizer.pitch', 'float')
    # measure_log.add_variable('stabilizer.yaw', 'float')

    measure_log.add_variable('pm.state', 'int8_t')

    datalog_async(sync_crazyflie, measure_log)

    return measure_log


def reset_estimator(scf):
    """
    Resets drone internal estimator.

    :param scf: Synchronization wrapper of the Crazyflie object.
    :type scf: SyncCrazyflie object
    :return: None.
    :rtype: None
    """

    cf = scf.cf
    # Actual reset
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)

    # Wait for the position estimator to converge before doing anything
    wait_for_position_estimator(cf)


def wait_for_position_estimator(scf):
    """
    Waits for the state estimation variance to have a small variation, under a
    given threshold.

    :param scf: Synchronization wrapper of the Crazyflie object.
    :type scf: SyncCrazyflie object
    :return: None.
    :rtype: None
    """

    global posvar_period
    logging.info('Waiting for the estimator to converge...')

    # Setting some logging configurations
    log_config = LogConfig(name='Kalman Pos Variance',
                           period_in_ms=posvar_period)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    # Initial variance values
    var_x_history = [1000] * 10
    var_y_history = [1000] * 10
    var_z_history = [1000] * 10

    # Given threshold
    threshold = 0.3  # 001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            # adds current variance values, keeping the vector of the
            # same length
            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            # Compute max and min variance values
            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            logging.debug("dx: %s dy: %s dz: %s",
                          max_x - min_x, max_y - min_y, max_z - min_z)

            # Stop the waiting action if all 3D variances are beyond threshold
            if (max_x - min_x) < threshold and \
                    (max_y - min_y) < threshold and \
                    (max_z - min_z) < threshold:
                break


def sign(x):
    """
    Return whether the argument is positive, negative or neither.

    :param x: Input data.
    :type x: any
    :return: Argument sign.
    :rtype: integer/exception
    """

    if x > 0:
        return +1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        raise Exception("The argument is not a number.")


# Good reading for generator functions:
# https://www.programiz.com/python-programming/generator
def repeat_fun(period, func, *args):
    """
    Uses an internal generator function to run another function
    at a set interval of time.

    :param period: Period at which repeat the execution of func.
    :type period: integer
    :param func: Function to execute.
    :type func: callable
    :param args: Arguments to pass to func.
    :type args: any
    :return: None.
    :rtype: None
    """

    def time_tick():
        """
        Generator function that scan the passing of the desired period of time.

        :return: Yields the passed time.
        :rtype: iterator
        """
        t = time.time()
        while True:
            t += period
            yield max(t - time.time(), 0)

    # Initiates the generator function
    tick = time_tick()

    # Uses a global flag to stop the execution
    while run:
        time.sleep(next(tick))
        func(*args)


# standard argument for signal handler calls
def handler_stop_signal(signum, frame):
    """
    Sets the global flag 'run' to False to stop other threads.

    :return: None.
    :rtype: None
    """

    global run
    run = False


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

    # Send to drone estimator
    sync_cf.cf.extpos.send_extpose(sc_v.drone_pos[0], sc_v.drone_pos[1],
                                   sc_v.drone_pos[2],
                                   sc_v.drone_or[0], sc_v.drone_or[1],
                                   sc_v.drone_or[2], sc_v.drone_or[3])

    # logging.debug("sent pose: %s %s",
    #               str(sc_v.drone_pos), str(sc_v.drone_or))

    # Log to file
    vicon_matlab.write(sc_v.drone_pos[0], sc_v.drone_pos[1],
                       sc_v.drone_pos[2],
                       sc_v.drone_or[0], sc_v.drone_or[1],
                       sc_v.drone_or[2], sc_v.drone_or[3])


def set_wand_track():
    """
    Function to set the next setpoint, read as a Wand position from a file.

    :return: None.
    :rtype: None
    """

    global wand_setpoint
    wand_setpoint = wand_matlab.read_point()


def wand_sending():
    """
    Logs the Wand position to a file.

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
    sc_v.wand_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]

    # Converts in meters
    sc_v.wand_pos = (float(sc_v.wand_pos[0] / 1000),
                     float(sc_v.wand_pos[1] / 1000),
                     float(sc_v.wand_pos[2] / 1000))

    logging.debug("Acquired Wand position: %s", str(sc_v.wand_pos))

    # Log to file
    wand_matlab.write(sc_v.wand_pos[0],
                      sc_v.wand_pos[1],
                      sc_v.wand_pos[2])


# Monkey-patch; useful:
# https://stackoverflow.com/questions/5626193/what-is-monkey-patching/6647776#6647776
def go_to_nosleep(self,
                  x, y, z=PositionHlCommander.DEFAULT,
                  velocity=PositionHlCommander.DEFAULT):
    """
    Monkey-patch of the PositionHlCommander standard 'go_to' method.

    :param self: PositionHLCommander object
    :param x: X coordinate [m]
    :param y: Y coordinate [m]
    :param z: Z coordinate [m]
    :param velocity: the velocity (meters/second)
    :return: None.
    :rtype: None
    """

    z = self._height(z)

    dx = x - self._x
    dy = y - self._y
    dz = z - self._z
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    if distance > 0.0:
        duration_s = distance / self._velocity(velocity)
        self._hl_commander.go_to(x, y, z, 0, duration_s)
        # time.sleep(duration_s)

        self._x = x
        self._y = y
        self._z = z


class MatlabPrint:
    """
    Class for datafile handling.

    flag = 0 -> points obtained from Vicon
    flag = 1 -> setpoints sent to the drone
    flag = 2 -> drone internal estimation
    flag = 3 -> wand position in Vicon
    """

    write_descriptor = 0
    read_descriptor = 0

    def __init__(self, flag=0):
        """
        Open the file in which to append data. Creates it if it doesn't exist.

        :param flag: Indication for the type of data to log into the file.
        :type flag: integer
        """

        file_name = os.path.normpath(__file__).split(os.sep)[-1][:-3]

        # Different kind of data to manage
        print_type = {
            0: "vicon_data",
            1: "setpoint_data",
            2: "internal_data",
            3: "wand_data",
            4: "command_data"
        }

        folder = print_type.get(flag, "Unmanaged")

        # Creates folder if it does not exist
        if not os.path.exists("../" + folder):
            os.makedirs("../" + folder)

        mat_file = "../" + folder + "/" + file_name \
                   + datetime.now().strftime("__%Y%m%d_%H%M%S")

        mat_file = mat_file + ".txt"
        ff = os.path.normpath(
            os.path.join(Path(__file__).parent.absolute(), mat_file))

        self.write_descriptor = open(ff, 'a')
        self.read_descriptor = open(ff, 'r')

    def __del__(self):
        """
        Class destructor.

        :return: None.
        :rtype: None
        """

        # Useful for the flush() argument:
        # https://stackoverflow.com/questions/15608229/what-does-prints-flush-do

        if self.write_descriptor:
            self.write_descriptor.flush()  # ensures complete output to file
            self.write_descriptor.close()

        if self.read_descriptor:
            self.read_descriptor.flush()  # ensures complete output to file
            self.read_descriptor.close()

    def __enter__(self):
        """
        Allows the class to be used in environment creations
        with the keyword 'with'.

        :return: Class instanced.
        :rtype: MatlabPrint object
        """

        return self

    def __exit__(self):
        """
        To specify what to do at environment destruction.

        :return: None.
        :rtype: None
        """
        self.__del__()

    def write(self, *args):
        """
        Append content to file, adding a timestamp in MATLAB 'datenum' format.

        :param args: What to print to the file.
        :type args: any
        :return: None.
        :rtype: None
        """

        # Create a single string with all data passed to the function.
        # You can use a \n as argument to get a newline.
        s = ""
        for arg in args:
            s = s + "{} ".format(str(arg))

        # timestamp to use in MATLAB
        s = s + " {}".format(str(datetime2matlabdatenum(datetime.now())))
        print(s, file=self.write_descriptor, flush=True)

    def read_point(self):
        """
        Reads a line from the class file and return an array
        with the first three elements: this will represent a point's
        coordinates.

        :return:
        :rtype:
        """

        if self.write_descriptor:
            self.write_descriptor.close()

        line = self.read_descriptor.readline().split(" ")
        element = line[0]

        if element == "%":  # Skip the header line
            line = self.read_descriptor.readline().split(" ")

        point = []
        index = 0
        for element in line:
            element = element.strip()
            point.append(element)
            index += 1
            if index >= 3:
                break
        return point


###################################################################
# Extra functions for incomplete scripts
def check_obstacle(poscom):
    logging.info("Getting object setpoint...")
    obj_pos = sc_s.vicon. \
        GetSegmentGlobalTranslation('Obstacle', 'OneMarker')[0]

    dist_array = np.array(sc_v.drone_pos - obj_pos)

    theta_ver = math.atan2(dist_array[2], dist_array[0])
    theta_hor = math.atan2(dist_array[1], dist_array[0])

    # if it's not the first time the object has been registered
    if len(tv_prec) and len(th_prec):
        ver_warning = (0 < theta_ver < tv_prec[-1]) or \
                      (0 > theta_ver > tv_prec[-1])
        hor_warning = (0 < theta_hor < th_prec[-1]) or \
                      (0 > theta_hor > th_prec[-1])

        if ver_warning and hor_warning and \
                (np.linalg.norm(dist_array) <= safety_threshold):
            avoid(poscom, dist_array)

    tv_prec.append(theta_ver)
    th_prec.append(theta_hor)


def avoid(vehicle, dist):
    """
    Computes the direction the drone has to move to avoid the incoming
    obstacle.

    :param vehicle: Drone's commander.
    :type vehicle: PositionHLCommander
    :param dist: 3D array containing the drone-obstacle distance.
    :type dist: numpy.array
    :return: None.
    :rtype: None
    """

    movement = sc_v.drone_pos

    # Decide which 3D coordinate to change
    direction = min(dist)
    ind = dist.index(direction)

    # Update the coordinate
    movement[ind] += -1 * sign(direction) * 0.3

    # Move the Crazyflie
    vehicle.go_to(movement[0], movement[1], movement[2])
###################################################################


# Global variables used
log_pos_x = 0
log_pos_y = 0
log_pos_z = 0
log_roll = 0
log_pitch = 0
log_yaw = 0
battery = 0

# Period at which Vicon data will be sent to the Crazyflie
vicon2drone_period = 0.1  # [s]
wand_period = 0.1  # [s]
obstacle_period = 0.1  # [s]

# Period used in Log configurations
datalog_period = 10  # ms
posvar_period = 500  # ms

# Parameters used in the collision avoidance script
safety_threshold = 10  # cm
tv_prec = []
th_prec = []
wand_setpoint = [0, 0, 0]

run = True
vbat = 0

# Signal handling
# A PyCharm registry option has to be changed, according to
# https://youtrack.jetbrains.com/issue/PY-13316#focus=Comments-27-4240420.0-0,
# in order to catch the red 'Stop program' button press in the console
signal.signal(signal.SIGINT, handler_stop_signal)
signal.signal(signal.SIGTERM, handler_stop_signal)

# Istances of the MatlabPrint classes with relative logfile creation
vicon_matlab = MatlabPrint(flag=0)
set_matlab = MatlabPrint(flag=1)
int_matlab = MatlabPrint(flag=2)
wand_matlab = MatlabPrint(flag=3)
command_matlab = MatlabPrint(flag= 4)
command_matlab.write(0,0,0)
safety_offset = 0.3
time_limit = 60  # [s]
tracking = True
pos_limit = 0.001  # [m]
max_equal_pos = 10
