from __future__ import print_function
import os
import time
from datetime import datetime
from pathlib import Path
from cflib.crazyflie.log import LogConfig
from own_module import script_variables as sc_v
import signal
import threading
import ctypes
import numpy as np

# Increase resolution of PC timer:
# time.sleep() becomes far more precise
winmm = ctypes.WinDLL('winmm')
winmm.timeBeginPeriod(1)

# The value of estimation_mode changes the callback:
# estimation_mode = False => callback used for guidance
# estimation_mode = True => callback used for estimation test
estimation_mode = True


def print_callback_guidance(timestamp, data, log_conf):
    """
    Prints gathered data to a specific file.

    :param data: Data to be logged.
    :type data:
    :return: None.
    :rtype: None
    """
    callback_mutex.acquire(blocking=True)
    pos_x = sc_v.pos_estimate[0] = data['kalman.stateX']
    pos_y = sc_v.pos_estimate[1] = data['kalman.stateY']
    yaw = sc_v.pos_estimate[2] = data['stabilizer.yaw']
    v_x = sc_v.vel_estimate[0] = data['kalman.statePX']
    v_y = sc_v.vel_estimate[1] = data['kalman.statePY']
    yaw_rate = sc_v.vel_estimate[2] = data['stateEstimateZ.rateYaw']
    callback_mutex.release()

    # Print state estimate to file
    int_matlab.write(pos_x, pos_y, yaw, v_x, v_y, yaw_rate, time.time())


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

    # Print state estimate to file
    int_matlab.write(pos_x, pos_y, pos_z, roll, pitch, yaw, time.time())


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
    if estimation_mode:
        log_conf.data_received_cb.add_callback(print_callback)
    else:
        log_conf.data_received_cb.add_callback(print_callback_guidance)


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
    if estimation_mode:
        measure_log.add_variable('kalman.stateX', 'float')
        measure_log.add_variable('kalman.stateY', 'float')
        measure_log.add_variable('kalman.stateZ', 'float')
        measure_log.add_variable('stabilizer.roll', 'float')
        measure_log.add_variable('stabilizer.pitch', 'float')
        measure_log.add_variable('stabilizer.yaw', 'float')
    else:
        measure_log.add_variable('kalman.stateX', 'float')
        measure_log.add_variable('kalman.stateY', 'float')
        measure_log.add_variable('stateEstimateZ.rateYaw', 'float')
        measure_log.add_variable('kalman.statePX', 'float')
        measure_log.add_variable('kalman.statePY', 'float')
        measure_log.add_variable('stabilizer.yaw', 'float')

    datalog_async(sync_crazyflie, measure_log)

    return measure_log


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
def repeat_fun(thread_type, period, func, *args):
    """
    Uses an internal generator function to run another function
    at a set interval of time.

    :param thread_type: Type of periodic thread (Guidance or DataCore)
    :type thread_type: bool
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
    if thread_type:
        while run:
            func(*args)
            time.sleep(next(tick))
    else:
        while run_data:
            func(*args)
            time.sleep(next(tick))


# standard argument for signal handler calls
def handler_stop_signal(signum, frame):
    """
    Sets the global flag 'run' to False to stop other threads.

    :return: None.
    :rtype: None
    """

    global run
    run = False


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
            4: "command_data",
            5: "guidance_data",
            6: "core_data"
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
            if hasattr(arg, '__iter__'):
                for elem in arg:
                    s = s + "{} ".format(str(elem))
            else:
                s = s + "{} ".format(str(arg))

        # timestamp to use in MATLAB
        # s = s + " {}".format(str(datetime2matlabdatenum(datetime.now())))
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


class AsyncMatlabPrint(MatlabPrint):
    def __init__(self, num_data, flag=0):
        super().__init__(flag)
        self.saved_data = np.zeros((num_data, 1))
        self.first_acquisition = False

    def append(self, data):
        dim_data = np.shape(data)
        dim_store = np.shape(self.saved_data)
        if dim_data[0] != dim_store[0]:
            print('[ERROR] passed data has wrong dimension;'
                  ' data will not be saved')
            return False
        if not self.first_acquisition:
            self.saved_data = data
            self.first_acquisition = True
        else:
            self.saved_data = np.concatenate((self.saved_data, data), axis=1)
        return True

    def save_all(self):
        num_write = np.shape(self.saved_data)
        num_write = num_write[1]
        for i in range(num_write):
            self.write(self.saved_data[:, i])


# Global variables used
log_pos_x = 0
log_pos_y = 0
log_pos_z = 0
log_roll = 0
log_pitch = 0
log_yaw = 0
battery = 0

# # Period at which Vicon data will be sent to the Crazyflie
vicon2drone_period = 0.1  # [s]

# # Period used in Log configurations
datalog_period = 10  # ms
posvar_period = 500  # ms

# # Flag used by repeat_fun to
run = True
run_data = True


# Signal handling
# A PyCharm registry option has to be changed, according to
# https://youtrack.jetbrains.com/issue/PY-13316#focus=Comments-27-4240420.0-0,
# in order to catch the red 'Stop program' button press in the console
signal.signal(signal.SIGINT, handler_stop_signal)
signal.signal(signal.SIGTERM, handler_stop_signal)

# Instances of the MatlabPrint classes with relative logfile creation
vicon_matlab = MatlabPrint(flag=0)
int_matlab = MatlabPrint(flag=2)
safety_offset = 0.3
time_limit = 60  # [s]
tracking = True
pos_limit = 0.001  # [m]
max_equal_pos = 10
callback_mutex = threading.Semaphore(value=1)
