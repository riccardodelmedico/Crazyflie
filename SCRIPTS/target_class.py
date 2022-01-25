import threading
import time
import numpy as np

run = True


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


def update_virtual_target(obj_target, dt=0.05):
    # integrate velocity and acceleration with forward euler
    obj_target.mutex.acquire(True)
    obj_target.time_line = np.append(obj_target.time_line, np.array(time.time()))
    if len(obj_target.time_line) < 2:
        deltat = dt
    else:
        deltat = obj_target.time_line[-1] - obj_target.time_line[-2]
    if np.linalg.norm(obj_target.v, 2) != 0 and run is True:
        obj_target.p += obj_target.v * deltat + \
                        obj_target.omega_vers_hat.dot(obj_target.v/np.linalg.norm(obj_target.v, 2))*obj_target.a*deltat*deltat/2
        obj_target.v += obj_target.omega_vers_hat.dot(obj_target.v/np.linalg.norm(obj_target.v, 2))*obj_target.a*deltat

    obj_target.list_pos = np.concatenate((obj_target.list_pos, obj_target.p.reshape((1, 3))), axis=0)
    obj_target.mutex.release()
    if not run:
        print('Extra runs not needed')


class Target:
    def __init__(self, initial_position=np.array([0.0, 0.0, 0.5]),
                 initial_velocity=np.array([0.0, 0.0, 0.0]),
                 initial_acceleration_module=0,
                 dt=0.05):
        self.p = initial_position
        self.v = initial_velocity
        self.a = initial_acceleration_module
        self.list_pos = initial_position.reshape((1, 3))
        self.time_line = np.array([0.0])
        self.omega_vers_hat = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 0]])
        self.update_thread = threading.Thread(target=repeat_fun, args=(dt, update_virtual_target, self, dt))
        self.mutex = threading.Semaphore(1)

    def get_target(self):
        self.mutex.acquire(True)
        if len(np.argwhere(self.v == 0)) < 3:
            acc = self.omega_vers_hat.dot(self.v/np.linalg.norm(self.v, 2))*self.a
        else:
            acc = np.zeros(3)
        if self.time_line[0] == 0.0:
            ret = (self.p, self.v, acc, 0.0)
        else:
            ret = (self.p, self.v, acc, self.time_line[-1])
        self.mutex.release()
        return ret

    def start(self):
        global run
        self.time_line[0] = time.time()
        print('Starting Thread')
        run = True
        self.update_thread.start()

    def stop(self):
        global run
        print('Stopping Thread')
        run = False
        self.update_thread.join()
