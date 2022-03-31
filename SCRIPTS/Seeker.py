from DataCore import get_data
import numpy

class Seeker:
    """
    Seeker manages the access to a subsection of guidance_variable to simulate
    the presence of a seeker on drone.

    Constructor:
    :param data_list: list of data to be fetched
    :type data_list: string np.array[...]
    """

    def __init__(self, data_list):
        self.data_list = data_list

    def get_measures(self):
        return get_data(self.data_list)
    def get_drone_vel(self):
        vel_drone = ['d_est_vel_x','d_est_vel_y']
        [vx,vy,_] = get_data(vel_drone)
        drone_v = numpy.linalg.norm(numpy.array([vx,vy]), 2)
        return drone_v

