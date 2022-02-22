from Data_Core import get_data
import numpy as np

class Seeker:
    def __init__(self,list_data):
        self.list_data = list_data
    def get_measures(self):
        return get_data(self.list_data)
