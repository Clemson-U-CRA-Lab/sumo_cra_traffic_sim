import math
import csv
import os
import numpy as np
from _sensor import *
from _agents import PCC

class IDM():
    def __init__(self, a, b, s0, v0, T):
        self.a = a
        self.b = b
        self.s0 = s0
        self.v0 = v0
        self.T = T

    def IDM_acceleration(self, front_v, ego_v, front_s, ego_s):
        s_safe = self.s0 + ego_v * self.T + front_v * \
            (ego_v - front_v) / (2 * (self.a * self.b)**0.5)
        acc = self.a * (1 - (ego_v / self.v0) ** 4 -
                        (s_safe / (front_s - ego_s - 5)) ** 2)
        acc = np.clip(acc, -3, 3)
        return acc
    
class PCC_MPC_controller():
    def __init__(self):
        self.s = 0.0
        self.v = 0.0
        self.a = 0.0
        
        if os.name == 'nt':
            libraryname = 'pcc_so'
        elif os.name == 'posix':
            libraryname = 'libpcc_so'
        else:
            raise ValueError('PCC class cannot determine system type?!...')
    
        self.api = cpp_api(libraryname)