# /usr/bin/env python3

from scipy.interpolate import RegularGridInterpolator

import math
import csv
import os
import numpy as np
from utils import *
from _sensor import *
from _agents import *

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
    def __init__(self, dirname):
        self.s = 0.0
        self.v = 0.0
        self.a = 0.0
        self.svs = PCC(dirname, self.s, self.v, self.a, v_max=20)
        
    def step_forward(self, PV_ds, PV_v, PV_a, future_s, future_v, t):
        # Find control from MPC optimization and control the vehicle
        pred_pose = self.svs.setCommand_SUMO(t=t, pv_ds=PV_ds, pv_v=PV_v, pv_a=PV_a, pv_ind=0, cycle_ss=future_s, cycle_vs=future_v)

