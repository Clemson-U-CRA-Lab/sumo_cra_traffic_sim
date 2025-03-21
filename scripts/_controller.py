# /usr/bin/env python3

import torch
import torch.nn as nn
import torch.nn.functional as F
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

class Model(nn.Module):
    def __init__(self, in_features=3, h1=256, h2=256, h3=32, out_features=1):
        super().__init__()
        self.fc1 = nn.Linear(in_features, h1)
        self.fc2 = nn.Linear(h1, h2)
        self.out = nn.Linear(h2, out_features)

    def forward(self, x):
        x = F.sigmoid(self.fc1(x))
        x = F.sigmoid(self.fc2(x))
        x = self.out(x)
        return x
        
class NN_controller():
    def __init__(self, nn_pt_file):
        self.nn_controller = Model(h1=128, h2=128)
        self.nn_controller.load_state_dict(torch.load(nn_pt_file))
        self.IDM_brake = IDM(a=4, b=6, s0=3, v0=20, T=1)
    
    def step_forward(self, s_vt, pv_vt, s_st, pv_st):
        ttc_i = TTCi_estimate(ego_v=s_vt, front_v=pv_vt, front_s=pv_st - s_vt)
        nn_input_vec = np.array([s_vt, pv_vt - s_vt, pv_st - s_st])
        nn_input = torch.FloatTensor(nn_input_vec.T)
        
        # Compute the neural network control
        with torch.no_grad():
            ego_a_nn = self.nn_controller.forward(nn_input)
            s_a_nn = ego_a_nn.flatten().tolist()
        
        # Compute intelligent driver model control
        s_a_IDM = self.IDM_brake.IDM_acceleration(front_v=pv_vt, ego_v=s_vt, front_s=pv_st, ego_s=s_st)
        
        # Check the if IDM braking is needed
        IDM_w = ttc_i > 0.5
        ego_a_tgt = (IDM_w) * s_a_IDM + (~IDM_w) * s_a_nn
        
        # Return control value
        return ego_a_tgt
    
class lookup_table_controller():
    def __init__(self, table_filename, max_s1, max_s2, max_dv, num_s1, num_s2, num_dv):
        self.s1_range = np.linspace(-1.0, max_s1, num_s1)
        self.s2_range = np.linspace(-1.0, max_s2, num_s2)
        self.dv_range = np.linspace(-1.0, max_dv, num_dv)
        
        with open(table_filename, 'rb') as f:
            u_table = np.load(f)
        
        self.fn_table = RegularGridInterpolator((self.s1_range, self.s2_range, self.dv_range), u_table)
    
    def step_forward(self, ds1, ds2, ego_v_t):
        input_vec = np.array([ds1, ds2, ego_v_t])
        s_a_table_tgt = (self.fn_table(input_vec.T).tolist())
        
        return s_a_table_tgt
        
    def pred_s(self, ego_s, veh_a, veh_v, veh_s, Dt = 5):
        if veh_v + veh_a * Dt < 0:
            t_to_stop = np.abs(veh_v / veh_a)
            ds1 = veh_s - ego_s
            ds2 = veh_v * Dt + 0.5 * veh_a * t_to_stop ** 2
        else:
            ds1 = veh_s - ego_s
            ds2 = veh_v * Dt + 0.5 * veh_a * Dt ** 2
        
        return ds1, ds2
    
    def preview_s(self, sim_t, ego_s, veh_init, veh_s, cycle_t, cycle_s, Dt = 5):
        t_id_terminal = np.argmin(np.abs(np.array(cycle_t) - (sim_t + Dt)))
        cycle_terminal = cycle_s[t_id_terminal]
        ds1 = veh_s - ego_s
        ds2 = cycle_terminal - veh_s + veh_init
        
        return ds1, ds2