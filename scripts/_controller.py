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
        s_safe = self.s0 + ego_v * self.T + front_v * (ego_v - front_v) / (2 * (self.a * self.b)**0.5)
        s_safe[s_safe < self.s0 + 3] = self.s0 + 3
        acc = self.a * (1 - (ego_v / self.v0) ** 4 -
                        (s_safe / (front_s - ego_s - self.s0)) ** 2)
        acc = np.clip(acc, -4, 4)
        return acc
    
class PCC_MPC_controller():
    def __init__(self, dirname):
        self.s = 0.0
        self.v = 0.0
        self.a = 0.0
        self.svs = PCC(dirname, self.s, self.v, self.a, v_max=35)

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

class Model_3_input(nn.Module):
    def __init__(self, in_features=4, h1=256, h2=256, h3=32, out_features=1):
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
    def __init__(self, nn_pt_file, input_num):
        self.num_input = input_num
        if input_num == 3:
            self.nn_controller = Model(h1=256, h2=256)
        if input_num == 4:
            self.nn_controller = Model_3_input(h1=256, h2=256)
        self.nn_controller.eval()
        self.nn_controller.load_state_dict(torch.load(nn_pt_file, map_location='cpu'))
        self.nn_controller.to('cuda')
    
    def CBF_acceleration_bound_check(self, pv_vt, s_vt, pv_st, s_st, tao, alpha, L):
        a_ego_max = (pv_vt - s_vt + alpha * (pv_st - s_st - L - tao * s_vt)) / tao
        return a_ego_max
    
    def step_forward(self, s_vt, pv_vt, s_st, pv_st, s_at, pv_at, use_prediction_horizon, sim_t):
        # Calculate the prediction horizon length
        pv_s_end = np.zeros(pv_st.shape)
        pv_v_end = np.zeros(pv_vt.shape)
        a_input = np.array(pv_at)
        a = np.tile(a_input, (49, 1))
        v = pv_vt + np.cumsum(a * 0.5, axis=0)
        v = np.clip(v, 0, np.Inf)
        s = pv_st + np.cumsum(v * 0.5, axis=0)
        pv_s_end = np.clip(s[-1, :] - s_st, -10, 1500)
        pv_v_end = v[-1, :] - s_vt
        
        if self.num_input == 3:
            if use_prediction_horizon:
                nn_input_vec = np.array([s_vt, pv_v_end, pv_s_end])
            else:
                nn_input_vec = np.array([s_vt, pv_vt - s_vt, pv_st - s_st])
        if self.num_input == 4:
            nn_input_vec = np.array([s_vt, pv_vt - s_vt, pv_s_end, pv_v_end])
        nn_input = torch.FloatTensor(nn_input_vec.T).cuda()
        # Compute the neural network control
        with torch.no_grad():
            ego_a_nn = self.nn_controller.forward(nn_input)
            s_a_nn = (ego_a_nn.flatten()).tolist()
        
        # Check if CBF safety constraint is violated
        a_ego_max = self.CBF_acceleration_bound_check(pv_vt=pv_vt, s_vt=s_vt, pv_st=pv_st, s_st=s_st, tao=1.5, alpha=2.0, L=7.0)
        if np.any(s_a_nn > a_ego_max):
            print(f"CBF safety constraint is violated at time {sim_t}! Adjusting NN control to ensure safety...")
            ego_a_tgt = np.minimum(s_a_nn, a_ego_max)
        else:
            ego_a_tgt = s_a_nn

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
        
    def pred_s(self, ego_s, veh_a, veh_v, veh_s, Dt = 3):
        if veh_v + veh_a * Dt < 0:
            t_to_stop = np.abs(veh_v / veh_a)
            ds1 = veh_s - ego_s
            ds2 = veh_v * Dt + 0.5 * veh_a * t_to_stop ** 2
        else:
            ds1 = veh_s - ego_s
            ds2 = veh_v * Dt + 0.5 * veh_a * Dt ** 2
        
        return ds1, ds2
    
    def preview_s(self, sim_t, ego_s, veh_init, veh_s, cycle_t, cycle_s, Dt = 3):
        t_id_terminal = np.argmin(np.abs(np.array(cycle_t) - (sim_t + Dt)))
        cycle_terminal = cycle_s[t_id_terminal]
        ds1 = veh_s - ego_s
        ds2 = cycle_terminal - veh_s + veh_init
        
        return ds1, ds2