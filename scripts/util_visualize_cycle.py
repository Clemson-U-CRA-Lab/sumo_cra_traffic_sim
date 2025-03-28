#! /usr/bin/env python3
'''
To visualize drive cycles.

Prakhar Gupta
'''

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _constants import *
import time

import struct
from x2v_constants import *

import struct
from x2v_constants import *

current_dirname = os.path.dirname(__file__)
parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"

leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)

record_t = np.array(leading_vehicle_speed_profile[:, 0])
front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
front_s_t = np.array(leading_vehicle_speed_profile[:, 3])


plt.figure(1)

plt.subplot(2,1,1)
plt.plot(record_t, front_v_t)
plt.xlabel('Time [s]')
plt.ylabel('Speed [m/s]')
plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])

plt.show()