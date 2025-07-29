#! /usr/bin/env python3

import os
import sys
import time
from datetime import datetime
import csv

csv_header = ["Realtime [sec]","Sim Time [sec]", "MPC runtime",\
                                "v0_dist [m]","v0_lane [-]","v0_spd [m/s]","v1_acc [m/s2]",\
                                "v1_dist [m]","v1_lane [-]","v1_spd [m/s]","v1_acc [m/s2]","MachE_accCmd [m/s2]"]

def save_csv_sumo(data, file_prefix='sumo_v2x_log', csv_header=None):

    file = file_prefix + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'
    
    if csv_header is None:
        csv_header = ["Sim Time [sec]", \
                            "MPC runtime",\
                                "v0_dist [m]",\
                                "v0_lane [-]",\
                                "v0_spd [m/s]",\
                                "v1_acc [m/s2]", \
                                "v1_dist [m]",\
                                "v1_lane [-]",\
                                "v1_spd [m/s]",\
                                "v1_acc [m/s2]"]
    
    data_folder = datetime.now().strftime("%Y_%m_%d")
    path = "/home/cra/sumo_ws/sumo_cra_traffic_sim/data/" + data_folder
    if os.path.exists(path):
            os.chdir(path)
    else:
        os.chdir("/home/cra/sumo_ws/sumo_cra_traffic_sim/data/")
        os.mkdir(data_folder)
        os.chdir(path)

    # start a CSV file       
    with open(file, "a", newline="") as csv_file:
        # Create a CSV writer object
        csv_writer = csv.writer(csv_file)

        # If the file is empty, write the header row
        if csv_file.tell() == 0:
            csv_writer.writerow(csv_header)

        csv_writer.writerows(data)