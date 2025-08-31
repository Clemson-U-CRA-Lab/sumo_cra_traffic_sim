#! /usr/bin/env python3

import os
import sys
import time
from datetime import datetime
import csv

csv_header = ["Realtime [sec]","Sim Time [sec]", "Sim Time from MPC [sec]", "MPC runtime",\
                "v0_dist [m]","v0_lane [-]","v0_spd [m/s]","v0_acc [m/s2]",\
                "v1_dist [m]","v1_lane [-]","v1_spd [m/s]","v1_acc [m/s2]","v1_accCmd [m/s2]"]

def save_csv_sumo(data, file_prefix='sumo_v2x_log', csv_header=None):

    file = file_prefix + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'
    
    if csv_header is None:
        csv_header = ["Realtime [sec]",\
                            "Sim Time [sec]", \
                            "Sim Time from MPC [sec]",\
                            "MPC runtime",\
                            "v0_dist [m]",\
                            "v0_lane [-]",\
                            "v0_spd [m/s]",\
                            "v1_acc [m/s2]", \
                            "v1_dist [m]",\
                            "v1_lane [-]",\
                            "v1_spd [m/s]",\
                            "v1_accCmd [m/s2]"]
    
    data_folder = datetime.now().strftime("%Y_%m_%d")
    path0 = os.path.dirname(os.path.abspath(__file__)) + "/../" + "data/"
    path = path0 + data_folder
    if os.path.exists(path):
            os.chdir(path)
    else:
        os.chdir(path0)
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



def save_all_veh_data_csv(veh_data, veh_sim_t, real_time_list = None, file_prefix='sumo_log'):
    """
    Save all vehicle data in one CSV file.
    veh_data: dict of vehicle data, e.g. {veh: {'dist': [...], 'spd': [...], 'acc': [...], 'accCmd': [...]}}
    veh_sim_t: list of simulation times
    filename: output CSV filename
    """
    file = file_prefix + "_allVeh_" + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'

    data_folder = datetime.now().strftime("%Y_%m_%d")
    path0 = os.path.dirname(os.path.abspath(__file__)) + "/../" + "data/"
    path = path0 + data_folder
    if os.path.exists(path):
            os.chdir(path)
    else:
        os.chdir(path0)
        os.mkdir(data_folder)
        os.chdir(path)


    if real_time_list is not None:
        csv_header = ["SimTime", "RealTime"]
    else:
        csv_header = ["Time"]
    for veh in veh_data:
        csv_header += [f"{veh}_Dist", f"{veh}_Spd", f"{veh}_Acc", f"{veh}_AccCmd"]
    rows = []
    for i, t in enumerate(veh_sim_t):
        row = [t] if real_time_list is None else [t, real_time_list[i]]
        for veh in veh_data:
            row.append(veh_data[veh]['dist'][i] if i < len(veh_data[veh]['dist']) else None)
            row.append(veh_data[veh]['spd'][i] if i < len(veh_data[veh]['spd']) else None)
            row.append(veh_data[veh]['acc'][i] if i < len(veh_data[veh]['acc']) else None)
            row.append(veh_data[veh]['accCmd'][i] if i < len(veh_data[veh]['accCmd']) else None)
        rows.append(row)
    with open(file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(csv_header)
        writer.writerows(rows)