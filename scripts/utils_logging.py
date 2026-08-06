#! /usr/bin/env python3

import os
import sys
import time
from datetime import datetime
import csv

import matplotlib.pyplot as plt

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


def plot_veh_data(veh_data, veh_sim_t, record_t=None, front_s_t=None, front_v_t=None,
                  file_prefix='sumoSim_log_', show_plot=True):
    """
    Plot distance, speed, and acceleration time series for all vehicles.
    Optionally overlay a front/reference profile (record_t, front_s_t, front_v_t).
    Saves a PNG next to the current working directory (typically the data folder).
    """
    fig = plt.figure(figsize=(6, 6))

    ax1 = fig.add_subplot(3, 1, 1)
    if record_t is not None and front_s_t is not None:
        ax1.plot(record_t, front_s_t, 'r:', label='front')
    for veh in veh_data:
        ax1.plot(veh_sim_t, veh_data[veh]['dist'], label=f'{veh} dist')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Distance [m]')
    ax1.minorticks_on()
    ax1.grid(True, which='major', linestyle='-', linewidth=0.3)
    ax1.grid(True, which='minor', linestyle=':', linewidth=0.2, alpha=0.6)
    ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.45), ncol=4, frameon=False)

    ax2 = fig.add_subplot(3, 1, 2)
    if record_t is not None and front_v_t is not None:
        ax2.plot(record_t, front_v_t, 'r:', label='front')
    for veh in veh_data:
        ax2.plot(veh_sim_t, veh_data[veh]['spd'], label=f'{veh} spd')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Speed [m/s]')
    ax2.minorticks_on()
    ax2.grid(True, which='major', linestyle='-', linewidth=0.3)
    ax2.grid(True, which='minor', linestyle=':', linewidth=0.2, alpha=0.6)
    ax2.legend(loc='upper center', bbox_to_anchor=(0.5, -0.45), ncol=4, frameon=False)

    ax3 = fig.add_subplot(3, 1, 3)
    for veh in veh_data:
        ax3.plot(veh_sim_t, veh_data[veh]['acc'], label=f'{veh} acc')
        ax3.plot(veh_sim_t, veh_data[veh]['accCmd'], '--', label=f'{veh} accCmd')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acc [m/s^2]')
    ax3.minorticks_on()
    ax3.grid(True, which='major', linestyle='-', linewidth=0.3)
    ax3.grid(True, which='minor', linestyle=':', linewidth=0.2, alpha=0.6)
    ax3.legend(loc='upper center', bbox_to_anchor=(0.5, -0.45), ncol=3, frameon=False)

    fig.tight_layout(pad=0.3, h_pad=-0.5, w_pad=0.2)
    fig.subplots_adjust(left=0.12, right=0.98)

    plt.savefig(file_prefix + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.png')
    if show_plot:
        plt.show()

    return fig
