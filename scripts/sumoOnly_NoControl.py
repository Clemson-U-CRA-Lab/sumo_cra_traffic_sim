#! /usr/bin/env python3

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
import time

# import classes
from SumoSim import SumoSim
from x2v_constants import *

# logging utils
from  utils_logging import *
LOG_RUNNING = False

# Run params
vizTraj = True
guiSumo = False
live_plt = True


if __name__=="__main__":


    veh_0_dist = []
    veh_0_spd = []
    veh_0_lane = []
    veh_0_acc = []
    
    veh_1_dist = []
    veh_1_spd = []
    veh_1_lane = []
    veh_1_acc = []
    mache_accCmd = []
    
    veh_2_dist = []
    veh_2_spd = []
    # veh_1_lane = []
    veh_2_acc = []

    veh_sim_t = []

    front_ref_v = []
    
    runtime_record = []

    acc = {}
    preds_s ={}
    preds_v = {}

        # Data buffers for live plotting
    times = []
    dist0, dist1 = [], []
    spd0, spd1   = [], []
    dist2, spd2 = [], []
    
    # init speed profiles
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))


    # Init SUMO sim
    sumo_sim_manager = SumoSim(sumo_config_name=parent_dir + "/sumo/v2x/v2x_2veh.sumocfg")
    sumo_sim_manager.start_Sumo(gui=guiSumo)


    sim_start_time = 0  # SUMO's starting simulation time
    while sumo_sim_manager.step < 15/SIM_STEP:

        sim_time = traci.simulation.getTime()  # Get SUMO's current simulation time
        if sim_time % 10 == 0:
            print(f"sim time: {sim_time}")
        # Calculate expected real-time equivalent for SUMO's sim_time

        # Step SUMO forward
        sumo_sim_manager.simulationStepForward()


        # Get all vehicles currently in sim
        vehicle_list = traci.vehicle.getIDList()

        # Assign speeds to leading vehicle

        if sim_time < 3:
            pass
        else:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=0)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]


        
        # if local testing w/o gps:
        # sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=acc['nv1'], dt=SUMO_ACC_INTEGRATE_DT) # careful: assign commmand or real sensed acc?
        # time.sleep(0.05)
        if len(veh_states_matrix) < 2 and sim_time > 1:
            break

        # front_ref_v.append(v_tgt_lead)
        veh_0_acc.append(veh_states_matrix[0][1])
        veh_0_spd.append(veh_states_matrix[0][2])
        veh_0_dist.append(veh_states_matrix[0][3])
        
        veh_1_acc.append(veh_states_matrix[1][1])
        veh_1_spd.append(veh_states_matrix[1][2])
        veh_1_dist.append(veh_states_matrix[1][3])
        # mache_accCmd.append(acc['nv1'])
    
        veh_sim_t.append(sim_time)

    



    plt.figure(2)
    
    plt.subplot(3,1,1)
    # plt.plot(record_t, front_s_t, 'r:') 
    plt.plot(veh_sim_t, veh_0_dist,'k--')
    plt.plot(veh_sim_t, veh_1_dist,'b--')
    # plt.plot(veh_sim_t, veh_2_dist,'r--')
    # plt.plot(veh_sim_t, veh_3_dist)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['US06 Ref', 'Leading Vehicle',  'mache'])
    
    plt.subplot(3,1,2)
    # plt.plot(record_t, front_v_t, 'r:') 
    # plt.plot(veh_sim_t, front_ref_v, 'r--') 
    plt.plot(veh_sim_t, veh_0_spd, 'k--')
    plt.plot(veh_sim_t, veh_1_spd, 'b--')
    # plt.plot(veh_sim_t, veh_2_spd, 'r--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['US06 Ref', 'Leading Vehicle','mache'])

    plt.subplot(3,1,3)
    plt.plot(veh_sim_t, veh_0_acc, 'k--')
    plt.plot(veh_sim_t, veh_1_acc,'b--')
    # plt.plot(veh_sim_t, mache_accCmd, 'g--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Acc [m/s^2]')
    plt.legend(['Leading Vehicle', 'mache', 'mache_accCmd'])
    
    # plt.savefig('sumo_'+ datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.png')

    plt.show()
    

    traci.close(False)