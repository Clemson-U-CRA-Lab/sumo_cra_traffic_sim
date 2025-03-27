#! /usr/bin/env python3

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

# import classes
from SumoSim import SumoSim
from x2vSocketInterface import x2vSocketInterfaceAsync as x2vSocketInterface
from x2vSocketInterface import x2vSocketInterfaceAsync as x2vSocketInterface

SIM_STEP = 0.1
StalledNv = 'nv1' # the car that stalls
RealCav = "nv2" # mache

if __name__=="__main__":

        # Init socket connections
    sockInt = x2vSocketInterface()
    sockInt = x2vSocketInterface()

    veh_0_dist = []
    veh_0_spd = []
    veh_0_lane = []
    veh_0_acc = []
    
    veh_1_dist = []
    veh_1_spd = []
    veh_1_lane = []
    veh_1_acc = []
    
    veh_2_dist = []
    veh_2_spd = []
    veh_2_lane = []
    veh_2_acc = []
    
    veh_3_dist = []
    veh_3_spd = []
    veh_3_lane = []
    veh_3_acc = []
    
    veh_sim_t = []
    
    runtime_record = []
    
    # init speed profiles
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3])

    # Init SUMO sim
    sumo_sim_manager = SumoSim(sumo_config_name=parent_dir + "/sumo/v2x/v2x.sumocfg")
    sumo_sim_manager.start_Sumo(gui=True)
    traci.vehicle.setSpeed("nv2", 0.0)

    
    # Inti and Setup controller
    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')
        
    # Start simulation 
    total_elapsed = 0
    # Get the real-world start time
    real_start_time = time.monotonic()  
    sim_start_time = 0  # SUMO's starting simulation time
    while sumo_sim_manager.step < 300:
        sim_time = traci.simulation.getTime()  # Get SUMO's current simulation time
        # Calculate expected real-time equivalent for SUMO's sim_time
        real_expected_time = real_start_time + (sim_time - sim_start_time)

        # Step SUMO forward
        sumo_sim_manager.simulationStepForward()

        # Get all vehicles currently in sim
        vehicle_list = traci.vehicle.getIDList()

        # Assign speeds to leading vehicle
        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]
        sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=v_tgt_lead)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]
        # print(veh_states_matrix)

        start_t = time.time()
        # Run MPC control if enabled
        if USING_ONLINE_MPC:
            acc, preds_s, preds_v = traffic_online_MPC_control_step_nVeh(veh_states_matrix, 
                                                       sim_t=sim_time, 
                                                       record_t=record_t,
                                                       front_v_t=front_v_t,
                                                       online_MPC_control=online_MPC_control)
        else:
            acc = {}
            for veh in vehicle_list:
                acc[veh] = 0.0
        runtime_record.append(time.time() - start_t)


        # Send NV states to realCAV
        # sim_time, ego_s, ego_v, ego_a  front_s, front_v, front_a, 
        lead_nv_array = [sim_time, 
                         veh_states_matrix[2][3], veh_states_matrix[2][2], veh_states_matrix[2][1], # ego
                         veh_states_matrix[1][3], veh_states_matrix[1][2], veh_states_matrix[1][1]  # front
                         ] + preds_v['nv1'] + preds_s['nv1'] # front's
        # print(len(lead_nv_array))
        sockInt.send_sim_info(lead_nv_array)
        print(lead_nv_array[0:6])

        # Recv realCAV info
        realCavArray = sockInt.recv_veh_info()
        if realCavArray is not None:        
            print("RealCAV value: ", realCavArray[0:3])

            # Update Real CAV pos in simulation:::
            # sumo_sim_manager.assignAcceleration(vehicle_ID="nv2", tgt_acc=realCavArray[3], dt=0.1) # careful: assign commmand or real sensed acc?
            # traci.vehicle.moveToXY(vehID="nv2", edgeID="76146229#1", laneIndex="0", x=mache_pos[0], y=mache_pos[1])
            traci.vehicle.setSpeed("nv2", realCavArray[2])

        # Assign the acceleration to follower vehicle
        sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=acc["nv1"], dt=0.1)
        
        # Log
        veh_0_acc.append(veh_states_matrix[0][1])
        veh_0_spd.append(veh_states_matrix[0][2])
        veh_0_dist.append(veh_states_matrix[0][3])
        
        veh_1_acc.append(veh_states_matrix[1][1])
        veh_1_spd.append(veh_states_matrix[1][2])
        veh_1_dist.append(veh_states_matrix[1][3])

        veh_2_acc.append(veh_states_matrix[2][1])
        veh_2_spd.append(veh_states_matrix[2][2])
        veh_2_dist.append(veh_states_matrix[2][3])
        
        veh_sim_t.append(sim_time)

        # Sleep timing
        real_now = time.monotonic()
        sleep_time = max(0, real_expected_time - real_now)  # Sleep only if ahead of real time
        time.sleep(sleep_time)  # Sync with real-world time
        print(f"Real elapsed: {real_now - real_start_time:.3f}s, Sim Time: {sim_time:.3f}s")
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    
    plt.figure(1)
    
    plt.subplot(2,1,1)
    plt.plot(veh_sim_t, veh_0_dist)
    plt.plot(veh_sim_t, veh_1_dist)
    plt.plot(veh_sim_t, veh_2_dist)
    # plt.plot(veh_sim_t, veh_3_dist)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])
    
    plt.subplot(2,1,2)
    plt.plot(veh_sim_t, veh_0_spd)
    plt.plot(veh_sim_t, veh_1_spd)
    plt.plot(veh_sim_t, veh_2_spd)
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])
    
    plt.show()
    
    traci.close(False)