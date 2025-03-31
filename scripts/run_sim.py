#! /usr/bin/env python3

import os
import sys
import argparse
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _constants import *
import time
import random
class sumo_sim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo-gui"
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = len(self.vehID_list)
        self.step = 0
        
    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
    
    def getVehicleStates(self, vehicle_ID):
        if vehicle_ID  in self.vehID_list:
            veh_spd_t = traci.vehicle.getSpeed(vehID=vehicle_ID)
            veh_dist_t = traci.vehicle.getLanePosition(vehID=vehicle_ID)
            veh_lane_t = traci.vehicle.getLaneID(vehID=vehicle_ID)
            veh_acc_t = traci.vehicle.getAcceleration(vehID=vehicle_ID)
            return [veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t]
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            return [0.0, 0.0, 0.0, 0.0]
    
    def assignAcceleration(self, vehicle_ID, tgt_acc, dt):
        if vehicle_ID in self.vehID_list:
            traci.vehicle.setAcceleration(vehID=vehicle_ID, acceleration=tgt_acc, duration=dt)
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            
    def assignTargetSpeed(self, vehicle_ID, tgt_spd):
        if vehicle_ID in self.vehID_list:
            traci.vehicle.setSpeed(vehID=vehicle_ID, speed=tgt_spd)
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
    
    def simulationStepForward(self):
        traci.simulationStep()
        self.vehID_list = traci.vehicle.getIDList()
        self.step += 1
    
    def assignTargetLane(self, vehicle_ID, edge_ID, pos):
        traci.vehicle.moveTo(vehID=vehicle_ID, laneID=edge_ID)
        
    def assignLaneChangeMode(self, veh_id, mode):
        traci.vehicle.setLaneChangeMode(vehID=veh_id, laneChangeMode=mode)


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logging_sim", help="whether to save the simulation data", action="store_true")
    args = parser.parse_args()
    
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
    
    P_1_list = []
    P_2_list = []
    P_3_list = []
    
    veh_sim_t = []
    
    runtime_record = []
    
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/I85_nycccol.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    
    # sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/I-85_highway/I-85.sumocfg")
    sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/I-85_highway/I-85.sumocfg")
    sumo_sim_manager.start_Sumo()
    
    # Initialize controller
    dirname = os.path.dirname(__file__)
    nn_pt_filename = dirname + '/traffic_following_control_v3_256_best.pt'
    table_filename = dirname + '/Utable_2states_MPC_terminal.npy'
    
    # Setup controller
    if USING_NEURAL_NETWORK:
        FCN_control = NN_controller(nn_pt_file=nn_pt_filename)
        controller_name = 'Neural_Network'
        print('Use neural network to control traffic vehicles')
    elif USING_LOOKUP_TABLE:
        LKTable_control = lookup_table_controller(table_filename=table_filename, max_s1=150, max_s2=150,
                                              max_dv=30, num_s1=20, num_s2=20, num_dv=15)
        controller_name = 'Lookup_Table'
        print('Use lookup table to control traffic vehicles')
    elif USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=dirname)
        controller_name = 'Online_MPC'
        print('Use online MPC to control traffic vehicles')
    elif USING_IDM:
        IDM_control = IDM(a=4, b=5, s0=3, v0=20, T=1)
        controller_name = 'Intelligent Driving Model'
        print('Use IDM to control traffic vehicles')
    else:
        print('No controller for all vehicles')
        
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3])
    
    traci.gui.trackVehicle("View #0", "veh1")
    traci.gui.setZoom("View #0", 10000)
    while sumo_sim_manager.step < len(record_t):
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        
        # Get leading vehicle speed
        v_lead_id = np.argmin(np.abs([record_t - sim_t]))
        v_tgt_lead = front_v_t[v_lead_id] #+ 2.0 * (random.random() - 0.5)
        sumo_sim_manager.assignTargetSpeed(vehicle_ID="veh0", tgt_spd=v_tgt_lead)
        sumo_sim_manager.assignLaneChangeMode(veh_id="veh0", mode=0)
        
        [veh_0_acc_t, veh_0_spd_t, veh_0_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh0")
        [veh_1_acc_t, veh_1_spd_t, veh_1_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh1")
        
        # Estimate and record power consumption
        P_1_list.append(engine_power_estimation(veh_1_spd_t, veh_1_acc_t))
        
        # Perform neural network control
        start_t = time.time()
        
        if USING_ONLINE_MPC:
            acc_traffic_step_t = traffic_online_MPC_control_step(veh_0_acc_t, veh_0_spd_t, veh_0_dist_t,
                                                                 veh_1_acc_t, veh_1_spd_t, veh_1_dist_t,
                                                                 sim_t, record_t, front_v_t, online_MPC_control)
        elif USING_LOOKUP_TABLE:
            # Find reference states for lookup table
            ds1_0, ds2_0 = LKTable_control.preview_s(sim_t, veh_1_dist_t, veh_init=30, veh_s=veh_0_dist_t, cycle_t=record_t, cycle_s=front_s_t)
            acc_traffic_step_t = LKTable_control.step_forward([ds1_0], [ds2_0], [veh_1_spd_t])
        elif USING_NEURAL_NETWORK:
            # Get back vehicle speed and distance
            s_vt_traffic = np.array([veh_1_spd_t])
            pv_vt_traffic = np.array([veh_0_spd_t])
            s_st_traffic = np.array([veh_1_dist_t])
            pv_st_traffic = np.array([veh_0_dist_t])
            acc_traffic_step_t = FCN_control.step_forward(s_vt=s_vt_traffic, pv_vt=pv_vt_traffic, s_st=s_st_traffic, pv_st=pv_st_traffic)
        elif USING_IDM:
            acc_traffic_step_t = IDM_control.IDM_acceleration(front_v=np.array([veh_0_spd_t]),
                                                              ego_v=np.array([veh_1_spd_t]),
                                                              front_s=np.array([veh_0_dist_t]),
                                                              ego_s=np.array([veh_1_dist_t]))
        else:
            acc_traffic_step_t = np.zeros(3)
            
        runtime_record.append(time.time() - start_t)
        
        acc_1 = acc_traffic_step_t[0]
        
        # Assign the acceleration to ego vehicle
        sumo_sim_manager.assignAcceleration(vehicle_ID="veh1", tgt_acc=acc_1, dt=0.1)
        sumo_sim_manager.assignLaneChangeMode(veh_id="veh1", mode=0)
        
        if args.logging_sim:
            data_logger(sim_t=sim_t, ego_a=acc_1, ego_v=veh_1_spd_t, ego_s=veh_1_dist_t,
                        pv_a=veh_0_acc_t, pv_v=veh_0_spd_t, pv_s=veh_0_dist_t, filename="EPA_SUMO_nyc.csv")
        
        veh_0_acc.append(veh_0_acc_t)
        veh_0_spd.append(veh_0_spd_t)
        veh_0_dist.append(veh_0_dist_t)
        
        veh_1_acc.append(veh_1_acc_t)
        veh_1_spd.append(veh_1_spd_t)
        veh_1_dist.append(veh_1_dist_t)
        
        veh_sim_t.append(sim_t)
        
        time.sleep(0.01)
    
    traci.close(False)
    
    veh_sim_t = np.array(veh_sim_t)
    
    # Compute total power consumption
    E_1 = np.sum(np.array(P_1_list) * 0.1)
    E = E_1
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    print('Energy consumption for this traffic section is: ', str(round(E / 1000, 3)) + 'kW')
    
    plt.figure(1)
    
    # plt.subplot(2,1,1)
    # plt.plot(veh_sim_t, veh_0_dist)
    # plt.plot(veh_sim_t, veh_1_dist)
    # plt.plot(veh_sim_t, veh_2_dist)
    # plt.plot(veh_sim_t, veh_3_dist)
    # plt.xlabel('Time [s]')
    # plt.ylabel('Distance from route edge [m]')
    # plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])
    
    plt.subplot(2,1,1)
    plt.title(controller_name)
    plt.plot(veh_sim_t, veh_0_spd)
    plt.plot(veh_sim_t, veh_1_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['Leading Vehicle', 'Vehicle 0'])
    
    plt.subplot(2,1,2)
    plt.plot(veh_sim_t, veh_0_acc)
    plt.plot(veh_sim_t, veh_1_acc)
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s^2]')
    plt.legend(['Leading Vehicle', 'Vehicle 0'])
    
    plt.show()