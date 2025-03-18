#! /usr/bin/env python3

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
import time
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

if __name__=="__main__":
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
    
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    
    sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/CMI/cmi.sumocfg")
    sumo_sim_manager.start_Sumo()
    
    # Initialize controller
    dirname = os.path.dirname(__file__)
    nn_pt_filename = dirname + '/traffic_following_control_v0_128.pt'
    FCN_control = NN_controller(nn_pt_file=nn_pt_filename)
    
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    
    while sumo_sim_manager.step < 950:
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        
        # Get leading vehicle speed
        v_lead_id = np.argmin(np.abs([record_t - sim_t]))
        v_tgt_lead = front_v_t[v_lead_id]
        sumo_sim_manager.assignTargetSpeed(vehicle_ID="veh0", tgt_spd=v_tgt_lead)
        
        [veh_0_acc_t, veh_0_spd_t, veh_0_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh0")
        [veh_1_acc_t, veh_1_spd_t, veh_1_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh1")
        [veh_2_acc_t, veh_2_spd_t, veh_2_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh2")
        [veh_3_acc_t, veh_3_spd_t, veh_3_dist_t, _] = sumo_sim_manager.getVehicleStates(vehicle_ID="veh3")
        
        # Get back vehicle speed and distance
        s_vt_traffic = np.array([veh_1_spd_t, veh_2_spd_t, veh_3_spd_t])
        pv_vt_traffic = np.array([veh_0_spd_t, veh_1_spd_t, veh_2_spd_t])
        s_st_traffic = np.array([veh_1_dist_t, veh_2_dist_t, veh_3_dist_t])
        pv_st_traffic = np.array([veh_0_dist_t, veh_1_dist_t, veh_2_dist_t])
        
        # Perform neural network control
        start_t = time.time()
        acc_traffic_step_t = FCN_control.step_forward(s_vt=s_vt_traffic, pv_vt=pv_vt_traffic, s_st=s_st_traffic, pv_st=pv_st_traffic)
        runtime_record.append(time.time() - start_t)
        
        acc_1 = acc_traffic_step_t[0]
        acc_2 = acc_traffic_step_t[1]
        acc_3 = acc_traffic_step_t[2]
        
        # Assign the acceleration to ego vehicle
        sumo_sim_manager.assignAcceleration(vehicle_ID="veh1", tgt_acc=acc_1, dt=0.1)
        sumo_sim_manager.assignAcceleration(vehicle_ID="veh2", tgt_acc=acc_2, dt=0.1)
        sumo_sim_manager.assignAcceleration(vehicle_ID="veh3", tgt_acc=acc_3, dt=0.1)
        
        veh_0_acc.append(veh_0_acc_t)
        veh_0_spd.append(veh_0_spd_t)
        veh_0_dist.append(veh_0_dist_t)
        
        veh_1_acc.append(veh_1_acc_t)
        veh_1_spd.append(veh_1_spd_t)
        veh_1_dist.append(veh_1_dist_t)
        
        veh_2_acc.append(veh_2_acc_t)
        veh_2_spd.append(veh_2_spd_t)
        veh_2_dist.append(veh_2_dist_t)
        
        veh_3_acc.append(veh_3_acc_t)
        veh_3_spd.append(veh_3_spd_t)
        veh_3_dist.append(veh_3_dist_t)
        
        veh_sim_t.append(sim_t)
        
        time.sleep(0.01)
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    
    plt.figure(1)
    
    plt.subplot(2,1,1)
    plt.plot(veh_sim_t, veh_0_dist)
    plt.plot(veh_sim_t, veh_1_dist)
    plt.plot(veh_sim_t, veh_2_dist)
    plt.plot(veh_sim_t, veh_3_dist)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])
    
    plt.subplot(2,1,2)
    plt.plot(veh_sim_t, veh_0_spd)
    plt.plot(veh_sim_t, veh_1_spd)
    plt.plot(veh_sim_t, veh_2_spd)
    plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['Leading Vehicle', 'Vehicle 0', 'Vehicle 1', 'Vehicle 2'])
    
    plt.show()
    
    traci.close(False)