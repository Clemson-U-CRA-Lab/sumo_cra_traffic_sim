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
from _chicago import *
import time
import random

class sumo_sim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo-gui"
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = 0
        self.step = 0
        self.sumo_veh = []
        
        # Init parameters for SUMO traffic
        self.init_t = []
        self.init_state = []
        self.init_s = []
        self.init_v = []
        self.init_a = []
        self.lane_id = []
        self.entering_type = []

    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
    
    def init_scenario(self, INIT_STAT):
        self.num_veh = len(INIT_STAT)
        for i in range(self.num_veh):
            self.init_t.append(INIT_STAT[i][4])
            self.init_state.append(False)
            self.init_s.append(INIT_STAT[i][0])
            self.init_v.append(INIT_STAT[i][1])
            self.init_a.append(INIT_STAT[i][2])
            self.lane_id.append(INIT_STAT[i][3])
        self.sumo_veh = [None] * self.num_veh
        self.pv_id = [None] * self.num_veh
    
    def init_status_update(self, veh_id):
        if self.step * 0.1 > self.init_t[veh_id]:
            if self.init_state[veh_id] == False:
                self.sumo_veh[veh_id] = SUMO_vehicles(vehicle_ID="veh" + str(veh_id), init_s = self.init_s[veh_id], init_lane=self.lane_id[veh_id] - 1, route_ID="route1", lane_change_mode=0)
                self.init_state[veh_id] = True
        else:
            self.init_state[veh_id] = False
    
    def update_traffic_vehicle_state(self, veh_id, tgt_acc):
        self.sumo_veh[veh_id].assignTargetAcceleration(tgt_acc)
    
    def simulationStepForward(self):
        traci.simulationStep()
        self.vehID_list = traci.vehicle.getIDList()
        self.step += 1
        
    def update_preceding_vehicle(self, veh_id):
        # Find existed vehicles in the traffic
        if len(self.vehID_list) > 0:
            ego_veh_id = "veh" + str(veh_id)
            # Check if vehicle of interest existed in the traffic
            _ego_traffic_status = len(np.where(np.array(self.vehID_list) == ego_veh_id)[0])
            if _ego_traffic_status:
                # Check the lane id of ego vehicles
                ego_lane_id = int(traci.vehicle.getLaneID(vehID=ego_veh_id)[-1])
                traffic_veh_lane_id = [int(traci.vehicle.getLaneID(vehID=id)[-1]) for id in self.vehID_list]
                [_, _, ego_veh_s_t] = self.sumo_veh[veh_id].getVehicleStates()
                traffic_veh_s_t = np.array([self.sumo_veh[int(id[3:])].getVehicleStates()[2] for id in self.vehID_list])
                traffic_front_s_id = np.where(np.array(traffic_veh_lane_id) == ego_lane_id)[0].tolist()
                traffic_front_s_t = traffic_veh_s_t[traffic_front_s_id]
                
                traffic_front_s_id = np.where(traffic_front_s_t > ego_veh_s_t)[0]
                if len(traffic_front_s_id) > 0:
                    traffic_pv_s = traffic_front_s_t[traffic_front_s_id]
                    traffic_pv_s_min = np.min(traffic_pv_s)
                    traffic_pv_id = np.where(traffic_veh_s_t == traffic_pv_s_min)[0]
                    if len(traffic_pv_id) > 0:
                        self.sumo_veh[veh_id].pv_ID = self.vehID_list[traffic_pv_id[0]]
            
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logging_sim", help="whether to save the simulation data", action="store_true")
    parser.add_argument("control_type", choices=['MPC', 'NN', 'IDM'], help='Choose control method for traffic vehicles')
    parser.add_argument("scenario_id", type=int, choices=range(101, 121))
    args = parser.parse_args()
    
    # Initalize SUMO
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/I-85_highway/I-85.sumocfg")
    sumo_sim_manager.start_Sumo()
    
    # Traffic control setting
    if args.control_type == 'MPC':
        USING_ONLINE_MPC = 1 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 0 # If using neural network controller to track front vehicle
        USING_IDM = 0 # If using IDM to traffic front vehicle
    elif args.control_type == 'NN':
        USING_ONLINE_MPC = 0 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 1 # If using neural network controller to track front vehicle
        USING_IDM = 0 # If using IDM to traffic front vehicle
    else:
        USING_ONLINE_MPC = 0 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 0 # If using neural network controller to track front vehicle
        USING_IDM = 1 # If using IDM to traffic front vehicle
        
    # Setup controller
    if USING_NEURAL_NETWORK:
        nn_pt_filename = current_dirname + '/traffic_following_control.pt'
        FCN_control = NN_controller(nn_pt_file=nn_pt_filename, input_num=3)
        controller_name = 'Neural_Network'
        print('Use neural network to control traffic vehicles')
    elif USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
        controller_name = 'Online_MPC'
        print('Use online MPC to control traffic vehicles')
    elif USING_IDM:
        IDM_control = IDM(a=3, b=3, s0=7, v0=20, T=2)
        controller_name = 'Intelligent Driving Model'
        print('Use IDM to control traffic vehicles')
    else:
        print('No controller for all vehicles')
    
    
    # Initialize Traffic
    CHI_init_state = getChicagoTraffic(args.scenario_id)
    sumo_sim_manager.init_scenario(CHI_init_state)
    runtime_record = []
    
    while True:
        loop_start_t = time.time()
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        
        # Check if the sim terminate
        if len(sumo_sim_manager.vehID_list) == 0 and sim_t > 20:
            print('Simulation Terminated')
            break
        
        # Update all traffic vehicles' status
        for i in range(sumo_sim_manager.num_veh):
            sumo_sim_manager.init_status_update(i)
        
        # Initialize each vehicle states vehicles
        veh_ctrl_input = np.zeros((6, len(sumo_sim_manager.vehID_list)))
        
        # Update traffic vehicles inside the traffic
        for k in range(len(sumo_sim_manager.vehID_list)):
            # Get ego vehicle states
            ego_id = int(sumo_sim_manager.vehID_list[k][3:])
            ego_states = sumo_sim_manager.sumo_veh[ego_id].getVehicleStates()
            sumo_sim_manager.update_preceding_vehicle(veh_id=ego_id)
            
            # Check if preceding vehicle exists
            if sumo_sim_manager.sumo_veh[ego_id].pv_ID is not None:
                if sumo_sim_manager.sumo_veh[ego_id].pv_ID in sumo_sim_manager.vehID_list:
                    pv_id = int(sumo_sim_manager.sumo_veh[ego_id].pv_ID[3:])
                    pv_states = sumo_sim_manager.sumo_veh[pv_id].getVehicleStates()
                else:
                    pv_states = [0, 20, ego_states[2] + 100]
            else:
                pv_states = [0, 20, ego_states[2] + 100]
            
            # Store vehicle states
            veh_ctrl_input[:, k] = np.concatenate((ego_states, pv_states))
        
        # Apply control all traffic vehicles in the sim
        if USING_IDM:
            veh_acc_t = IDM_control.IDM_acceleration(front_v=veh_ctrl_input[4, :], ego_v=veh_ctrl_input[1, :],
                                                     front_s=veh_ctrl_input[5, :], ego_s=veh_ctrl_input[2, :])
        if USING_NEURAL_NETWORK:
            veh_acc_t = FCN_control.step_forward(s_vt=veh_ctrl_input[1, :], pv_vt=veh_ctrl_input[4, :],
                                                   s_st=veh_ctrl_input[2, :], pv_st=veh_ctrl_input[5, :],
                                                   s_at=veh_ctrl_input[0, :], pv_at=veh_ctrl_input[3, :])
        
        for k in range(len(sumo_sim_manager.vehID_list)):
            # Get ego vehicle states
            ego_id = int(sumo_sim_manager.vehID_list[k][3:])
            sumo_sim_manager.sumo_veh[ego_id].assignTargetAcceleration(veh_acc_t[k], 20)
        loop_end_t = time.time()
        print("Traffic simulation duration: " + str(round(sim_t, 1)) + " with " + str(len(sumo_sim_manager.vehID_list)) + 
              " vehicles in traffic. The control runtime for this frame is: " + str(round((loop_end_t - loop_start_t) * 1000, 2)) + " ms")
        runtime_record.append(round((loop_end_t - loop_start_t) * 1000, 2))
        time.sleep(0.01)
    print('Average runtime is: ' + str(np.mean(np.array(runtime_record))))