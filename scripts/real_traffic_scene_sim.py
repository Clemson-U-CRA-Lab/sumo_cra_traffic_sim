#! /usr/bin/env python3

import os
import sys
import argparse
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _i85_traffic_info import *
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
        self.traffic_density_meas = 0
        self.density_meas_s_start = 0
        self.density_meas_s_end = 0

    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
        traci.gui.setSchema("View #0", "real world")
    
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
        if len(self.vehID_list) >= 0:
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
    
    def update_traffic_density_measurement(self, s_start, s_end, num_vehicles):
        self.density_meas_s_start = s_start
        self.density_meas_s_end = s_end
        self.traffic_density_meas = num_vehicles
    
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
    traffic_light_manager = SUMO_Traffic_Light(s_TL=TL_s, t_TL=TL_timing, status_TL=TL_status, red_duration=25, amber_duration=2.5, green_duration=45)
    
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
        IDM_control = IDM(a=2, b=5, s0=8, v0=30, T=5)
        controller_name = 'Intelligent Driving Model'
        print('Use IDM to control traffic vehicles')
    else:
        print('No controller for all vehicles')
    
    
    # Initialize Traffic
    CHI_init_state = getChicagoTraffic(args.scenario_id)
    sumo_sim_manager.init_scenario(CHI_init_state)
    sim_t_record = []
    runtime_record = []
    traffic_flow_record = []
    power_record = []
    spd_record = []
    
    while True:
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        sumo_sim_manager.update_traffic_density_measurement(s_start=2000, s_end=3000, num_vehicles=0)
        power_t = 0
        spd_t = 0
        
        # Check if the sim terminate
        if len(sumo_sim_manager.vehID_list) == 0 and sim_t > 20:
            print('\n')
            print('Simulation Terminated')
            break
        
        # Update all traffic vehicles' status
        for i in range(sumo_sim_manager.num_veh):
            sumo_sim_manager.init_status_update(i)
        
        # Update all traffic light information
        for j in range(num_TL):
            traffic_light_manager.TL_update(TL_id=j)
        
        # Initialize each vehicle states vehicles
        veh_ctrl_input = np.zeros((7, len(sumo_sim_manager.vehID_list)))
        
        loop_start_t = time.time() # Start recording runtime
        # Update traffic vehicles inside the traffic
        for k in range(len(sumo_sim_manager.vehID_list)):
            # Get ego vehicle states
            ego_id = int(sumo_sim_manager.vehID_list[k][3:])
            ego_states = sumo_sim_manager.sumo_veh[ego_id].getVehicleStates()
            sumo_sim_manager.sumo_veh[ego_id].update_preceding_traffic_light(TL_s=TL_s)
            sumo_sim_manager.update_preceding_vehicle(veh_id=ego_id)
            
            if ego_states[2] > sumo_sim_manager.density_meas_s_start and ego_states[2] < sumo_sim_manager.density_meas_s_end:
                sumo_sim_manager.traffic_density_meas += 1
            power_t += engine_power_estimation(ego_a=ego_states[0], ego_v=ego_states[1]) 
            spd_t += ego_states[1]
            
            # Check if preceding vehicle exists
            if sumo_sim_manager.sumo_veh[ego_id].pv_ID is not None:
                if sumo_sim_manager.sumo_veh[ego_id].pv_ID in sumo_sim_manager.vehID_list:
                    pv_id = int(sumo_sim_manager.sumo_veh[ego_id].pv_ID[3:])
                    pv_states = sumo_sim_manager.sumo_veh[pv_id].getVehicleStates()
                    # Check if stop in front of the traffic light is needed
                    if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] is not None:
                        if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] == 0:
                            if sumo_sim_manager.sumo_veh[ego_id].pTL_s < 200:
                                pv_states = [0, 0, ego_states[2] + sumo_sim_manager.sumo_veh[ego_id].pTL_s]
                # else:
                #     if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] is not None:
                #         if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] == 0:
                #             pv_states = [0, 0, ego_states[2] + sumo_sim_manager.sumo_veh[ego_id].pTL_s]
                #         else:
                #             pv_states = [1, ego_states[1] + 5, ego_states[2] + 200]
            else:
                if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] is not None:
                    if traffic_light_manager.status_TL[sumo_sim_manager.sumo_veh[ego_id].pTL_id] == 0:
                        pv_states = [0, 0, ego_states[2] + sumo_sim_manager.sumo_veh[ego_id].pTL_s]
                    else:
                        pv_states = [1, ego_states[1] + 5, ego_states[2] + 200]
            
            # Store vehicle states
            veh_ctrl_input[0:-1, k] = np.concatenate((ego_states, pv_states))
        
        # Apply control all traffic vehicles in the sim
        if USING_IDM:
            veh_acc_t = IDM_control.IDM_acceleration(front_v=veh_ctrl_input[4, :], ego_v=veh_ctrl_input[1, :],
                                                     front_s=veh_ctrl_input[5, :], ego_s=veh_ctrl_input[2, :])
        if USING_NEURAL_NETWORK:
            veh_acc_t = FCN_control.step_forward(s_vt=veh_ctrl_input[1, :], pv_vt=veh_ctrl_input[4, :],
                                                   s_st=veh_ctrl_input[2, :], pv_st=veh_ctrl_input[5, :],
                                                   s_at=veh_ctrl_input[0, :], pv_at=veh_ctrl_input[3, :])
        if USING_ONLINE_MPC:
            veh_acc_t = []
            for i in range(len(sumo_sim_manager.vehID_list)):
                mpc_acc_t = traffic_online_MPC_control_step(veh_0_acc_t=veh_ctrl_input[3, i], veh_0_spd_t=veh_ctrl_input[4, i], veh_0_dist_t=veh_ctrl_input[5, i],
                                                            veh_1_acc_t=veh_ctrl_input[0, i], veh_1_spd_t=veh_ctrl_input[1, i], veh_1_dist_t=veh_ctrl_input[2, i],
                                                            sim_t=sim_t, online_MPC_control=online_MPC_control, record_t=[], front_v_t=[], mpc_dt=0.5,
                                                            pv_object=None, ego_object=None, leading_preview=False)
                veh_acc_t.append(mpc_acc_t[0])
        loop_end_t = time.time() # End recording runtime
        
        for k in range(len(sumo_sim_manager.vehID_list)):
            # Get ego vehicle states
            ego_id = int(sumo_sim_manager.vehID_list[k][3:])
            sumo_sim_manager.sumo_veh[ego_id].assignTargetAcceleration(veh_acc_t[k], 30)
            
        if len(sumo_sim_manager.vehID_list) > 0:
            sim_t_record.append(sim_t)
            traffic_flow_record.append(sumo_sim_manager.traffic_density_meas)
            power_record.append(power_t)
            runtime_record.append(round((loop_end_t - loop_start_t) * 1000, 2))
            spd_t_avg = spd_t / len(sumo_sim_manager.vehID_list)
            spd_record.append(round(spd_t_avg, 2))
            
            print("Simulation duration: " + str(round(sim_t, 1)) 
                  + ". Num Vehicles: " + str(len(sumo_sim_manager.vehID_list)) 
                  + " vehicles. Runtime is: " + str(round((loop_end_t - loop_start_t) * 1000, 2))
                  + " ms. Edge flow is: " + str(sumo_sim_manager.traffic_density_meas)
                  + ". Avg EV power: " + str(round(power_t / (1000 * len(sumo_sim_manager.vehID_list)), 2)) 
                  + ". Avg speed is: " + str(round(spd_t_avg, 2)), end='\r')
        
        time.sleep(0.01)
    
    print('Average runtime is: ' + str(np.mean(np.array(runtime_record))))
    # Compute total power consumption
    E = np.sum(np.array(power_record) * 0.1)
    print('Total energy consumption of all vehicles: ' + str(round(E / 1000, 2)) + ' kJ.')
    print('Average traffic flow between 2-3km is: ' + str(round(np.mean(np.array(traffic_flow_record)), 2)))
    print('Sim duration: ' + str(sim_t_record[-1] - sim_t_record[0]) + ' s.')
    
    plt.figure(1)
    plt.subplot(3,1,1)
    plt.plot(sim_t_record, traffic_flow_record, 'k-', linewidth=2.5)
    plt.xlabel('Time [s]')
    plt.ylabel('Traffic flow [nveh]')
    plt.title('Scenario' + str(args.scenario_id))
    
    plt.subplot(3,1,2)
    plt.plot(sim_t_record, spd_record, 'k-', linewidth=2.5)
    plt.xlabel('Time [s]')
    plt.ylabel('Average speed [m/s]')
    
    plt.subplot(3,1,3)
    plt.plot(sim_t_record, runtime_record, 'k-', linewidth=2.5)
    plt.xlabel('Time [s]')
    plt.ylabel('Runtime [ms]')
    
    plt.show()