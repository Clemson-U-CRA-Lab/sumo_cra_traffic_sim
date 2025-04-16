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
        self.num_veh = 0
        self.step = 0
    
    def init_vehicles_large_map(self, num_vehicle):
        self.num_veh = num_vehicle
        self.sumo_veh = [None]*num_vehicle
        for i in range(int(self.num_veh / 2)):
            self.sumo_veh[i] = SUMO_vehicles(vehicle_ID="veh" + str(i), init_s= 20 - 12 * i, init_lane=0, route_ID="route1", lane_change_mode=0)
        for j in range(int(self.num_veh / 2), self.num_veh):
            self.sumo_veh[j] = SUMO_vehicles(vehicle_ID="veh" + str(j), init_s= 30 - 12 * (j - int(num_veh/2)), init_lane=0, route_ID="route1", lane_change_mode=0)
    
    def init_vehicles_CMI(self, num_vehicle):
        self.num_veh = num_vehicle
        self.sumo_veh = [None]*num_vehicle
        for i in range(self.num_veh):
            self.sumo_veh[i] = SUMO_vehicles(vehicle_ID="veh" + str(i), init_s= 30 - 10 * i, init_lane=0, route_ID="route1", lane_change_mode=0)

    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
    
    def simulationStepForward(self):
        traci.simulationStep()
        self.vehID_list = traci.vehicle.getIDList()
        self.step += 1


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logging_sim", help="whether to save the simulation data", action="store_true")
    parser.add_argument("--num_sv", type=int, default=5.0, help="Number of vehicles in the traffic")
    parser.add_argument('leading_speed_profile', choices=['Nyc', 'Hwy', 'US06','Simple'], help='Choose leading vehicles speed profile')
    parser.add_argument("control_type", choices=['MPC', 'NN', 'IDM'], help='Choose control method for traffic vehicles')
    args = parser.parse_args()
    
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
        
    num_veh = args.num_sv
    
    Power_t = []
    
    veh_sim_t = []
    
    runtime_record = []
    
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    if args.leading_speed_profile == 'Hwy':
        spd_filename = parent_dir + "/speed_profile/I85_hwycol.csv"
    elif args.leading_speed_profile == 'Nyc':
        spd_filename = parent_dir + "/speed_profile/I85_nycccol.csv"
    elif args.leading_speed_profile == 'US06':
        spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    elif args.leading_speed_profile == 'Simple':
        spd_filename = parent_dir + "/speed_profile/ITIC_Lane_Change_Modeling_StartLane_Fast.csv"
    else:
        print('Unable to locate speed profile')
        sys.exit(1)
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    
    if args.leading_speed_profile == 'Hwy' or args.leading_speed_profile == 'Nyc':
        sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/I-85_highway/I-85.sumocfg")
        sumo_sim_manager.init_vehicles_large_map(num_vehicle=num_veh)
    else:
        sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/CMI/cmi.sumocfg")
        sumo_sim_manager.init_vehicles_CMI(num_vehicle=num_veh)
        
    sumo_sim_manager.start_Sumo()
    
    # Initialize controller
    dirname = os.path.dirname(__file__)
    nn_pt_filename = dirname + '/traffic_following_control.pt'
    
    # Setup controller
    if USING_NEURAL_NETWORK:
        FCN_control = NN_controller(nn_pt_file=nn_pt_filename)
        controller_name = 'Neural_Network'
        print('Use neural network to control traffic vehicles')
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
    
    Avg_spd_traffic = []
    Avg_density_traffic = []
    lead_s = 325.0
    end_s = 0.0
    
    while sumo_sim_manager.step * 0.1 < record_t[-1]:
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        
        # Initialize power record
        P_t = []
        Spd_t = []
        
        start_t = time.time()
        
        s_vt_traffic = []
        pv_vt_traffic = []
        s_st_traffic = []
        pv_st_traffic = []
        
        for i in range(0, num_veh):
            if i == 0 or  i == int(num_veh/2):
                # Get leading vehicle speed
                v_lead_id = np.argmin(np.abs([record_t - sim_t]))
                v_tgt_lead = front_v_t[v_lead_id] #+ 2.0 * (random.random() - 0.5)
                sumo_sim_manager.sumo_veh[i].assignTargetSpeed(v_tgt_lead)
                [veh_1_acc_t, veh_1_spd_t, veh_1_dist_t] = sumo_sim_manager.sumo_veh[i].getVehicleStates()
                lead_s = veh_1_dist_t
                # Update state preview
                lead_prev_v, lead_prev_s = driving_cycle_state_preview_searching(sim_t=sim_t, record_t=record_t, front_v_t=front_v_t, mpc_dt=0.5, front_s_init=lead_s)
                # Load future state preview                 
                sumo_sim_manager.sumo_veh[i].update_vehicle_future_states_preview(lead_prev_s, lead_prev_v)
                continue
            
            [veh_0_acc_t, veh_0_spd_t, veh_0_dist_t] = sumo_sim_manager.sumo_veh[i-1].getVehicleStates()
            [veh_1_acc_t, veh_1_spd_t, veh_1_dist_t] = sumo_sim_manager.sumo_veh[i].getVehicleStates()
            
            if i == num_veh - 1:
                end_s = veh_1_dist_t
            
            # Compute vehicle power at time sim_t
            P_t.append(engine_power_estimation(ego_v=veh_1_spd_t, ego_a=veh_1_acc_t))
            
            # Add the vehicle speed
            Spd_t.append(veh_1_spd_t)
            
            if USING_ONLINE_MPC:
                acc_traffic_step_t = traffic_online_MPC_control_step(veh_0_acc_t, veh_0_spd_t, veh_0_dist_t,
                                                                     veh_1_acc_t, veh_1_spd_t, veh_1_dist_t,
                                                                     sim_t, online_MPC_control, record_t,
                                                                     front_v_t, 0.5, pv_object=sumo_sim_manager.sumo_veh[i-1],
                                                                     ego_object=sumo_sim_manager.sumo_veh[i], leading_preview=False)
            elif USING_NEURAL_NETWORK:
                s_vt_traffic.append(veh_1_spd_t)
                pv_vt_traffic.append(veh_0_spd_t)
                s_st_traffic.append(veh_1_dist_t)
                pv_st_traffic.append(veh_0_dist_t)
                continue
            elif USING_IDM:
                acc_traffic_step_t = IDM_control.IDM_acceleration(front_v=np.array([veh_0_spd_t]),
                                                                  ego_v=np.array([veh_1_spd_t]),
                                                                  front_s=np.array([veh_0_dist_t]),
                                                                  ego_s=np.array([veh_1_dist_t]))
            else:
                acc_traffic_step_t = np.zeros(3)
                
            acc = acc_traffic_step_t[0]
            
            # Assign the acceleration to ego vehicle
            sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc)
        
        if USING_NEURAL_NETWORK:
            acc_traffic_step_t = FCN_control.step_forward(s_vt=np.array(s_vt_traffic), pv_vt=np.array(pv_vt_traffic), 
                                                          s_st=np.array(s_st_traffic), pv_st=np.array(pv_st_traffic))
            
            for i in range(1, num_veh):
                if i < int(num_veh / 2):
                    sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc_traffic_step_t[i-1])
                elif i > int(num_veh / 2):
                    sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc_traffic_step_t[i-2])
                else:
                    continue
        
        # Add power consumption
        Power_t.append(P_t)
        
        # Add traffic flow speed
        avg_spd_t = np.mean(np.array(Spd_t))
        Avg_spd_traffic.append(avg_spd_t)
        
        # Add traffic density
        traffic_rho = traffic_density_measurement(lead_s=lead_s, end_s=end_s, num_vehicles=num_veh)
        Avg_density_traffic.append(traffic_rho)
        veh_sim_t.append(sim_t)
        
        runtime_record.append(time.time() - start_t)
        
        if args.logging_sim:
            data_logger(sim_t=sim_t, ego_a=acc, ego_v=veh_1_spd_t, ego_s=veh_1_dist_t,
                        pv_a=veh_0_acc_t, pv_v=veh_0_spd_t, pv_s=veh_0_dist_t, filename= args.leading_speed_profile + "_" + controller_name + ".csv")
        
        time.sleep(0.01)
    
    traci.close(True)
    
    veh_sim_t = np.array(veh_sim_t)
    
    Energy_t = np.sum(np.array(Power_t) * 0.1, axis=0)
    
    print(np.round(Energy_t / 1000, decimals=2))
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    print('Runtime standard deviation is: ', str(round(np.std(runtime_record) * 1000, 4)), 'ms')
    
    plt.figure(1)
    plt.subplot(2,1,1)
    plt.plot(veh_sim_t, Avg_spd_traffic, '-b')
    plt.xlabel('Time [s]', fontsize=20)
    plt.ylabel('Average speed [m/s]', fontsize=20)
    
    plt.subplot(2,1,2)
    plt.plot(veh_sim_t, Avg_density_traffic, '-b')
    plt.xlabel('Time [s]', fontsize=20)
    plt.ylabel('Traffic density [veh/km]', fontsize=20)
    
    plt.figure(2)
    plt.hist(Avg_density_traffic, bins=40, color='skyblue', density=True)
    plt.xlabel('Average density [veh/km]', fontsize=20)
    plt.ylabel('Percentage', fontsize=20)
    plt.xlim([0, 150])
    
    plt.show()