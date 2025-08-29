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

# import classes
from SumoSim import SumoSim
from x2v_constants import *

# logging utils
from  utils_logging import *
logRunning_ = False
if logRunning_:
    fileNameTemp = 'sumo_v2x_logRuntime' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'

# Comms
asyncSocket = True
if asyncSocket:
    from x2vSocketInterface_periodic import x2vSocketInterfaceAsync as x2vSocketInterface
else:
    from x2vSocketInterface_periodic import x2vSocketInterface as x2vSocketInterface

# Run params
guiSumo = True
vizTraj = True
testWithoutGPS = True

if __name__=="__main__":

    # Init socket connections
    sockInt = x2vSocketInterface()

    # Dynamic vehicle data storage
    veh_data = {}
    veh_sim_t = []
    veh_real_t = []
    runtime_record = []
    runtimeAll_record = []

    # init speed profiles
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3])+20.0

    # Init SUMO sim
    sumo_sim_manager = SumoSim(sumo_config_name=parent_dir + "/sumo/v2x/" + SUMO_CONFIG)
    sumo_sim_manager.start_Sumo(gui=guiSumo)

    if guiSumo:
        traci.gui.trackVehicle("View #0", "nv1")
        traci.gui.setZoom("View #0", 500)
    
    # Inti and Setup controller
    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')
        
    # Start simulation 
    real_start_time = time.monotonic()  
    real_now = real_start_time
    sim_start_time = 0  # SUMO's starting simulation time
    IntiSpeedSet = False
    while sumo_sim_manager.step < END_TIME/SIM_STEP:

        sim_time = traci.simulation.getTime()  # Get SUMO's current simulation time
        real_expected_time = real_start_time + (sim_time - sim_start_time)
        sumo_sim_manager.simulationStepForward()
        vehicle_list = traci.vehicle.getIDList()

        # start_t0 = time.time()

        if not IntiSpeedSet:
            for veh in vehicle_list:
                traci.vehicle.setSpeed(veh, 0.0)  # Set initial speed to 0 for all vehicles
                IntiSpeedSet = True

        # Initialize data storage for new vehicles
        for veh in vehicle_list:
            if veh not in veh_data:
                veh_data[veh] = {'dist': [], 'spd': [], 'lane': [], 'acc': [], 'accCmd': []}

        # Assign speeds to leading vehicle
        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]
        if "nv0" in vehicle_list:
            if sim_time < STALLTIME:
                sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=v_tgt_lead)
            else:
                sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=0)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]
        veh_states_dict = {veh: state for veh, state in zip(vehicle_list, veh_states_matrix)}

        # SOLVE CONTROL
        start_t = time.time()
        if USING_ONLINE_MPC:
            acc, preds_s, preds_v, cycle_ss, cycle_vs = traffic_online_MPC_control_step_nVeh(
                veh_states_matrix, 
                sim_t=sim_time, 
                record_t=record_t,
                front_v_t=front_v_t,
                online_MPC_control=online_MPC_control,
                simStep=SIM_STEP, # unused
                mpc_dt=MPC_DT,
                mpc_ref_stages=MPC_REF_STAGES,
                cycle_dt=REF_CYCLE_DT,
                cycle_stages= REF_CYCLE_STAGES,
                PassIntention=BOOL_USE_FRONT_PREVIEW,
                outputUsedCycleforFront=True,
                verbose=False
            )
        else:
            acc = {veh: 0.0 for veh in vehicle_list}
        runtime_record.append(time.time() - start_t)

        # viz traj
        if vizTraj:
            sumo_sim_manager.add_traj_leader("nv0", 
                                            veh_states_matrix[0][3],
                                            record_t=record_t,
                                            front_v_t=front_v_t,
                                            sim_t=sim_time,
                                            pred_dt=MPC_DT, mpc_ref_stages=MPC_REF_STAGES,
                                            colorChoice=(255,255,100), fill=False, layer=3)
            sumo_sim_manager.add_traj("nv1", preds_s=preds_s["nv1"],colorChoice=(0, 255, 2, 100), fill=False, layer=4)
        if sim_time >= STALLTIME:
            # Stalling it
            sim_nv_array = [sim_time, 
                            veh_states_matrix[1][3], veh_states_matrix[1][2], veh_states_matrix[1][1], # ego
                            veh_states_matrix[0][3], veh_states_matrix[0][2], veh_states_matrix[0][1]  # front
                            ] + [veh_states_matrix[0][3]]*REF_CYCLE_STAGES + [0.0]*REF_CYCLE_STAGES # front's s, front's v
        else:
            # sim_time, ego_s, ego_v, ego_a  front_s, front_v, front_a, ...
            sim_nv_array = [sim_time, 
                            veh_states_matrix[1][3], veh_states_matrix[1][2], veh_states_matrix[1][1], # ego
                            veh_states_matrix[0][3], veh_states_matrix[0][2], veh_states_matrix[0][1]  # front
                            ] + [x for x in cycle_ss] + [x for x in cycle_vs] # front's s, front's v   
        
        # Send NV states to realCAV
        sockInt.send_sim_info(sim_nv_array)
        # print(f"Send Front info: {sim_nv_array[0], sim_nv_array[1:4], sim_nv_array[4:7]}")

        #update SUMO vehicles
        for veh in vehicle_list[1:-1]:  # Skip nv0 and realCAV
            sumo_sim_manager.assignAcceleration(vehicle_ID=veh, tgt_acc=acc.get(veh, 0.0), dt=SUMO_ACC_INTEGRATE_DT)

        # Recv realCAV info and updat ereal CAV in sim
        realCavArray = sockInt.get_veh_info()
        if realCavArray is not None:        
            print(f"{bcolors.OKCYAN}==============Got from VEH============{bcolors.ENDC}" )
            # print(f"{bcolors.OKCYAN}Elapsed @ VEH Real: {realCavArray[6]:.2f}, {bcolors.OKBLUE}MPC got SimTime: {realCavArray[0]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKGREEN}Delta T RSPCSim-VEHReal: {(sim_time-realCavArray[6]):.2f}s{bcolors.ENDC}")
            print(f"{bcolors.OKCYAN}Ego x,y: {realCavArray[4]:.2f}, {realCavArray[5]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKCYAN}Ego [GPS] s: -- , v:{realCavArray[2]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKCYAN}Ego MpcCmd: {realCavArray[7]:.2f}.{bcolors.ENDC}" )

        # Update realCAV in SUMO
        realCAVName = vehicle_list[-1]  # Assuming the last vehicle in the list is the real CAV
        if testWithoutGPS:
            # if local testing w/o gps:
            sumo_sim_manager.assignAcceleration(vehicle_ID=realCAVName, tgt_acc=realCavArray[7], dt=SUMO_ACC_INTEGRATE_DT) # careful: assign commmand or real sensed acc?
        else:
            # if testing with gps and vehicle run
            sumo_sim_manager.update_CAV_in_sumo(veh=realCAVName, 
                                                    spd=realCavArray[2],
                                                    pos=[realCavArray[4],realCavArray[5]]
                                                    )

        # Logging and data collection
        for veh in vehicle_list:
            veh_data[veh]['acc'].append(veh_states_dict[veh][1])
            veh_data[veh]['spd'].append(veh_states_dict[veh][2])
            veh_data[veh]['dist'].append(veh_states_dict[veh][3])
            # veh_data[veh]['lane'].append(veh_states_dict[veh][4] if len(veh_states_dict[veh]) > 4 else None)
            veh_data[veh]['accCmd'].append(acc.get(veh, 0.0))
        veh_sim_t.append(sim_time)


        # runtimeAll_record.append(time.time() - start_t0)

        # Sleep timing
        real_now = time.monotonic()
        veh_real_t.append(real_now - real_start_time)

        if asyncSocket:
            sleep_time = max(0, real_expected_time - real_now)
            time.sleep(sleep_time)
        print(f"Sim-Real Delta: {((real_now - real_start_time)-sim_time):.2f}s | Vehicles: {vehicle_list}")
    
    print('Average MPC runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    # print('Average runtime Whole is: ', str(round(np.mean(runtimeAll_record) * 1000, 4)), 'ms')

    # Save all vehicle data in one CSV file
    save_all_veh_data_csv(veh_data, veh_sim_t, real_time_list=veh_real_t , file_prefix='sumo_log')

    # Dynamic plotting
    plt.figure(2)
    plt.subplot(3,1,1)
    plt.plot(record_t, front_s_t, 'r:')
    for veh in veh_data:
        plt.plot(veh_sim_t, veh_data[veh]['dist'], label=f'{veh} dist')
    plt.xlabel('Time [s]')
    plt.ylabel('Distance [m]')
    plt.legend()
    plt.subplot(3,1,2)
    plt.plot(record_t, front_v_t, 'r:')
    for veh in veh_data:
        plt.plot(veh_sim_t, veh_data[veh]['spd'], label=f'{veh} spd')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend()
    plt.subplot(3,1,3)
    for veh in veh_data:
        plt.plot(veh_sim_t, veh_data[veh]['acc'], label=f'{veh} acc')
        plt.plot(veh_sim_t, veh_data[veh]['accCmd'], '--', label=f'{veh} accCmd')
    plt.xlabel('Time [s]')
    plt.ylabel('Acc [m/s^2]')
    plt.legend()
    plt.show()
    traci.close(False)