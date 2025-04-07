#! /usr/bin/env python3

# Has not TCP sockets. Simple scenario wih mpc-pcc. sumo2v_v2.py has tcp sockets too.
# this was a WIP towards sumo2V_v2.py

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

StalledNv = 'nv1' # the car that stalls
RealCav = "nv2" # mache

realtime_pacing = False
vizTraj = True
AccIntegrateDT = MPC_DT # MPC_DT or SIM_STEP


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
    sumo_sim_manager = SumoSim(sumo_config_name=parent_dir + "/sumo/v2x/v2x_2veh.sumocfg")
    sumo_sim_manager.start_Sumo()

    # 96 - no checks, 0 - most chcks off but speed limit adhered
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMinGap(vehID, 0.1)
        traci.vehicle.setSpeed(vehID, 0.0)
        traci.vehicle.setSpeedMode(vehID, 96)
        # traci.vehicle.setAccel(vehID, 10)
        # traci.vehicle.setDecel(vehID, 10)
        # traci.vehicle.setEmergencyDecel(vehID, 10)

    # traci.gui.trackVehicle("View #0", "nv2")
    # traci.gui.setZoom("View #0", 500)
    
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
    while sumo_sim_manager.step < END_TIME/SIM_STEP:

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
        if sim_time < 45:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=v_tgt_lead)
        else:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=0)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]

        # SOLVE CONOTROL
        # Run MPC control if enabled
        start_t = time.time()
        if USING_ONLINE_MPC:
            acc, preds_s, preds_v = traffic_online_MPC_control_step_nVeh(veh_states_matrix, 
                                                       sim_t=sim_time, 
                                                       record_t=record_t,
                                                       front_v_t=front_v_t,
                                                       online_MPC_control=online_MPC_control,
                                                       simStep=SIM_STEP,
                                                       mpc_dt=MPC_DT,
                                                       mpc_ref_stages=6,
                                                       verbose=True,
                                                       useFirstVehCyclePreview=True
                                                       )
        else:
            acc = {}
            for veh in vehicle_list:
                acc[veh] = 0.0
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
           
        
        # Assign the acceleration to MAchE vehicle
        sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=acc['nv1'], dt=AccIntegrateDT) # MPC_DT or SIM_STEP


        veh_0_acc.append(veh_states_matrix[0][1])
        veh_0_spd.append(veh_states_matrix[0][2])
        veh_0_dist.append(veh_states_matrix[0][3])
        
        veh_1_acc.append(veh_states_matrix[1][1])
        veh_1_spd.append(veh_states_matrix[1][2])
        veh_1_dist.append(veh_states_matrix[1][3])
        mache_accCmd.append(acc["nv1"])
        
        veh_sim_t.append(sim_time)

        # Sleep timing
        real_now = time.monotonic()
        
        if realtime_pacing:
            sleep_time = max(0, real_expected_time - real_now)  # Sleep only if ahead of real time
            time.sleep(sleep_time)  # Sync with real-world time
        print(f"Real elapsed: {real_now - real_start_time:.3f}s, Sim Time: {sim_time:.3f}s")
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    
    plt.figure(1)
    
    plt.subplot(3,1,1)
    plt.plot(veh_sim_t, veh_0_dist,'k')
    plt.plot(veh_sim_t, veh_1_dist,'b--')
    # plt.plot(veh_sim_t, veh_3_dist)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['Leading Vehicle',  'mache'])
    
    plt.subplot(3,1,2)
    plt.plot(veh_sim_t, veh_0_spd, 'k')
    plt.plot(veh_sim_t, veh_1_spd, 'b--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['Leading Vehicle','mache'])

    plt.subplot(3,1,3)
    plt.plot(veh_sim_t, veh_0_acc, 'k')
    plt.plot(veh_sim_t, veh_1_acc,'b')
    plt.plot(veh_sim_t, mache_accCmd, 'b--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Acc [m/s^2]')
    plt.legend(['Leading Vehicle', 'mache', 'mache_accCmd'])
    
    plt.show()
    
    traci.close(False)