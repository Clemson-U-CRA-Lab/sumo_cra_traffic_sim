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
LOG_RUNNING = False
fileNameTemp = 'sumoSim_v2x_logRuntime' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'
csv_header = ["Realtime [sec]","Sim Time [sec]", "MPC runtime",\
                                "v0_dist [m]","v0_lane [-]","v0_spd [m/s]","v1_acc [m/s2]",\
                                "v1_dist [m]","v1_lane [-]","v1_spd [m/s]","v1_acc [m/s2]","MachE_accCmd [m/s2]"]
data = np.zeros([int(END_TIME/SIM_STEP)+1,len(csv_header)])


# Comms
asyncSocket = True
if asyncSocket:
    from x2vSocketInterface import x2vSocketInterfaceAsync as x2vSocketInterface
else:
    from x2vSocketInterface import x2vSocketInterface as x2vSocketInterface

# Run params
vizTraj = True
AccIntegrateDT = MPC_DT # MPC_DT or SIM_STEP
guiSumo = True
stallTime = 180 #45 seconds, 180 for no stall at cmi


if __name__=="__main__":

    # Init socket connections
    sockInt = x2vSocketInterface()


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
    sumo_sim_manager.start_Sumo(gui=guiSumo)

    # 96 - no checks, 0 - most chcks off but speed limit adhered
    for vehID in traci.vehicle.getIDList():
        traci.vehicle.setMinGap(vehID, 0.1)
        traci.vehicle.setSpeed(vehID, 0.0)
        traci.vehicle.setSpeedMode(vehID, 96)
        # traci.vehicle.setAccel(vehID, 10)
        # traci.vehicle.setDecel(vehID, 10)
        # traci.vehicle.setEmergencyDecel(vehID, 10)

    if guiSumo:
        traci.gui.trackVehicle("View #0", "nv1")
        traci.gui.setZoom("View #0", 500)
    
    # Inti and Setup controller
    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')
        
    # Start simulation 
    total_elapsed = 0
    # Get the real-world start time
    real_start_time = time.monotonic()  
    real_now = real_start_time

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
        if sim_time < stallTime:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=v_tgt_lead)
        else:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=0)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]

        # SOLVE CONOTROL
        # Run MPC control if enabled
        start_t = time.time()
        if USING_ONLINE_MPC:
            acc, preds_s, preds_v, cycle_ss, cycle_vs = traffic_online_MPC_control_step_nVeh(veh_states_matrix, 
                                                       sim_t=sim_time, 
                                                       record_t=record_t,
                                                       front_v_t=front_v_t,
                                                       online_MPC_control=online_MPC_control,
                                                       simStep=SIM_STEP,
                                                       mpc_dt=MPC_DT,
                                                       mpc_ref_stages=MPC_REF_STAGES,
                                                       PassIntention=BOOL_USE_FRONT_PREVIEW,
                                                       outputUsedCycleforFront=True,
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
            # sumo_sim_manager.add_traj("nv1", preds_s=preds_s["nv1"],colorChoice=(0, 255, 2, 100), fill=False, layer=4)
           
        # Assign the acceleration to leader nv0
        if sim_time >= 45.0:
            # Stalling it
            lead_nv_array = [sim_time, 
                            veh_states_matrix[1][3], veh_states_matrix[1][2], veh_states_matrix[1][1], # ego
                            veh_states_matrix[0][3], veh_states_matrix[0][2], veh_states_matrix[0][1]  # front
                            ] + [veh_states_matrix[0][3]]*32 + [0.0]*32 # front's s, front's v
        else:
            # sim_time, ego_s, ego_v, ego_a  front_s, front_v, front_a, ...
            lead_nv_array = [sim_time, 
                            veh_states_matrix[1][3], veh_states_matrix[1][2], veh_states_matrix[1][1], # ego
                            veh_states_matrix[0][3], veh_states_matrix[0][2], veh_states_matrix[0][1]  # front
                            ] + [x for x in cycle_ss] + [x for x in cycle_vs] # front's s, front's v   
        
        # Send NV states to realCAV
        sockInt.send_sim_info(lead_nv_array)
        # print(f"Send Front info: {lead_nv_array[0], lead_nv_array[1:4], lead_nv_array[4:7]}")

        # Recv realCAV info and updat ereal CAV in sim
        realCavArray = sockInt.get_veh_info()
        if realCavArray is not None:        
            print(f"{bcolors.OKCYAN}==============Got from VEH============{bcolors.ENDC}" )
            # print(f"{bcolors.OKCYAN}Elapsed @ VEH Real: {realCavArray[6]:.2f}, {bcolors.OKBLUE}MPC got SimTime: {realCavArray[0]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKGREEN}Delta T RSPCSim-VEHReal: {(sim_time-realCavArray[6]):.2f}s{bcolors.ENDC}")
            print(f"{bcolors.OKCYAN}Ego x,y: {realCavArray[4]:.2f}, {realCavArray[5]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKCYAN}Ego [GPS] s: -- , v:{realCavArray[2]:.2f}.{bcolors.ENDC}" )
            print(f"{bcolors.OKCYAN}Ego MpcCmd: {realCavArray[7]:.2f}.{bcolors.ENDC}" )


            # Update Real CAV pos in simulation:::

            # if local testing w/o gps:
            sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=realCavArray[7], dt=AccIntegrateDT) # careful: assign commmand or real sensed acc?
            
            # if testing with gps and vehicle run
            # sumo_sim_manager.update_CAV_in_sumo(veh='nv1', 
            #                                         spd=realCavArray[2],
            #                                         pos=[realCavArray[4],realCavArray[5]]
            #                                         )

        
            veh_0_acc.append(veh_states_matrix[0][1])
            veh_0_spd.append(veh_states_matrix[0][2])
            veh_0_dist.append(veh_states_matrix[0][3])
            
            veh_1_acc.append(veh_states_matrix[1][1])
            veh_1_spd.append(veh_states_matrix[1][2])
            veh_1_dist.append(veh_states_matrix[1][3])
            mache_accCmd.append(realCavArray[7])
        
            veh_sim_t.append(sim_time)

            data[sumo_sim_manager.step,:] = ([real_now-real_start_time, sim_time, time.time() - start_t,
                        veh_states_matrix[0][3],0.0,veh_states_matrix[0][2],veh_states_matrix[0][1],
                        veh_states_matrix[1][3],0.0,veh_states_matrix[1][2],veh_states_matrix[1][1], realCavArray[7]])
            
        if LOG_RUNNING:
            with open(fileNameTemp, "a", newline="") as csv_file:
                # Create a CSV writer object
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(data[sumo_sim_manager.step,:])            

        # Sleep timing
        real_now = time.monotonic()
        if asyncSocket:
            sleep_time = max(0, real_expected_time - real_now)  # Sleep only if ahead of real time
            time.sleep(sleep_time)  # Sync with real-world time
        print(f"{bcolors.OKGREEN}Delta T RSPC[Sim-Real]: {((real_now - real_start_time)-sim_time):.2f}s{bcolors.ENDC}")
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    save_csv_sumo(data, file_prefix='sumoSim_log', csv_header=csv_header)


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