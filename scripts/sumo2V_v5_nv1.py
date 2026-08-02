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
data = np.zeros([int(END_TIME/SIM_STEP)+1,len(csv_header)])
logRunning_ = False
if logRunning_:
    fileNameTemp = 'sumoSim_v2x_logRuntime' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'

# Comms
asyncSocket = True
# interface = 'periodicInterface' # 'latency'
interface = 'periodicInterface' # 'latency', 'naiveAsync', 'hybrid', 'periodicInterface', 'periodic_sendDelay'


# Run params
guiSumo = True
vizTraj = False
testWithoutGPS = BOOL_TEST_WITHOUT_GPS
verbosity = False


if asyncSocket:
    if interface == 'periodicInterface':
        from x2vSocketInterface_Udp_periodic import x2vSocketInterfaceUdpAsync as x2vSocketInterface
    else:
        raise ValueError("Invalid interface type. Choose 'periodicInterface', 'latency', or 'naiveAsync'.")
else:
    from x2vSocketInterface import x2vSocketInterface as x2vSocketInterface


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
    # Get the real-world start time
    real_start_time = time.monotonic()  
    real_now = real_start_time
    sim_start_time = 0  # SUMO's starting simulation time
    next_deadline = time.monotonic()
    while sumo_sim_manager.step < END_TIME/SIM_STEP:

        sim_time = traci.simulation.getTime()  # Get SUMO's current simulation time
        # Calculate expected real-time equivalent for SUMO's sim_time
        real_expected_time = real_start_time + (sim_time - sim_start_time)

        # Step SUMO forward
        sumo_sim_manager.simulationStepForward()

        # Get all vehicles currently in sim
        vehicle_list = traci.vehicle.getIDList()

        if sim_time < 2*SIM_STEP:
            for veh in vehicle_list:
                traci.vehicle.setSpeed(veh, 0.0)
                traci.vehicle.setMinGap(veh, 0.001) # try to avoid collision
                traci.vehicle.setSpeedMode(veh, 96) # no safety, no auto
                traci.vehicle.setLength(veh, 3.2) # set length
                traci.vehicle.setAccel(veh, 8.0) # set max accel
                traci.vehicle.setDecel(veh, 8.0) # set max decel
                # traci.vehicle.setTau(veh, 0.1) # reaction time
                # These MUST be set after the first step, otherwise SUMO will ignore them.
            continue

        # Assign speeds to leading vehicle
        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]
        if sim_time < STALLTIME:
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
            acc = {}
            for veh in vehicle_list:
                acc[veh] = 0.0
        runtime_record.append(time.time() - start_t)

        # viz traj
        if guiSumo and  vizTraj:
            sumo_sim_manager.add_traj_leader("nv0", 
                                            veh_states_matrix[0][3],
                                            record_t=record_t,
                                            front_v_t=front_v_t,
                                            sim_t=sim_time,
                                            pred_dt=MPC_DT, mpc_ref_stages=MPC_REF_STAGES,
                                            colorChoice=(255,255,100), fill=False, layer=3)
            sumo_sim_manager.add_traj("nv1", preds_s=preds_s["nv1"],colorChoice=(0, 255, 2, 100), fill=False, layer=4)
           
        # Assign the acceleration to leader nv0
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

        if sim_time > ATTACK_START_TIME and BOOL_ATTACK and not ATTACK_ACTIVE:
            print(f"{bcolors.FAIL_RED}***** Starting Attack! *****{bcolors.ENDC}")
            sockInt.send_delay_sec = DELAY_SECONDS
            ATTACK_ACTIVE = True


              #########
      
        # Send NV states to realCAV
        if interface == 'periodicInterface':
            with sockInt.simData_lock:
                sockInt.latest_sim_data = sim_nv_array
        else:
            raise ValueError("Invalid interface type. Choose 'periodicInterface', 'latency', or 'naiveAsync'.")

            #########

        realCavArray = sockInt.get_veh_info()

        if interface == 'hybrid' and sim_time % 1.0 < SIM_STEP:
            stats = sockInt.get_stats()
            print(f"{bcolors.WARNING}[Hybrid Debug] Queue: {stats['queue_len']} | Jitter: {stats['jitter_s']:.3f}s{bcolors.ENDC}")

        if interface == 'latency':
            # === Debug: Observe DoS-induced latency ===
            if sim_time % 1.0 < SIM_STEP:  # Print every ~1 second
                stats = sockInt.get_stats()
                qlen = stats["queue_len"]
                jitter = stats["jitter_s"]
                print(f"{bcolors.WARNING}[Latency Debug] Queue Length: {qlen} | Jitter: {jitter:.3f}s{bcolors.ENDC}")


            #########

        if realCavArray is not None:        
            if verbosity:
                print(f"{bcolors.OKCYAN}==============Got from VEH============{bcolors.ENDC}" )
                # print(f"{bcolors.OKCYAN}Elapsed @ VEH Real: {realCavArray[6]:.2f}, {bcolors.OKBLUE}MPC got SimTime: {realCavArray[0]:.2f}.{bcolors.ENDC}" )
                print(f"{bcolors.OKGREEN}Delta T RSPCSim-VEHReal: {(sim_time-realCavArray[6]):.2f}s{bcolors.ENDC}")
                print(f"{bcolors.OKCYAN}Ego x,y: {realCavArray[4]:.2f}, {realCavArray[5]:.2f}.{bcolors.ENDC}" )
                print(f"{bcolors.OKCYAN}Ego [GPS] s: -- , v:{realCavArray[2]:.2f}.{bcolors.ENDC}" )
                print(f"{bcolors.OKCYAN}Ego MpcCmd: {realCavArray[7]:.2f}.{bcolors.ENDC}" )

            # collided = True if veh_states_matrix[0][3] - veh_states_matrix[1][3] < 3.2 else False

            print(f"{bcolors.OKBLUE}Sim Time: {sim_time:.2f} | {bcolors.OKBLUE}Vehicle's SimTime: {realCavArray[0]:.3f} |  {bcolors.OKGREEN}Delta RTT: {sim_time-realCavArray[0]:.2f}.{bcolors.ENDC}" )


            # Update Real CAV pos in simulation:         
            if testWithoutGPS:
                # print(f"{bcolors.FAIL_RED}Delta MpcCmd: {realCavArray[7]- acc['nv1']:.2f} | {bcolors.OKBLUE}VehCmd: {realCavArray[7]:.3f} |  {bcolors.OKGREEN}ExpectSim: {acc['nv1']:.3f}.{bcolors.ENDC}" )
                # if local testing w/o gps:
                sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=realCavArray[7], dt=SUMO_ACC_INTEGRATE_DT) # careful: assign commmand or real sensed acc?
                # sumo_sim_manager.assignAcceleration(vehicle_ID="nv1", tgt_acc=acc['nv1'], dt=SUMO_ACC_INTEGRATE_DT) # careful: assign commmand or real sensed acc?

                # sumo_sim_manager.update_CAV_in_sumo(veh='nv1', 
                                                        # spd=realCavArray[2]+realCavArray[7]*SUMO_ACC_INTEGRATE_DT)
                                                        # dist = realCavArray[2]*SUMO_ACC_INTEGRATE_DT  + 0.5*realCavArray[7]*SUMO_ACC_INTEGRATE_DT**2)            

            else:
                # if testing with gps and vehicle run
                sumo_sim_manager.update_CAV_in_sumo(veh='nv1', 
                                                        spd=realCavArray[2],
                                                        pos=[realCavArray[4],realCavArray[5]]
                                                        )

            veh_0_acc.append(veh_states_matrix[0][1])
            veh_0_spd.append(veh_states_matrix[0][2])
            veh_0_dist.append(veh_states_matrix[0][3])
            
            veh_1_acc.append(veh_states_matrix[1][1])
            veh_1_spd.append(veh_states_matrix[1][2])
            veh_1_dist.append(veh_states_matrix[1][3])
            mache_accCmd.append(realCavArray[7])
        
            veh_sim_t.append(sim_time)
            data[sumo_sim_manager.step,:] = ([real_now-real_start_time, sim_time, realCavArray[0], runtime_record[-1],
                        veh_states_matrix[0][3],0.0,veh_states_matrix[0][2],veh_states_matrix[0][1],
                        veh_states_matrix[1][3],0.0,veh_states_matrix[1][2],veh_states_matrix[1][1], realCavArray[7]])
            
        if logRunning_:
            with open(fileNameTemp, "a", newline="") as csv_file:
                # Create a CSV writer object
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(data[sumo_sim_manager.step,:])            

        # Sleep timing
        real_now = time.monotonic()
        # if asyncSocket:
        #     sleep_time = max(0, real_expected_time - real_now)  # Sleep only if ahead of real time
        #     time.sleep(sleep_time)  # Sync with real-world time
        # print(f"{bcolors.OKGREEN}Delta T RSPC[Sim-Real]: {((real_now - real_start_time)-sim_time):.2f}s{bcolors.ENDC}")
    
        ## Fixed rate scheduling.
        if asyncSocket:
            next_deadline += SIM_STEP         # fixed cadence
            sleep_time = next_deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        # kill cleanly if vehicles out of the road.
        if veh_1_dist[-1] >= 210:
            break



    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    if testWithoutGPS:
        prefix='sumIndoorVIL_log_'
    else:
        prefix='sumo_log_'
    save_csv_sumo(data, file_prefix=prefix, csv_header=csv_header)

    plt.figure(1)
    
    plt.subplot(3,1,1)
    plt.plot(record_t, front_s_t, 'r:') 
    plt.plot(veh_sim_t, veh_0_dist,'k--')
    plt.plot(veh_sim_t, veh_1_dist,'b--')
    # plt.plot(veh_sim_t, veh_3_dist)
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['US06 Ref', 'Leading Vehicle',  'mache'])
    
    plt.subplot(3,1,2)
    plt.plot(record_t, front_v_t, 'r:') 
    plt.plot(veh_sim_t, veh_0_spd, 'k--')
    plt.plot(veh_sim_t, veh_1_spd, 'b--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['US06 Ref', 'Leading Vehicle','mache'])

    plt.subplot(3,1,3)
    plt.plot(veh_sim_t, veh_0_acc, 'k--')
    plt.plot(veh_sim_t, veh_1_acc,'b--')
    plt.plot(veh_sim_t, mache_accCmd, 'g--')
    # plt.plot(veh_sim_t, veh_3_spd)
    plt.xlabel('Time [s]')
    plt.ylabel('Acc [m/s^2]')
    plt.legend(['Leading Vehicle', 'mache', 'mache_accCmd'])
    
    plt.savefig(prefix+ datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.png')
    plt.show()

    traci.close(True)
