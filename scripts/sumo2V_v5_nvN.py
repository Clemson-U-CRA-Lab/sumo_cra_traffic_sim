#! /usr/bin/env python3

##########

# Script to run generic sumocfg with multiple vehicles. 
# Need to define the ID fo the real CAV used in the experiment.
# Stremalined saving of data in the correct order in CSV/PNG


##########
import os
import re
import sys
import csv
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _constants import *
import time
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np
import yaml

# import classes
from SumoSim import SumoSim
from x2v_constants import *


# logging utils
from  utils_logging import *
logRunning_ = False
if logRunning_:
    fileNameTemp = 'sumoSim_v2x_logRuntime' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'


#----------------------------------------------------------
# Comms
asyncSocket = True
# interface = 'periodicInterface' # 'latency'
# interface = 'periodicInterface' # 'periodicInterface', 'periodic_sendDelay'
interface = 'periodic_sendDelay' # 'periodicInterface', 'periodic_sendDelay'

# Run params
testWithoutGPS = BOOL_TEST_WITHOUT_GPS
verbosity = False
mpc_verbose = False
guiSumo = False
showPlot = False

vizTraj = False
# None / [] = all vehicles; e.g. ["nv1"] or ["nv0", "nv2"] for selected only
vizTrajVehs = ["nv0", "nv2"]

# ID of the real cav for VIL sim
REAL_CAV_ID = "nv1" # must be a non-leader vehicle id from the SUMO route file

# Fro automated tests only
automate_test = False

#----------------------------------------------------------


if automate_test:
    automationYamlPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "x2v_automate.yaml")
    with open(automationYamlPath, "r") as automationYamlFile:
        automationYaml = yaml.safe_load(automationYamlFile) or {}
    END_TIME = float(automationYaml.get("END_TIME", END_TIME))
    BOOL_ATTACK = automationYaml.get("BOOL_ATTACK", BOOL_ATTACK)
    ATTACK_START_TIME = float(automationYaml.get("ATTACK_START_TIME"))
    # ATTACK_END_TIME = float(automationYaml.get("ATTACK_END_TIME", ATTACK_START_TIME + 15.0))
    ATTACK_ACTIVE = automationYaml.get("ATTACK_ACTIVE")
    DELAY_SECONDS = float(automationYaml.get("DELAY_SECONDS"))



if asyncSocket:
    if interface == 'periodicInterface':
        from x2vSocketInterface_Udp_periodic import x2vSocketInterfaceUdpAsync as x2vSocketInterface
    elif interface == 'periodic_sendDelay':
        from x2vSocketInterface_Udp_periodic_sendDelay import x2vSocketInterfaceUdpAsync as x2vSocketInterface
    else:
        raise ValueError("Invalid interface type. Choose 'periodicInterface' or 'periodic_sendDelay'.")
else:
    from x2vSocketInterface import x2vSocketInterface as x2vSocketInterface


def natural_sort_key(vehicle_id):
    return [int(text) if text.isdigit() else text for text in re.split(r'(\d+)', vehicle_id)]


def get_configured_vehicle_order(sumo_config_name):
    try:
        config_root = ET.parse(sumo_config_name).getroot()
    except (ET.ParseError, OSError) as exc:
        print(f"{bcolors.WARNING}Could not read SUMO config vehicle order: {exc}{bcolors.ENDC}")
        return []

    route_files = []
    for route_files_elem in config_root.findall(".//route-files"):
        route_files.extend(
            value for value in re.split(r"[\s,]+", route_files_elem.get("value", "")) if value
        )

    vehicle_order = []
    config_dir = os.path.dirname(sumo_config_name)
    for route_file in route_files:
        route_path = route_file if os.path.isabs(route_file) else os.path.join(config_dir, route_file)
        try:
            route_root = ET.parse(route_path).getroot()
        except (ET.ParseError, OSError) as exc:
            print(f"{bcolors.WARNING}Could not read route file {route_path}: {exc}{bcolors.ENDC}")
            continue

        for vehicle_elem in route_root.findall(".//vehicle"):
            vehicle_id = vehicle_elem.get("id")
            if vehicle_id:
                vehicle_order.append(vehicle_id)

    return vehicle_order


def ordered_vehicle_ids(vehicle_ids, configured_order=None):
    if configured_order:
        configured_index = {veh: i for i, veh in enumerate(configured_order)}
        return sorted(
            vehicle_ids,
            key=lambda veh: (configured_index.get(veh, len(configured_index)), natural_sort_key(veh)),
        )
    return sorted(vehicle_ids, key=natural_sort_key)


def fit_horizon(values, stages, fill_value):
    values = list(values) if values is not None else []
    if len(values) >= stages:
        return values[:stages]
    if values:
        fill_value = values[-1]
    return values + [fill_value] * (stages - len(values))


def reuse_siminfo(sim_array, delay_amount, fallback_mode,
                  ref_cycle_stages=MPC_REF_STAGES, mpc_dt=MPC_DT):
    """Shift/extrapolate stale front preview (same as vehicle fallback dummy)."""
    shift = round(delay_amount / mpc_dt)
    sim = list(sim_array)
    pv_s = sim[7 + 1 + shift]
    pv_spd = sim[7 + ref_cycle_stages + 1 + shift]
    pv_acc = sim[6]
    front_pred_s = np.roll(np.asarray(sim[7:7 + ref_cycle_stages], dtype=float), -shift)
    front_pred_v = np.roll(
        np.asarray(sim[7 + ref_cycle_stages:7 + 2 * ref_cycle_stages], dtype=float),
        -shift,
    )

    if fallback_mode == "stop":
        for i in range(1, shift + 1):
            front_pred_s[-i] = front_pred_s[ref_cycle_stages - shift - 1]
            front_pred_v[-i] = 0.0
    elif fallback_mode == "carry":
        for i in range(1, shift + 1):
            front_pred_v[-i] = front_pred_v[ref_cycle_stages - shift - 1]
        for i in range(shift, 0, -1):
            front_pred_s[ref_cycle_stages - i] = (
                front_pred_s[ref_cycle_stages - i - 1]
                + front_pred_v[ref_cycle_stages - i - 1] * mpc_dt
            )
    elif fallback_mode == "carryAcc":
        for i in range(shift, 0, -1):
            front_pred_v[ref_cycle_stages - i] = max(
                0.0,
                front_pred_v[ref_cycle_stages - i - 1] + pv_acc * mpc_dt,
            )
            front_pred_s[ref_cycle_stages - i] = (
                front_pred_s[ref_cycle_stages - i - 1]
                + front_pred_v[ref_cycle_stages - i - 1] * mpc_dt
            )
    else:
        raise ValueError(f"unknown fallback mode: {fallback_mode}")

    return pv_s, pv_spd, pv_acc, front_pred_s, front_pred_v, shift


def build_real_cav_payload(sim_time, ego_state, front_state, front_id, leader_id,
                           preds_s, preds_v, cycle_ss, cycle_vs):
    if front_id == leader_id:
        if sim_time >= STALLTIME:
            front_pred_s = [front_state[3]] * REF_CYCLE_STAGES
            front_pred_v = [0.0] * REF_CYCLE_STAGES
        else:
            front_pred_s = cycle_ss
            front_pred_v = cycle_vs
    else:
        front_pred_s = preds_s.get(front_id)
        front_pred_v = preds_v.get(front_id)

    front_pred_s = fit_horizon(front_pred_s, REF_CYCLE_STAGES, front_state[3])
    front_pred_v = fit_horizon(front_pred_v, REF_CYCLE_STAGES, front_state[2])

    # sim_time, ego_s, ego_v, ego_a, front_s, front_v, front_a, front_s_preview, front_v_preview
    return [
        sim_time,
        ego_state[3], ego_state[2], ego_state[1],
        front_state[3], front_state[2], front_state[1],
    ] + front_pred_s + front_pred_v


def add_vehicle_log_slots(veh_data, vehicle_ids, row_count):
    for veh in vehicle_ids:
        if veh not in veh_data:
            veh_data[veh] = {
                'dist': [np.nan] * row_count,
                'spd': [np.nan] * row_count,
                'lane': [np.nan] * row_count,
                'acc': [np.nan] * row_count,
                'accCmd': [np.nan] * row_count,
            }


def build_nv1_style_csv_header(vehicle_ids):
    header = ["Realtime [sec]", "Sim Time [sec]", "Sim Time from MPC [sec]", "MPC runtime"]
    for veh_idx, _ in enumerate(vehicle_ids):
        header += [
            f"v{veh_idx}_dist [m]",
            f"v{veh_idx}_lane [-]",
            f"v{veh_idx}_spd [m/s]",
            f"v{veh_idx}_acc [m/s2]",
        ]
        if veh_idx > 0:
            header.append(f"v{veh_idx}_accCmd [m/s2]")
    return header


def save_all_veh_data_nv1_format(veh_data, veh_sim_t, real_time_list,
                                 mpc_time_list, runtime_list, file_prefix):
    vehicle_ids = list(veh_data.keys())
    rows = []
    for i, sim_time in enumerate(veh_sim_t):
        row = [
            real_time_list[i] if i < len(real_time_list) else np.nan,
            sim_time,
            mpc_time_list[i] if i < len(mpc_time_list) else np.nan,
            runtime_list[i] if i < len(runtime_list) else np.nan,
        ]
        for veh_idx, veh in enumerate(vehicle_ids):
            row += [
                veh_data[veh]['dist'][i] if i < len(veh_data[veh]['dist']) else np.nan,
                veh_data[veh]['lane'][i] if i < len(veh_data[veh]['lane']) else np.nan,
                veh_data[veh]['spd'][i] if i < len(veh_data[veh]['spd']) else np.nan,
                veh_data[veh]['acc'][i] if i < len(veh_data[veh]['acc']) else np.nan,
            ]
            if veh_idx > 0:
                row.append(
                    veh_data[veh]['accCmd'][i] if i < len(veh_data[veh]['accCmd']) else np.nan
                )
        rows.append(row)

    save_csv_sumo(rows, file_prefix=file_prefix, csv_header=build_nv1_style_csv_header(vehicle_ids))


def write_runtime_row(file_name, vehicle_ids, veh_states_dict, sim_time,
                      real_elapsed, real_cav_array, runtime_s, acc_cmds):
    header = build_nv1_style_csv_header(vehicle_ids)
    row = [
        real_elapsed,
        sim_time,
        real_cav_array[0] if real_cav_array is not None else np.nan,
        runtime_s,
    ]
    for veh_idx, veh in enumerate(vehicle_ids):
        state = veh_states_dict.get(veh)
        if state is None:
            row += [np.nan, np.nan, np.nan, np.nan]
        else:
            row += [state[3], state[4], state[2], state[1]]
        if veh_idx > 0:
            row.append(acc_cmds.get(veh, np.nan))

    write_header = not os.path.exists(file_name) or os.path.getsize(file_name) == 0
    with open(file_name, "a", newline="") as csv_file:
        csv_writer = csv.writer(csv_file)
        if write_header:
            csv_writer.writerow(header)
        csv_writer.writerow(row)


if __name__=="__main__":

    # Init socket connections
    sockInt = x2vSocketInterface()
    if BOOL_ATTACK and interface != 'periodic_sendDelay':
        print(f"{bcolors.WARNING}BOOL_ATTACK is enabled, but send delay requires interface='periodic_sendDelay'.{bcolors.ENDC}")

    veh_data = {}
    veh_sim_t = []
    veh_real_t = []
    veh_mpc_t = []
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
    sumo_config_path = parent_dir + "/sumo/v2x/" + SUMO_CONFIG
    configured_vehicle_order = get_configured_vehicle_order(sumo_config_path)
    add_vehicle_log_slots(veh_data, configured_vehicle_order, len(veh_sim_t))
    configured_leader_id = configured_vehicle_order[0] if configured_vehicle_order else None
    configured_real_cav_id = REAL_CAV_ID or (configured_vehicle_order[-1] if configured_vehicle_order else None)
    if configured_real_cav_id == configured_leader_id:
        raise ValueError(f"Real CAV must have a front vehicle; {configured_real_cav_id} is the leader.")
    if REAL_CAV_ID and configured_vehicle_order and REAL_CAV_ID not in configured_vehicle_order:
        raise ValueError(f"REAL_CAV_ID={REAL_CAV_ID} is not in SUMO route vehicle order {configured_vehicle_order}.")
    real_cav_seen = False
    sumo_sim_manager = SumoSim(sumo_config_name=sumo_config_path)
    sumo_sim_manager.start_Sumo(gui=guiSumo)
    tracked_vehicle = False
    
    # Inti and Setup controller
    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')
        
    print(f"{bcolors.OKGREEN}BOOL_USE_FRONT_PREVIEW={BOOL_USE_FRONT_PREVIEW}{bcolors.ENDC}")
    print(f"{bcolors.OKGREEN}BOOL_TEST_WITHOUT_GPS={BOOL_TEST_WITHOUT_GPS}{bcolors.ENDC}")
    print(f"{bcolors.OKGREEN}USE_FALLBACK={USE_FALLBACK} FALLBACK_MODE={FALLBACK_MODE} DELAY_THRESHOLD={DELAY_THRESHOLD}{bcolors.ENDC}")
    print(f"{bcolors.OKGREEN}Using sumo config: {SUMO_CONFIG}{bcolors.ENDC}")

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
        vehicle_list = ordered_vehicle_ids(traci.vehicle.getIDList(), configured_vehicle_order)

        if not vehicle_list:
            if asyncSocket:
                next_deadline += SIM_STEP
                sleep_time = next_deadline - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            continue

        if configured_real_cav_id and real_cav_seen and configured_real_cav_id not in vehicle_list:
            break

        leader_id = configured_leader_id if configured_leader_id in vehicle_list else vehicle_list[0]
        if configured_real_cav_id:
            real_cav_id = configured_real_cav_id if configured_real_cav_id in vehicle_list else None
            real_cav_seen = real_cav_seen or real_cav_id is not None
        else:
            real_cav_id = vehicle_list[-1]

        if guiSumo and not tracked_vehicle and real_cav_id is not None:
            traci.gui.trackVehicle("View #0", real_cav_id)
            traci.gui.setZoom("View #0", 600)
            tracked_vehicle = True

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

        add_vehicle_log_slots(veh_data, vehicle_list, len(veh_sim_t))

        # Assign speeds to leading vehicle
        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]
        if sim_time < STALLTIME:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID=leader_id, tgt_spd=v_tgt_lead)
        else:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID=leader_id, tgt_spd=0)

        # Get vehicle states
        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]
        veh_states_dict = {state[0]: state for state in veh_states_matrix}

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
                                                       verbose=mpc_verbose
                                                       )
        else:
            acc = {veh: 0.0 for veh in vehicle_list}
            preds_s = {}
            preds_v = {}
            cycle_ss = [veh_states_matrix[0][3]] * REF_CYCLE_STAGES
            cycle_vs = [0.0] * REF_CYCLE_STAGES
           
        real_cav_idx = vehicle_list.index(real_cav_id) if real_cav_id in vehicle_list else None
        if real_cav_idx is not None and real_cav_idx > 0:
            front_id = vehicle_list[real_cav_idx - 1]
            sim_nv_array = build_real_cav_payload(
                sim_time=sim_time,
                ego_state=veh_states_dict[real_cav_id],
                front_state=veh_states_dict[front_id],
                front_id=front_id,
                leader_id=leader_id,
                preds_s=preds_s,
                preds_v=preds_v,
                cycle_ss=cycle_ss,
                cycle_vs=cycle_vs,
            )
        else:
            sim_nv_array = None

        if sim_time > ATTACK_START_TIME and BOOL_ATTACK and not ATTACK_ACTIVE:
            print(f"{bcolors.FAIL_RED}***** Starting Attack! *****{bcolors.ENDC}")
            if hasattr(sockInt, "send_delay_sec"):
                sockInt.send_delay_sec = DELAY_SECONDS
            if hasattr(sockInt, "recv_delay_sec"):
                sockInt.recv_delay_sec = DELAY_SECONDS*0
            ATTACK_ACTIVE = True


              #########
      
        # Send NV states to realCAV
        if interface in ('periodicInterface', 'periodic_sendDelay'):
            with sockInt.simData_lock:
                sockInt.latest_sim_data = sim_nv_array
        else:
            raise ValueError("Invalid interface type. Choose 'periodicInterface' or 'periodic_sendDelay'.")

            #########

        realCavArray = sockInt.get_veh_info()

        # solve trailing vehicles' MPC
        if (USING_ONLINE_MPC and BOOL_USE_FRONT_PREVIEW and realCavArray is not None
                and real_cav_idx is not None):
            pred_s = list(realCavArray[7:7 + MPC_REF_STAGES])
            pred_v = list(realCavArray[7 + MPC_REF_STAGES:7 + 2 * MPC_REF_STAGES])
            preds_s[real_cav_id], preds_v[real_cav_id] = pred_s, pred_v
            delay_amount = sim_time - realCavArray[0]
            apply_fb = USE_FALLBACK and delay_amount >= DELAY_THRESHOLD
            for i in range(real_cav_idx + 1, len(vehicle_list)):
                ego = veh_states_matrix[i]
                front = veh_states_matrix[i - 1]
                pv_s, pv_v, pv_a = front[3], front[2], front[1]
                cycle_ss, cycle_vs = pred_s, pred_v
                # First trailing veh: front intention is stale realCAV reply → fallback
                if apply_fb and i == real_cav_idx + 1:
                    # print(f"{bcolors.WARNING}***** Falling back to {FALLBACK_MODE} mode *****{bcolors.ENDC}")
                    sim_arr = [0.0] * 7 + list(cycle_ss) + list(cycle_vs)
                    sim_arr[6] = pv_a
                    pv_s, pv_v, pv_a, cycle_ss, cycle_vs, _ = reuse_siminfo(
                        sim_arr, delay_amount, FALLBACK_MODE)
                pred_s, pred_v, cmd, _ = online_MPC_control.svs.setCommand_SUMO(
                    t=sim_time, ego_s=ego[3], ego_v=ego[2], ego_a=ego[1],
                    pv_s=pv_s, pv_v=pv_v, pv_a=pv_a,
                    cycle_ss=cycle_ss, cycle_vs=cycle_vs, cycle_dt=MPC_DT,
                    n_refs=MPC_REF_STAGES, preview=BOOL_USE_FRONT_PREVIEW)
                preds_s[ego[0]], preds_v[ego[0]], acc[ego[0]] = pred_s, pred_v, cmd


        # if verbose, print the preds_s and preds_v for all vehicles
        if verbosity:
            for veh in vehicle_list:
                if veh in preds_s:
                    print(f"{bcolors.OKCYAN}Preds_s for {veh}: {[round(s, 1) for s in preds_s[veh]]}{bcolors.ENDC}")
                    print(f"{bcolors.OKCYAN}Preds_v for {veh}: {[round(v, 1) for v in preds_v.get(veh, [])]}{bcolors.ENDC}")

        # viz traj after all preds are available (incl. real CAV + trailing regen)
        if guiSumo and vizTraj:
            show_all_traj = not vizTrajVehs
            if show_all_traj or leader_id in vizTrajVehs:
                r, g, b, _ = traci.vehicle.getColor(leader_id)
                sumo_sim_manager.add_traj_leader(leader_id,
                                                veh_states_matrix[0][3],
                                                record_t=record_t,
                                                front_v_t=front_v_t,
                                                sim_t=sim_time,
                                                pred_dt=MPC_DT, mpc_ref_stages=MPC_REF_STAGES,
                                                colorChoice=(r, g, b, 100), fill=False, layer=3)
            for i, veh in enumerate(vehicle_list[1:]):
                if veh in preds_s and (show_all_traj or veh in vizTrajVehs):
                    r, g, b, _ = traci.vehicle.getColor(veh)
                    sumo_sim_manager.add_traj(
                        veh,
                        preds_s=preds_s[veh],
                        colorChoice=(r, g, b, 100),
                        fill=False,
                        layer=4 + i,
                    )

        runtime_record.append(time.time() - start_t)

        for veh in vehicle_list[1:]:
            if veh != real_cav_id:
                sumo_sim_manager.assignAcceleration(
                    vehicle_ID=veh, tgt_acc=acc.get(veh, 0.0), dt=SUMO_ACC_INTEGRATE_DT)

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

        if realCavArray is not None and real_cav_id is not None:
            if verbosity:
                print(f"{bcolors.OKCYAN}==============Got from VEH============{bcolors.ENDC}" )
                print(f"{bcolors.OKCYAN}Ego x,y: {realCavArray[4]:.2f}, {realCavArray[5]:.2f}.{bcolors.ENDC}" )
                print(f"{bcolors.OKCYAN}Ego [GPS] s: -- , v:{realCavArray[2]:.2f}.{bcolors.ENDC}" )
                print(f"{bcolors.OKCYAN}Ego MpcCmd: {realCavArray[6]:.2f}.{bcolors.ENDC}" )

            # collided = True if veh_states_matrix[0][3] - veh_states_matrix[1][3] < 3.2 else False

            print(f"{bcolors.OKBLUE}Sim Time: {sim_time:.2f} | {bcolors.OKBLUE}Vehicle's SimTime: {realCavArray[0]:.3f} |  {bcolors.OKGREEN}Delta RTT: {sim_time-realCavArray[0]:.2f}.{bcolors.ENDC}" )


            # Update Real CAV pos in simulation:         Z
            if testWithoutGPS:
                # if local testing w/o gps:
                sumo_sim_manager.assignAcceleration(vehicle_ID=real_cav_id, tgt_acc=realCavArray[6], dt=SUMO_ACC_INTEGRATE_DT) # careful: assign commmand or real sensed acc?

            else:
                # if testing with gps and vehicle run
                sumo_sim_manager.update_CAV_in_sumo(veh=real_cav_id,
                                                        spd=realCavArray[2],
                                                        pos=[realCavArray[4],realCavArray[5]]
                                                        )

        acc_cmds = {veh: acc.get(veh, 0.0) for veh in vehicle_list}
        if realCavArray is not None and real_cav_id is not None:
            acc_cmds[real_cav_id] = realCavArray[6]
        elif real_cav_id is not None:
            acc_cmds[real_cav_id] = np.nan

        active_vehicle_ids = set(vehicle_list)
        for veh in veh_data:
            if veh in active_vehicle_ids:
                veh_data[veh]['acc'].append(veh_states_dict[veh][1])
                veh_data[veh]['spd'].append(veh_states_dict[veh][2])
                veh_data[veh]['dist'].append(veh_states_dict[veh][3])
                veh_data[veh]['lane'].append(veh_states_dict[veh][4])
            else:
                veh_data[veh]['acc'].append(np.nan)
                veh_data[veh]['spd'].append(np.nan)
                veh_data[veh]['dist'].append(np.nan)
                veh_data[veh]['lane'].append(np.nan)
            veh_data[veh]['accCmd'].append(acc_cmds.get(veh, np.nan))

        veh_sim_t.append(sim_time)
        veh_mpc_t.append(realCavArray[0] if realCavArray is not None else np.nan)
        real_now = time.monotonic()
        real_elapsed = real_now - real_start_time

        if logRunning_:
            write_runtime_row(
                fileNameTemp,
                list(veh_data.keys()),
                veh_states_dict,
                sim_time,
                real_elapsed,
                realCavArray,
                runtime_record[-1],
                acc_cmds,
            )

        # Sleep timing
        veh_real_t.append(real_elapsed)

        ## Fixed rate scheduling.
        if asyncSocket:
            next_deadline += SIM_STEP         # fixed cadence
            sleep_time = next_deadline - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

        # kill cleanly if vehicles out of the road.
        if real_cav_id is not None and veh_data.get(real_cav_id, {}).get('dist') and veh_data[real_cav_id]['dist'][-1] >= 210:
            print(f"{bcolors.FAIL_RED}***** Real CAV has reached the end of the road. Stopping simulation. *****{bcolors.ENDC}")
            break



    avg_runtime_ms = round(np.mean(runtime_record) * 1000, 4) if runtime_record else 0.0
    print('Average runtime is: ', str(avg_runtime_ms), 'ms')
    if testWithoutGPS:
        prefix='sumIndoorVIL_log_'
    else:
        prefix='sumo_log_'
    save_all_veh_data_nv1_format(
        veh_data,
        veh_sim_t,
        veh_real_t,
        veh_mpc_t,
        runtime_record,
        file_prefix=prefix,
    )

    plot_veh_data(
        veh_data,
        veh_sim_t,
        record_t=record_t,
        front_s_t=front_s_t,
        front_v_t=front_v_t,
        file_prefix=prefix,
        show_plot=showPlot,
    )

    traci.close(True)
