#! /usr/bin/env python3

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _network_message_logger import NetworkMessageLogger
from _network_degradation import NetworkDegradationLayer  # import logger from model simulation (Soumil)
from _constants import *
import time

from collections import deque

# import classes
from SumoSim import SumoSim
from x2v_constants import *

# logging utils
from utils_logging import *

logRunning_ = False
fileNameTemp = 'sumoSim_v2x_logRuntime' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.csv'
data = np.zeros([int(END_TIME / SIM_STEP) + 1, len(csv_header)])

# Run params
vizTraj = False
guiSumo = True
live_plt = False

# =========================
# DoS-as-latency on lead info (nv0 -> nv1)
# =========================
BOOL_ATTACK_LEAD_INFO = True
ATTACK_START_TIME = STALLTIME
LEAD_INFO_DELAY = 3
lead_state_buffer = deque()

# =========================
# Network degradation layer settings
# =========================
ENABLE_NETWORK_DEGRADATION = False

# Attack window for degrading nv0 -> nv1 messages
NETWORK_ATTACK_START = STALLTIME
NETWORK_ATTACK_END = STALLTIME + 5.0

# These are model knobs we will tune later
NETWORK_BASE_DELAY_SECONDS = 0.0
NETWORK_ATTACK_DELAY_SECONDS = 1.0
NETWORK_ATTACK_JITTER_SECONDS = 0.2
NETWORK_ATTACK_DROP_PROBABILITY = 0.2



if __name__ == "__main__":

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

    # Data buffers for live plotting
    times = []
    dist0, dist1 = [], []
    spd0, spd1 = [], []
    dist2, spd2 = [], []

    # init speed profiles
    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3]) + 20.0

    # Init SUMO sim
    sumo_sim_manager = SumoSim(
        sumo_config_name=parent_dir + "/sumo/v2x/" + "v2x_2veh.sumocfg"
    )
    sumo_sim_manager.start_Sumo(gui=guiSumo)

    if live_plt:
        # Prepare live plotting
        plt.ion()
        fig, axs = plt.subplots(2, 1, figsize=(8, 6))

        # Distance subplot with reference
        line_full_ref_s, = axs[0].plot(record_t, front_s_t, 'r:', label='US06 Ref s')
        line_dist0, = axs[0].plot([], [], 'k-', label='Leading Vehicle')
        line_dist1, = axs[0].plot([], [], 'b--', label='nv1')
        line_ref_s, = axs[0].plot([], [], 'k*', label='Cycle Ref s')
        axs[0].set_ylabel('Distance [m]')
        axs[0].legend()

        # Speed subplot with reference
        line_full_ref_v, = axs[1].plot(record_t, front_v_t, 'r:', label='US06 Ref v')
        line_spd0, = axs[1].plot([], [], 'k-', label='Leading Vehicle')
        line_spd1, = axs[1].plot([], [], 'b--', label='nv1')
        line_ref_v, = axs[1].plot([], [], 'k*', label='Cycle Ref v')
        axs[1].set_ylabel('Speed [m/s]')
        axs[1].legend()

    if guiSumo:
        traci.gui.trackVehicle("View #0", "nv1")
        traci.gui.setZoom("View #0", 500)

    # Init and setup controller
    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')

    # Start simulation
    real_start_time = time.monotonic()
    real_now = real_start_time

    sim_start_time = 0  # SUMO's starting simulation time

    # Network coupling baseline:
    # Logs clean SUMO vehicle states as network-style messages.
    network_logger = NetworkMessageLogger(
        "networksim/results/sumo_message_schedule.csv"
    )

    network_layer = NetworkDegradationLayer(
        enabled=ENABLE_NETWORK_DEGRADATION,
        attack_start=NETWORK_ATTACK_START,
        attack_end=NETWORK_ATTACK_END,
        base_delay_seconds=NETWORK_BASE_DELAY_SECONDS,
        attack_delay_seconds=NETWORK_ATTACK_DELAY_SECONDS,
        attack_jitter_seconds=NETWORK_ATTACK_JITTER_SECONDS,
        attack_drop_probability=NETWORK_ATTACK_DROP_PROBABILITY,
        trace_path="networksim/results/network_degradation_trace.csv",
        seed=1,
    )

    latest_network_nv0_state = None
    network_msg_id = 0

    while sumo_sim_manager.step < END_TIME / SIM_STEP:

        sim_time = traci.simulation.getTime()

        if sim_time % 10 == 0:
            print(f"sim time: {sim_time}")

        # Calculate expected real-time equivalent for SUMO's sim_time
        real_expected_time = real_start_time + (sim_time - sim_start_time)

        # Step SUMO forward
        sumo_sim_manager.simulationStepForward()

        # Get all vehicles currently in sim
        vehicle_list = traci.vehicle.getIDList()

        if sim_time < 2 * SIM_STEP:
            for veh in vehicle_list:
                traci.vehicle.setSpeed(veh, 0.0)
                traci.vehicle.setMinGap(veh, 0.001)
                traci.vehicle.setSpeedMode(veh, 96)
                traci.vehicle.setLength(veh, 3.2)
                traci.vehicle.setDecel(veh, 8.0)
                traci.vehicle.setAccel(veh, 8.0)
                traci.vehicle.setEmergencyDecel(veh, 8.0)
            continue

        # Assign speeds to leading vehicle
        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]

        if sim_time < STALLTIME:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID="nv0", tgt_spd=v_tgt_lead)
        else:
            traci.vehicle.setSpeedMode("nv1", 96)
            traci.vehicle.setSpeedMode("nv0", 96)
            traci.vehicle.setSpeed("nv0", 0.0)
            traci.vehicle.setAcceleration("nv0", -8.0, 1.0)

        # Get vehicle states
        veh_states_matrix = [
            sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5)
            for veh in vehicle_list
        ]

        if sim_time > STALLTIME:
            veh_states_matrix[1][2] = 5.0
            veh_states_matrix[1][1] = 1.0

        # Network coupling + degradation layer:
        # Treat nv0 lead vehicle state as a status message sent to nv1.
        state_by_id = {
            veh: state for veh, state in zip(vehicle_list, veh_states_matrix)
        }

        if "nv0" in state_by_id and "nv1" in state_by_id:
            nv0_state = list(state_by_id["nv0"])

            network_logger.log_status_message(
                send_time=sim_time,
                sender_id="nv0",
                receiver_id="nv1",
                position_x=nv0_state[3],
                position_y=0.0,
                speed=nv0_state[2],
                acceleration=nv0_state[1],
                packet_size_bytes=300,
            )

            clean_msg = {
                "message_id": network_msg_id,
                "send_time": sim_time,
                "sender_id": "nv0",
                "receiver_id": "nv1",
                "message_type": "status",
                "packet_size_bytes": 300,
                "state": nv0_state,
            }

            network_layer.send(clean_msg, sim_time)
            network_msg_id += 1

        delivered_messages = network_layer.receive_available(sim_time)

        if delivered_messages:
            latest_network_nv0_state = delivered_messages[-1]["state"]

        # Replace the clean nv0 state with the latest network-delivered nv0 state.
        # If messages are delayed or dropped, this causes nv1's controller to use stale nv0 data.
        if ENABLE_NETWORK_DEGRADATION and latest_network_nv0_state is not None and "nv0" in state_by_id:
            state_by_id["nv0"] = list(latest_network_nv0_state)
            veh_states_matrix = [state_by_id[veh] for veh in vehicle_list]


        # SOLVE CONTROL
        start_t = time.time()

        if USING_ONLINE_MPC:
            acc, preds_s, preds_v, cycle_ss, cycle_vs = traffic_online_MPC_control_step_nVeh(
                veh_states_matrix,
                sim_t=sim_time,
                record_t=record_t,
                front_v_t=front_v_t,
                online_MPC_control=online_MPC_control,
                simStep=SIM_STEP,
                mpc_dt=MPC_DT,
                mpc_ref_stages=MPC_REF_STAGES,
                cycle_dt=REF_CYCLE_DT,
                cycle_stages=REF_CYCLE_STAGES,
                PassIntention=BOOL_USE_FRONT_PREVIEW,
                outputUsedCycleforFront=True,
                verbose=True
            )
        else:
            acc = {}
            for veh in vehicle_list:
                acc[veh] = 0.0

        runtime_record.append(time.time() - start_t)

        # viz traj
        if guiSumo and vizTraj:
            sumo_sim_manager.add_traj_leader(
                "nv0",
                veh_states_matrix[0][3],
                record_t=record_t,
                front_v_t=front_v_t,
                sim_t=sim_time,
                pred_dt=MPC_DT,
                mpc_ref_stages=MPC_REF_STAGES,
                colorChoice=(255, 255, 100),
                fill=False,
                layer=3,
            )

            sumo_sim_manager.add_traj(
                "nv1",
                preds_s=preds_s["nv1"],
                colorChoice=(0, 255, 2, 100),
                fill=False,
                layer=4,
            )

        sumo_sim_manager.assignAcceleration(
            vehicle_ID="nv1",
            tgt_acc=acc['nv1'],
            dt=SUMO_ACC_INTEGRATE_DT
        )

        if live_plt:
            # Collect data for plotting
            times.append(sim_time)
            dist0.append(veh_states_matrix[0][3])
            dist1.append(veh_states_matrix[1][3])
            spd0.append(veh_states_matrix[0][2])
            spd1.append(veh_states_matrix[1][2])

            # Update live plot
            line_dist0.set_data(times, dist0)
            line_dist1.set_data(times, dist1)

            ref_times = np.arange(REF_CYCLE_STAGES) * REF_CYCLE_DT + sim_time
            line_ref_s.set_data(ref_times, cycle_ss)
            axs[0].relim()
            axs[0].autoscale_view()

            line_spd0.set_data(times, spd0)
            line_spd1.set_data(times, spd1)
            line_ref_v.set_data(ref_times, cycle_vs)
            axs[1].relim()
            axs[1].autoscale_view()

            fig.canvas.draw()
            fig.canvas.flush_events()

        veh_0_acc.append(veh_states_matrix[0][1])
        veh_0_spd.append(veh_states_matrix[0][2])
        veh_0_dist.append(veh_states_matrix[0][3])

        veh_1_acc.append(veh_states_matrix[1][1])
        veh_1_spd.append(veh_states_matrix[1][2])
        veh_1_dist.append(veh_states_matrix[1][3])
        mache_accCmd.append(acc['nv1'])

        veh_sim_t.append(sim_time)

        data[sumo_sim_manager.step, :] = ([
            real_now - real_start_time,
            sim_time,
            sim_time,
            time.time() - start_t,
            veh_states_matrix[0][3],
            0.0,
            veh_states_matrix[0][2],
            veh_states_matrix[0][1],
            veh_states_matrix[1][3],
            0.0,
            veh_states_matrix[1][2],
            veh_states_matrix[1][1],
            acc['nv1'],
        ])

        if logRunning_:
            with open(fileNameTemp, "a", newline="") as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(data[sumo_sim_manager.step, :])

    network_logger.close()
    network_layer.close()

    if live_plt:
        plt.ioff()

    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    save_csv_sumo(data, file_prefix='sumoSim_log', csv_header=csv_header)

    plt.figure(2)

    plt.subplot(3, 1, 1)
    plt.plot(record_t, front_s_t, 'r:')
    plt.plot(veh_sim_t, veh_0_dist, 'k--')
    plt.plot(veh_sim_t, veh_1_dist, 'b--')
    plt.xlabel('Time [s]')
    plt.ylabel('Distance from route edge [m]')
    plt.legend(['US06 Ref', 'Leading Vehicle', 'mache'])

    plt.subplot(3, 1, 2)
    plt.plot(record_t, front_v_t, 'r:')
    plt.plot(veh_sim_t, veh_0_spd, 'k--')
    plt.plot(veh_sim_t, veh_1_spd, 'b--')
    plt.xlabel('Time [s]')
    plt.ylabel('Speed [m/s]')
    plt.legend(['US06 Ref', 'Leading Vehicle', 'mache'])

    plt.subplot(3, 1, 3)
    plt.plot(veh_sim_t, veh_0_acc, 'k--')
    plt.plot(veh_sim_t, veh_1_acc, 'b--')
    plt.plot(veh_sim_t, mache_accCmd, 'g--')
    plt.xlabel('Time [s]')
    plt.ylabel('Acc [m/s^2]')
    plt.legend(['Leading Vehicle', 'mache', 'mache_accCmd'])

    plt.savefig('sumoSim_' + datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + '.png')

    plt.close('all')  # avoid blocking terminal after saving plot

    traci.close(False)
