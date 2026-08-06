#! /usr/bin/env python3

import os
import re
import traci
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _constants import *
import time
import xml.etree.ElementTree as ET
from datetime import datetime
import numpy as np

from SumoSim import SumoSim
from x2v_constants import *
from utils_logging import *

# Run params
guiSumo = True
vizTraj = True
# None / [] = all vehicles; e.g. ["nv1"] or ["nv0", "nv2"] for selected only
vizTrajVehs = None
showPlot = True
verbosity = False
# Live debug plot: nv0 ref → nv1 intention → nv2 MPC preds
livePredPlot = False
LIVE_LEADER_ID = "nv0"
LIVE_FRONT_ID = "nv1"
LIVE_EGO_ID = "nv2"


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


class LivePredPlot:
    """Live debug plot: leader ref horizon → front intention → ego MPC preds."""

    def __init__(self, leader_id, front_id, ego_id,
                 pred_dt=MPC_DT, cycle_dt=REF_CYCLE_DT, enabled=True,
                 record_t=None, front_v_t=None, front_s_t=None):
        self.enabled = enabled
        self.leader_id = leader_id
        self.front_id = front_id
        self.ego_id = ego_id
        self.pred_dt = pred_dt
        self.cycle_dt = cycle_dt
        if not enabled:
            return

        plt.ion()
        self.fig, self.axs = plt.subplots(
            3, 1, figsize=(8, 8),
            num=f"{leader_id} ref → {front_id} intention → {ego_id} MPC",
        )

        # Full driving cycle in background
        if record_t is not None and front_s_t is not None:
            self.axs[0].plot(record_t, front_s_t, 'k:', alpha=0.35, linewidth=1.0,
                             label='full cycle s', zorder=0)
        if record_t is not None and front_v_t is not None:
            self.axs[1].plot(record_t, front_v_t, 'k:', alpha=0.35, linewidth=1.0,
                             label='full cycle v', zorder=0)
        if front_s_t is not None and front_v_t is not None:
            n = min(len(front_s_t), len(front_v_t))
            self.axs[2].plot(front_s_t[:n], front_v_t[:n], 'k:', alpha=0.35, linewidth=1.0,
                             label='full cycle v(s)', zorder=0)

        self.line_leader_s, = self.axs[0].plot([], [], 'r*-', label=f'{leader_id} ref s')
        self.line_front_s, = self.axs[0].plot([], [], 'b*-', label=f'{front_id} intention s')
        self.line_ego_s, = self.axs[0].plot([], [], 'g*-', label=f'{ego_id} MPC s')
        self.line_leader_s_now, = self.axs[0].plot([], [], 'ro', markersize=8, label=f'{leader_id} s now')
        self.line_front_s_now, = self.axs[0].plot([], [], 'bo', markersize=8, label=f'{front_id} s now')
        self.line_ego_s_now, = self.axs[0].plot([], [], 'go', markersize=8, label=f'{ego_id} s now')
        self.axs[0].set_ylabel('Distance [m]')
        self.axs[0].grid(True)
        self.axs[0].legend(loc='upper left', fontsize=8)

        self.line_leader_v, = self.axs[1].plot([], [], 'r*-', label=f'{leader_id} ref v')
        self.line_front_v, = self.axs[1].plot([], [], 'b*-', label=f'{front_id} intention v')
        self.line_ego_v, = self.axs[1].plot([], [], 'g*-', label=f'{ego_id} MPC v')
        self.line_leader_v_now, = self.axs[1].plot([], [], 'ro', markersize=8, label=f'{leader_id} v now')
        self.line_front_v_now, = self.axs[1].plot([], [], 'bo', markersize=8, label=f'{front_id} v now')
        self.line_ego_v_now, = self.axs[1].plot([], [], 'go', markersize=8, label=f'{ego_id} v now')
        self.axs[1].set_ylabel('Speed [m/s]')
        self.axs[1].grid(True)
        self.axs[1].legend(loc='upper left', fontsize=8)

        self.line_leader_vs, = self.axs[2].plot([], [], 'r*-', label=f'{leader_id} ref v(s)')
        self.line_front_vs, = self.axs[2].plot([], [], 'b*-', label=f'{front_id} intention v(s)')
        self.line_ego_vs, = self.axs[2].plot([], [], 'g*-', label=f'{ego_id} MPC v(s)')
        self.line_leader_vs_now, = self.axs[2].plot([], [], 'ro', markersize=8, label=f'{leader_id} now')
        self.line_front_vs_now, = self.axs[2].plot([], [], 'bo', markersize=8, label=f'{front_id} now')
        self.line_ego_vs_now, = self.axs[2].plot([], [], 'go', markersize=8, label=f'{ego_id} now')
        self.axs[2].set_xlabel('Distance [m]')
        self.axs[2].set_ylabel('Speed [m/s]')
        self.axs[2].grid(True)
        self.axs[2].legend(loc='upper left', fontsize=8)
        self.fig.tight_layout()

    @staticmethod
    def _pair_sv(s_arr, v_arr):
        s_arr = np.asarray(s_arr, dtype=float)
        v_arr = np.asarray(v_arr, dtype=float)
        n = min(len(s_arr), len(v_arr))
        return s_arr[:n], v_arr[:n]

    def update(self, sim_time, preds_s, preds_v, veh_states_dict,
               cycle_ss=None, cycle_vs=None):
        if not self.enabled:
            return

        # nv0 reference-generated horizon (cycle used as nv1's front preview)
        if cycle_ss is not None:
            leader_s = np.asarray(cycle_ss, dtype=float)
            t_leader = sim_time + np.arange(len(leader_s)) * self.cycle_dt
            self.line_leader_s.set_data(t_leader, leader_s)
        else:
            leader_s = None
        if cycle_vs is not None:
            leader_v = np.asarray(cycle_vs, dtype=float)
            t_leader_v = sim_time + np.arange(len(leader_v)) * self.cycle_dt
            self.line_leader_v.set_data(t_leader_v, leader_v)
        else:
            leader_v = None
        if leader_s is not None and leader_v is not None:
            s_pair, v_pair = self._pair_sv(leader_s, leader_v)
            self.line_leader_vs.set_data(s_pair, v_pair)

        if self.front_id in preds_s:
            front_pred_s = np.asarray(preds_s[self.front_id], dtype=float)
            front_pred_v = np.asarray(preds_v.get(self.front_id, []), dtype=float)
            t_front = sim_time + np.arange(len(front_pred_s)) * self.pred_dt
            self.line_front_s.set_data(t_front, front_pred_s)
            self.line_front_v.set_data(t_front[:len(front_pred_v)], front_pred_v)
            s_pair, v_pair = self._pair_sv(front_pred_s, front_pred_v)
            self.line_front_vs.set_data(s_pair, v_pair)

        if self.ego_id in preds_s:
            ego_pred_s = np.asarray(preds_s[self.ego_id], dtype=float)
            ego_pred_v = np.asarray(preds_v.get(self.ego_id, []), dtype=float)
            t_ego = sim_time + np.arange(len(ego_pred_s)) * self.pred_dt
            self.line_ego_s.set_data(t_ego, ego_pred_s)
            self.line_ego_v.set_data(t_ego[:len(ego_pred_v)], ego_pred_v)
            s_pair, v_pair = self._pair_sv(ego_pred_s, ego_pred_v)
            self.line_ego_vs.set_data(s_pair, v_pair)

        for veh_id, line_s, line_v, line_vs in (
            (self.leader_id, self.line_leader_s_now, self.line_leader_v_now, self.line_leader_vs_now),
            (self.front_id, self.line_front_s_now, self.line_front_v_now, self.line_front_vs_now),
            (self.ego_id, self.line_ego_s_now, self.line_ego_v_now, self.line_ego_vs_now),
        ):
            if veh_id in veh_states_dict:
                s_now = veh_states_dict[veh_id][3]
                v_now = veh_states_dict[veh_id][2]
                line_s.set_data([sim_time], [s_now])
                line_v.set_data([sim_time], [v_now])
                line_vs.set_data([s_now], [v_now])

        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()
        self.fig.suptitle(f"sim t = {sim_time:.2f}s", fontsize=10)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def close(self):
        if self.enabled:
            plt.ioff()


if __name__ == "__main__":

    veh_data = {}
    veh_sim_t = []
    veh_real_t = []
    runtime_record = []

    current_dirname = os.path.dirname(__file__)
    parent_dir = os.path.abspath(os.path.join(current_dirname, os.pardir))
    spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3]) + 20.0

    sumo_config_path = parent_dir + "/sumo/v2x/" + SUMO_CONFIG
    configured_vehicle_order = get_configured_vehicle_order(sumo_config_path)
    add_vehicle_log_slots(veh_data, configured_vehicle_order, len(veh_sim_t))
    configured_leader_id = configured_vehicle_order[0] if configured_vehicle_order else None

    sumo_sim_manager = SumoSim(sumo_config_name=sumo_config_path)
    sumo_sim_manager.start_Sumo(gui=guiSumo)
    tracked_vehicle = False

    if USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=current_dirname)
    else:
        print('No controller for all vehicles')

    live_plot = LivePredPlot(
        LIVE_LEADER_ID, LIVE_FRONT_ID, LIVE_EGO_ID,
        pred_dt=MPC_DT, cycle_dt=REF_CYCLE_DT, enabled=livePredPlot,
        record_t=record_t, front_v_t=front_v_t, front_s_t=front_s_t,
    )

    real_start_time = time.monotonic()
    while sumo_sim_manager.step < END_TIME / SIM_STEP:

        sim_time = traci.simulation.getTime()
        if sim_time % 10 == 0:
            print(f"sim time: {sim_time}")

        sumo_sim_manager.simulationStepForward()
        vehicle_list = ordered_vehicle_ids(traci.vehicle.getIDList(), configured_vehicle_order)

        if not vehicle_list:
            continue

        leader_id = configured_leader_id if configured_leader_id in vehicle_list else vehicle_list[0]

        if guiSumo and not tracked_vehicle:
            track_id = vehicle_list[1] if len(vehicle_list) > 1 else leader_id
            traci.gui.trackVehicle("View #0", track_id)
            traci.gui.setZoom("View #0", 600)
            tracked_vehicle = True

        if sim_time < 2 * SIM_STEP:
            for veh in vehicle_list:
                traci.vehicle.setSpeed(veh, 0.0)
                traci.vehicle.setMinGap(veh, 0.001)
                traci.vehicle.setSpeedMode(veh, 96)
                traci.vehicle.setLength(veh, 3.2)
                traci.vehicle.setAccel(veh, 8.0)
                traci.vehicle.setDecel(veh, 8.0)
            continue

        add_vehicle_log_slots(veh_data, vehicle_list, len(veh_sim_t))

        v_lead_id = np.argmin(np.abs([record_t - sim_time]))
        v_tgt_lead = front_v_t[v_lead_id]
        if sim_time < STALLTIME:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID=leader_id, tgt_spd=v_tgt_lead)
        else:
            sumo_sim_manager.assignTargetSpeed(vehicle_ID=leader_id, tgt_spd=0)

        veh_states_matrix = [sumo_sim_manager.getVehicleStates(veh, returnStatesNum=5) for veh in vehicle_list]
        veh_states_dict = {state[0]: state for state in veh_states_matrix}

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
                verbose=verbosity,
            )
        else:
            acc = {veh: 0.0 for veh in vehicle_list}
            preds_s = {}
            preds_v = {}
            cycle_ss = None
            cycle_vs = None

        if verbosity:
            for veh in vehicle_list:
                if veh in preds_s:
                    print(f"{bcolors.OKCYAN}Preds_s for {veh}: {[round(s, 1) for s in preds_s[veh]]}{bcolors.ENDC}")
                    print(f"{bcolors.OKCYAN}Preds_v for {veh}: {[round(v, 1) for v in preds_v.get(veh, [])]}{bcolors.ENDC}")

        live_plot.update(
            sim_time, preds_s, preds_v, veh_states_dict,
            cycle_ss=cycle_ss, cycle_vs=cycle_vs,
        )

        if guiSumo and vizTraj:
            show_all_traj = not vizTrajVehs
            if show_all_traj or leader_id in vizTrajVehs:
                r, g, b, _ = traci.vehicle.getColor(leader_id)
                sumo_sim_manager.add_traj_leader(
                    leader_id,
                    veh_states_matrix[0][3],
                    record_t=record_t,
                    front_v_t=front_v_t,
                    sim_t=sim_time,
                    pred_dt=MPC_DT,
                    mpc_ref_stages=MPC_REF_STAGES,
                    colorChoice=(r, g, b, 100),
                    fill=False,
                    layer=3,
                )
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
            sumo_sim_manager.assignAcceleration(
                vehicle_ID=veh, tgt_acc=acc.get(veh, 0.0), dt=SUMO_ACC_INTEGRATE_DT
            )

        acc_cmds = {veh: acc.get(veh, 0.0) for veh in vehicle_list}
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
        veh_real_t.append(time.monotonic() - real_start_time)

        if veh_data.get(leader_id, {}).get('dist') and veh_data[leader_id]['dist'][-1] >= 210:
            print(f"{bcolors.FAIL_RED}***** Leader reached end of road. Stopping simulation. *****{bcolors.ENDC}")
            break

    live_plot.close()

    avg_runtime_ms = round(np.mean(runtime_record) * 1000, 4) if runtime_record else 0.0
    print('Average runtime is: ', str(avg_runtime_ms), 'ms')
    prefix = 'sumoSim_log_'
    save_all_veh_data_nv1_format(
        veh_data,
        veh_sim_t,
        veh_real_t,
        [np.nan] * len(veh_sim_t),
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

    traci.close(False)
