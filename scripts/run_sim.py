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


PREVIEW_STEPS = 20
PREVIEW_DT = 0.5
PREVIEW_TRAJECTORY_GAP_FEATURES = 19
PREVIEW_SAFE_DISTANCE_HEADWAY = 8.0


def preview_trajectory_to_vehicle_preview(
    trajectory_prediction,
):
    trajectory_prediction = np.asarray(trajectory_prediction, dtype=float).reshape(-1)
    if trajectory_prediction.size != PREVIEW_TRAJECTORY_GAP_FEATURES * 2:
        raise ValueError(
            "Unexpected preview trajectory output size: expected "
            + str(PREVIEW_TRAJECTORY_GAP_FEATURES * 2)
            + ", got "
            + str(trajectory_prediction.size)
        )

    ego_preview_s = np.concatenate(
        ([0.0], trajectory_prediction[:PREVIEW_TRAJECTORY_GAP_FEATURES])
    )
    ego_preview_v = np.concatenate(
        ([0.0], trajectory_prediction[PREVIEW_TRAJECTORY_GAP_FEATURES:])
    )
    return ego_preview_s, ego_preview_v


def terminal_prediction_to_vehicle_preview(
    terminal_prediction,
    preview_steps=PREVIEW_STEPS,
):
    terminal_prediction = np.asarray(terminal_prediction, dtype=float).reshape(-1)
    if terminal_prediction.size != 2:
        raise ValueError(
            "Unexpected terminal prediction size: expected 2, got "
            + str(terminal_prediction.size)
        )

    preview_progress = np.linspace(1.0 / preview_steps, 1.0, preview_steps)
    ego_preview_s = terminal_prediction[0] * preview_progress
    ego_preview_v = terminal_prediction[1] * preview_progress
    return ego_preview_s, ego_preview_v


def initialize_preview_animation(preview_steps, vehicle_index):
    plt.ion()
    figure, axes = plt.subplots(2, 1, num="Preview State Animation", figsize=(8, 6))
    preview_horizon = np.arange(1, preview_steps + 1)

    distance_headway_line, = axes[0].plot(preview_horizon, np.zeros(preview_steps), "b-o")
    axes[0].set_ylabel("Distance Headway [m]")
    axes[0].set_xlabel("Preview Step")
    axes[0].set_title("Preview States for veh" + str(vehicle_index))
    axes[0].grid(True)

    speed_gap_line, = axes[1].plot(preview_horizon, np.zeros(preview_steps), "r-o")
    axes[1].set_ylabel("Speed Gap [m/s]")
    axes[1].set_xlabel("Preview Step")
    axes[1].grid(True)

    plt.tight_layout()
    return {
        "figure": figure,
        "axes": axes,
        "preview_horizon": preview_horizon,
        "distance_headway_line": distance_headway_line,
        "speed_gap_line": speed_gap_line,
    }


def update_preview_animation(
    preview_animation,
    sim_t,
    vehicle_index,
    distance_headway_preview,
    speed_gap_preview,
):
    if preview_animation is None:
        return
    if not plt.fignum_exists(preview_animation["figure"].number):
        return

    distance_headway_preview = np.asarray(distance_headway_preview, dtype=float).reshape(-1)
    speed_gap_preview = np.asarray(speed_gap_preview, dtype=float).reshape(-1)

    preview_animation["distance_headway_line"].set_ydata(distance_headway_preview)
    preview_animation["speed_gap_line"].set_ydata(speed_gap_preview)

    preview_animation["axes"][0].set_title(
        "Preview States for veh" + str(vehicle_index) + " at t=" + str(round(sim_t, 2)) + " s"
    )

    preview_animation["axes"][0].relim()
    preview_animation["axes"][0].autoscale_view()
    preview_animation["axes"][1].relim()
    preview_animation["axes"][1].autoscale_view()

    preview_animation["figure"].canvas.draw_idle()
    plt.pause(0.001)


class sumo_sim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo"
        self.sumoGUIBinary = "/usr/bin/sumo-gui"
        self.open_gui = False
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = 0
        self.step = 0
    
    def init_vehicles_large_map(self, num_vehicle):
        self.num_veh = num_vehicle
        self.sumo_veh = [None]*num_vehicle
        for i in range(int(self.num_veh)):
            self.sumo_veh[i] = SUMO_vehicles(vehicle_ID="veh" + str(i), init_s= 600 - 12 * i, init_lane=0, route_ID="route1", lane_change_mode=0, sumo_brake=False)
        # for j in range(int(self.num_veh / 2), self.num_veh):
        #     self.sumo_veh[j] = SUMO_vehicles(vehicle_ID="veh" + str(j), init_s= 350 - 12 * (j - int(num_veh/2)), init_lane=1, route_ID="route1", lane_change_mode=0)
    
    def init_vehicles_CMI(self, num_vehicle):
        self.num_veh = num_vehicle
        self.sumo_veh = [None]*num_vehicle
        for i in range(self.num_veh):
            self.sumo_veh[i] = SUMO_vehicles(vehicle_ID="veh" + str(i), init_s= 30 - 8 * i, init_lane=0, route_ID="route1", lane_change_mode=0, sumo_brake=False)

    def start_Sumo(self, open_gui=False):
        self.open_gui = bool(open_gui)
        sumo_binary = self.sumoGUIBinary if open_gui else self.sumoBinary
        sumoCmd = [sumo_binary, "-c", self.sumoconfig, "--quit-on-end", "--collision.action", "none"]
        traci.start(sumoCmd)
    
    def simulationStepForward(self):
        traci.simulationStep()
        self.vehID_list = traci.vehicle.getIDList()
        self.step += 1


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--logging_sim", help="whether to save the simulation data", action="store_true")
    parser.add_argument("--plot_result", help="whether to plot result after sim stop", default=False, action="store_true")
    parser.add_argument("--animate_preview", help="show live preview-state animation", action="store_true")
    parser.add_argument("--no_gui", help="run SUMO headlessly", action="store_true")
    parser.add_argument(
        "--print_level",
        choices=["quiet", "info", "debug"],
        default="info",
        help="control console output level",
    )
    parser.add_argument(
        "--disable_preview_cbf",
        help="disable CBF safety control for the PreviewNN controller",
        action="store_true",
    )
    parser.add_argument(
        "--disable_explicit_cbf",
        help="disable CBF safety control for Explicit MPC controllers",
        action="store_true",
    )
    parser.add_argument("--preview_vehicle_index", type=int, default=1, help="vehicle index used for preview animation")
    parser.add_argument("--num_sv", type=int, default=2.0, help="Number of vehicles in the traffic")
    parser.add_argument('leading_speed_profile', choices=['Nyc', 'Hwy', 'Ftp', 'US06','FTPsec1','FTPsec2','FTPsec3'], help='Choose leading vehicles speed profile')
    parser.add_argument("control_type", choices=['MPC', 'ExplicitUnconnected', 'ExplicitConnected', 'NN', 'PreviewNN', 'TerminalFCN', 'IDM'], help='Choose control method for traffic vehicles')
    args = parser.parse_args()
    
    # Traffic control setting
    USING_EXPLICIT_UNCONNECTED = 0
    USING_EXPLICIT_CONNECTED = 0
    if args.control_type == 'MPC':
        USING_ONLINE_MPC = 1 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 0 # If using neural network controller to track front vehicle
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 0 # If using IDM to traffic front vehicle
    elif args.control_type == 'ExplicitUnconnected':
        USING_ONLINE_MPC = 0
        USING_NEURAL_NETWORK = 0
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 0
        USING_TERMINAL_FCN = 0
        USING_EXPLICIT_UNCONNECTED = 1
    elif args.control_type == 'ExplicitConnected':
        USING_ONLINE_MPC = 0
        USING_NEURAL_NETWORK = 0
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 0
        USING_TERMINAL_FCN = 0
        USING_EXPLICIT_CONNECTED = 1
    elif args.control_type == 'NN':
        USING_ONLINE_MPC = 0 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 1 # If using neural network controller to track front vehicle
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 0 # If using IDM to traffic front vehicle
    elif args.control_type == 'PreviewNN':
        USING_ONLINE_MPC = 0 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 0 # If using neural network controller to track front vehicle
        USING_PREVIEW_NEURAL_NETWORK = 1
        USING_IDM = 0 # If using IDM to traffic front vehicle
        USING_TERMINAL_FCN = 0
    elif args.control_type == 'TerminalFCN':
        USING_ONLINE_MPC = 0
        USING_NEURAL_NETWORK = 0
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 0
        USING_TERMINAL_FCN = 1
    else:
        USING_ONLINE_MPC = 0 # If using online MPC to track front vehicle
        USING_NEURAL_NETWORK = 0 # If using neural network controller to track front vehicle
        USING_PREVIEW_NEURAL_NETWORK = 0
        USING_IDM = 1 # If using IDM to traffic front vehicle
        USING_TERMINAL_FCN = 0

    if args.control_type != 'TerminalFCN':
        USING_TERMINAL_FCN = 0
        
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
    elif args.leading_speed_profile == 'Ftp':
        spd_filename = parent_dir + "/speed_profile/I85_ftp_short.csv"
    elif args.leading_speed_profile == 'US06':
        spd_filename = parent_dir + "/speed_profile/US06_CMI_Urban_speed_profile.csv"
    elif args.leading_speed_profile == 'FTPsec1':
        spd_filename = parent_dir + "/speed_profile/FTPcol_sec1_spd_profile.csv"
    elif args.leading_speed_profile == 'FTPsec2':
        spd_filename = parent_dir + "/speed_profile/FTPcol_sec2_spd_profile.csv"
    elif args.leading_speed_profile == 'FTPsec3':
        spd_filename = parent_dir + "/speed_profile/FTPcol_sec3_spd_profile.csv"    
    else:
        print('Unable to locate speed profile')
        sys.exit(1)
    leading_vehicle_speed_profile = driving_cycle_spd_profile_reader(spd_filename)
    
    if args.leading_speed_profile == 'Hwy' or args.leading_speed_profile == 'Nyc' or args.leading_speed_profile == 'Ftp':
        sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/I-85_highway/I-85.sumocfg")
        sumo_sim_manager.start_Sumo(open_gui=not args.no_gui and bool(os.environ.get("DISPLAY")))
        sumo_sim_manager.init_vehicles_large_map(num_vehicle=num_veh)
    else:
        sumo_sim_manager = sumo_sim(sumo_config_name=parent_dir + "/sumo/CMI/cmi.sumocfg")
        sumo_sim_manager.start_Sumo(open_gui=not args.no_gui and bool(os.environ.get("DISPLAY")))
        sumo_sim_manager.init_vehicles_CMI(num_vehicle=num_veh)
    
    # Initialize controller
    dirname = os.path.dirname(__file__)
    nn_pt_filename = dirname + '/traffic_following_control_dc_trained.pt'
    preview_nn_pt_filename = os.path.abspath(
        os.path.join(
            parent_dir,
            os.pardir,
            "offline_eco_car_following_control",
            "Preview_car_following_experiment",
            "preview_traffic_following_control_conv_best.pt",
        )
    )
    terminal_fcn_pt_filename = os.path.abspath(
        os.path.join(
            parent_dir,
            os.pardir,
            "offline_eco_car_following_control",
            "Preview_car_following_experiment",
            "terminal_preview_fcn_best.pt",
        )
    )
    
    # Setup controller
    if USING_NEURAL_NETWORK:
        FCN_control = NN_controller(
            nn_pt_file=nn_pt_filename,
            input_num=3,
            print_level=args.print_level,
        )
        controller_name = 'Neural_Network'
        print('Use neural network to control traffic vehicles')
    elif USING_PREVIEW_NEURAL_NETWORK:
        if not os.path.exists(preview_nn_pt_filename):
            raise FileNotFoundError(
                "Missing preview NN checkpoint: " + preview_nn_pt_filename
            )
        Preview_control = PreviewNN_controller(
            nn_pt_file=preview_nn_pt_filename,
            preview_steps=PREVIEW_STEPS,
            safe_distance_headway=PREVIEW_SAFE_DISTANCE_HEADWAY,
            enable_cbf_safety=not args.disable_preview_cbf,
            print_level=args.print_level,
        )
        controller_name = 'Preview_Neural_Network'
        print('Use preview neural network to control traffic vehicles')
    elif USING_TERMINAL_FCN:
        if not os.path.exists(terminal_fcn_pt_filename):
            raise FileNotFoundError(
                "Missing terminal FCN checkpoint: " + terminal_fcn_pt_filename
            )
        Terminal_control = TerminalPreviewFCN_controller(
            nn_pt_file=terminal_fcn_pt_filename,
            safe_distance_headway=PREVIEW_SAFE_DISTANCE_HEADWAY,
            enable_cbf_safety=not args.disable_preview_cbf,
            print_level=args.print_level,
        )
        controller_name = 'Terminal_Preview_FCN'
        print('Use terminal preview FCN to control traffic vehicles')
    elif USING_ONLINE_MPC:
        online_MPC_control = PCC_MPC_controller(dirname=dirname)
        controller_name = 'Online_MPC'
        print('Use online MPC to control traffic vehicles')
    elif USING_EXPLICIT_UNCONNECTED:
        explicit_control = ExplicitMPCUnconnectedController(
            config=ExplicitMPCConfig(dt=0.5, horizon=20, sample_count=10000,
                                     enable_cbf=not args.disable_explicit_cbf),
            generate_regions=True,
            print_level=args.print_level)
        controller_name = 'Explicit_Unconnected'
        print('Use reconstructed 20-stage explicit MPC with unconnected PV prediction')
    elif USING_EXPLICIT_CONNECTED:
        explicit_control = ExplicitMPCConnectedController(
            config=ExplicitMPCConfig(dt=0.5, horizon=20, sample_count=10000,
                                     enable_cbf=not args.disable_explicit_cbf),
            generate_regions=True,
            print_level=args.print_level)
        controller_name = 'Explicit_Connected'
        print('Use reconstructed 20-stage explicit MPC with connected PV preview')
    elif USING_IDM:
        IDM_control = IDM(a=2, b=3, s0=7, v0=20, T=1.5)
        controller_name = 'Intelligent_Driving_Model'
        print('Use IDM to control traffic vehicles')
    else:
        print('No controller for all vehicles')
        
    record_t = np.array(leading_vehicle_speed_profile[:, 0])
    front_v_t = np.array(leading_vehicle_speed_profile[:, 1])
    front_s_t = np.array(leading_vehicle_speed_profile[:, 3])
    
    if sumo_sim_manager.open_gui:
        traci.gui.trackVehicle("View #0", "veh1")
        traci.gui.setZoom("View #0", 1000)
    
    Avg_spd_traffic = []
    Avg_density_traffic = []
    ego_v = []
    pv_v = []
    explicit_diagnostics = {
        "evaluations": 0,
        "region_hits": 0,
        "fallbacks": 0,
        "cbf_overrides": 0,
    }
    lead_s = 50.0
    end_s = 0.0
    preview_animation = None

    if args.animate_preview:
        if args.preview_vehicle_index <= 0 or args.preview_vehicle_index >= num_veh:
            raise ValueError("Preview animation vehicle index must be between 1 and num_sv - 1.")
        preview_animation = initialize_preview_animation(
            preview_steps=PREVIEW_STEPS,
            vehicle_index=args.preview_vehicle_index,
        )
    
    while sumo_sim_manager.step * 0.1 < 45: #record_t[-1] + 20:
        # Measure one complete SUMO/control step.  The interval includes the
        # SUMO advance, state collection, all follower controller evaluations,
        # control assignment, and per-step statistics.  The deliberate sleep
        # used to pace the loop is kept outside this measurement.
        step_start_time = time.perf_counter()
        sumo_sim_manager.simulationStepForward()
        sim_t = sumo_sim_manager.step * 0.1
        
        # Initialize power record
        P_t = []
        Spd_t = []
        
        s_vt_traffic = []
        pv_vt_traffic = []
        
        s_st_traffic = []
        pv_st_traffic = []
        
        s_at_traffic = []
        pv_at_traffic = []
        terminal_predicted_s = {}
        terminal_predicted_v = {}
        terminal_vehicle_indices = []
        terminal_ego_v_traffic = []
        terminal_ego_a_traffic = []
        terminal_ego_s_traffic = []
        terminal_pv_v_traffic = []
        terminal_pv_s_traffic = []
        terminal_distance_headway_traffic = []
        terminal_speed_gap_traffic = []
        preview_vehicle_indices = []
        preview_ego_v_traffic = []
        preview_ego_a_traffic = []
        preview_ego_s_traffic = []
        preview_pv_v_traffic = []
        preview_pv_s_traffic = []
        preview_distance_headway_traffic = []
        preview_speed_gap_traffic = []
        # MPC running
        for i in range(0, num_veh):
            if i == 0:# or i == int(num_veh/2):
                # Get leading vehicle speed
                v_lead_id = np.argmin(np.abs([record_t - sim_t]))
                v_tgt_lead = front_v_t[v_lead_id] #+ 2.0 * (random.random() - 0.5)
                sumo_sim_manager.sumo_veh[i].assignTargetSpeed(v_tgt_lead)
                [veh_1_acc_t, veh_1_spd_t, veh_1_dist_t] = sumo_sim_manager.sumo_veh[i].getVehicleStates()
                lead_s = veh_1_dist_t
                # Update state preview
                lead_prev_v, lead_prev_s = driving_cycle_state_preview_searching(
                    sim_t=sim_t,
                    record_t=record_t,
                    front_v_t=front_v_t,
                    mpc_dt=PREVIEW_DT,
                    front_s_init=lead_s,
                )
                # Load future state preview
                sumo_sim_manager.sumo_veh[i].update_vehicle_future_states_preview(
                    lead_prev_s[:PREVIEW_STEPS],
                    lead_prev_v[:PREVIEW_STEPS],
                    sim_step=sumo_sim_manager.step,
                    preview_dt=PREVIEW_DT,
                    source="driving_cycle",
                )
                terminal_predicted_s[i] = lead_prev_s[PREVIEW_STEPS - 1]
                terminal_predicted_v[i] = lead_prev_v[PREVIEW_STEPS - 1]
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
                                                                     ego_object=sumo_sim_manager.sumo_veh[i], leading_preview=True)
            elif USING_EXPLICIT_UNCONNECTED:
                explicit_acceleration, explicit_diag = explicit_control.step(
                    ego_state=[veh_1_dist_t, veh_1_spd_t, veh_1_acc_t],
                    pv_state=[veh_0_dist_t, veh_0_spd_t, veh_0_acc_t],
                    return_diagnostics=True)
                acc_traffic_step_t = [explicit_acceleration]
                explicit_diagnostics["evaluations"] += 1
                explicit_diagnostics["region_hits"] += int(not explicit_diag["fallback"])
                explicit_diagnostics["fallbacks"] += int(explicit_diag["fallback"])
                explicit_diagnostics["cbf_overrides"] += int(explicit_diag["safety_override"])
            elif USING_EXPLICIT_CONNECTED:
                # Match the current Eco-MPC connected path: every follower
                # receives the same driving-cycle preview through setPred().
                pv_preview_v, pv_preview_s = driving_cycle_state_preview_searching(
                    sim_t=sim_t,
                    record_t=record_t,
                    front_v_t=front_v_t,
                    mpc_dt=0.5,
                    front_s_init=veh_0_dist_t)
                explicit_acceleration, explicit_diag = explicit_control.step(
                    ego_state=[veh_1_dist_t, veh_1_spd_t, veh_1_acc_t],
                    pv_state=[veh_0_dist_t, veh_0_spd_t, veh_0_acc_t],
                    pv_position_preview=pv_preview_s,
                    pv_velocity_preview=pv_preview_v,
                    return_diagnostics=True)
                acc_traffic_step_t = [explicit_acceleration]
                explicit_diagnostics["evaluations"] += 1
                explicit_diagnostics["region_hits"] += int(not explicit_diag["fallback"])
                explicit_diagnostics["fallbacks"] += int(explicit_diag["fallback"])
                explicit_diagnostics["cbf_overrides"] += int(explicit_diag["safety_override"])
                # Broadcast a compact four-step intent for the next follower.
                ego_preview_s, ego_preview_v, _ = sumo_sim_manager.sumo_veh[i].predict_constant_acceleration_preview(
                    preview_steps=4, preview_dt=0.1)
                sumo_sim_manager.sumo_veh[i].update_vehicle_future_states_preview(
                    ego_preview_s, ego_preview_v, sim_step=sumo_sim_manager.step,
                    preview_dt=0.1, source='explicit_connected')
            elif USING_NEURAL_NETWORK or USING_IDM:
                s_vt_traffic.append(veh_1_spd_t)
                pv_vt_traffic.append(veh_0_spd_t)
                s_st_traffic.append(veh_1_dist_t)
                pv_st_traffic.append(veh_0_dist_t)
                s_at_traffic.append(veh_1_acc_t)
                pv_at_traffic.append(veh_0_acc_t)
                continue
            elif USING_PREVIEW_NEURAL_NETWORK:
                distance_headway_preview, speed_gap_preview = (
                    sumo_sim_manager.sumo_veh[i].build_preview_features_from_preceding_vehicle(
                        preceding_vehicle=sumo_sim_manager.sumo_veh[i-1],
                        preview_steps=PREVIEW_STEPS,
                        preview_dt=PREVIEW_DT,
                        sim_step=sumo_sim_manager.step,
                        use_current_ego_state=True,
                    )
                )
                preview_vehicle_indices.append(i)
                preview_ego_v_traffic.append(veh_1_spd_t)
                preview_ego_a_traffic.append(veh_1_acc_t)
                preview_ego_s_traffic.append(veh_1_dist_t)
                preview_pv_v_traffic.append(veh_0_spd_t)
                preview_pv_s_traffic.append(veh_0_dist_t)
                preview_distance_headway_traffic.append(distance_headway_preview)
                preview_speed_gap_traffic.append(speed_gap_preview)
                continue
            elif USING_TERMINAL_FCN:
                if (i - 1) in terminal_predicted_s and (i - 1) in terminal_predicted_v:
                    terminal_distance_headway = (
                        terminal_predicted_s[i - 1] - veh_1_dist_t
                    )
                    terminal_speed_gap = (
                        terminal_predicted_v[i - 1] - veh_1_spd_t
                    )
                else:
                    distance_headway_preview, speed_gap_preview = (
                        sumo_sim_manager.sumo_veh[i].build_preview_features_from_preceding_vehicle(
                            preceding_vehicle=sumo_sim_manager.sumo_veh[i-1],
                            preview_steps=PREVIEW_STEPS,
                            preview_dt=PREVIEW_DT,
                            sim_step=sumo_sim_manager.step,
                            use_current_ego_state=True,
                        )
                    )
                    terminal_distance_headway = distance_headway_preview[-1]
                    terminal_speed_gap = speed_gap_preview[-1]

                terminal_vehicle_indices.append(i)
                terminal_ego_v_traffic.append(veh_1_spd_t)
                terminal_ego_a_traffic.append(veh_1_acc_t)
                terminal_ego_s_traffic.append(veh_1_dist_t)
                terminal_pv_v_traffic.append(veh_0_spd_t)
                terminal_pv_s_traffic.append(veh_0_dist_t)
                terminal_distance_headway_traffic.append(terminal_distance_headway)
                terminal_speed_gap_traffic.append(terminal_speed_gap)
                continue
            else:
                acc_traffic_step_t = np.zeros(3)
                
            acc = acc_traffic_step_t[0]
            
            # Assign the acceleration to ego vehicle
            sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc, v_max=30)

        if USING_PREVIEW_NEURAL_NETWORK and preview_vehicle_indices:
            acc_prediction, trajectory_prediction = Preview_control.step_forward(
                ego_vt=np.asarray(preview_ego_v_traffic, dtype=float),
                distance_headway_preview=np.asarray(preview_distance_headway_traffic, dtype=float),
                speed_gap_preview=np.asarray(preview_speed_gap_traffic, dtype=float),
                pv_vt=np.asarray(preview_pv_v_traffic, dtype=float),
                pv_st=np.asarray(preview_pv_s_traffic, dtype=float),
                s_st=np.asarray(preview_ego_s_traffic, dtype=float),
                sim_t=sim_t,
                lambda_smooth=1.0,
                s_at=np.asarray(preview_ego_a_traffic, dtype=float),
                use_cbf_safety=not args.disable_preview_cbf,
                return_trajectory=True,
            )

            for batch_id, vehicle_index in enumerate(preview_vehicle_indices):
                ego_preview_s, ego_preview_v = preview_trajectory_to_vehicle_preview(
                    trajectory_prediction=trajectory_prediction[batch_id],
                )
                sumo_sim_manager.sumo_veh[vehicle_index].update_vehicle_future_states_preview(
                    ego_preview_s,
                    ego_preview_v,
                    sim_step=sumo_sim_manager.step,
                    preview_dt=PREVIEW_DT,
                    source="preview_nn",
                    is_relative=True,
                )
                sumo_sim_manager.sumo_veh[vehicle_index].assignTargetAcceleration(
                    acc_prediction[batch_id],
                    v_max=30,
                )
        elif USING_TERMINAL_FCN and terminal_vehicle_indices:
            acc_prediction, terminal_prediction = Terminal_control.step_forward(
                ego_vt=np.asarray(terminal_ego_v_traffic, dtype=float),
                distance_headway_final=np.asarray(terminal_distance_headway_traffic, dtype=float),
                speed_gap_final=np.asarray(terminal_speed_gap_traffic, dtype=float),
                pv_vt=np.asarray(terminal_pv_v_traffic, dtype=float),
                pv_st=np.asarray(terminal_pv_s_traffic, dtype=float),
                s_st=np.asarray(terminal_ego_s_traffic, dtype=float),
                sim_t=sim_t,
                lambda_smooth=0.0,
                s_at=np.asarray(terminal_ego_a_traffic, dtype=float),
                use_cbf_safety=not args.disable_preview_cbf,
                return_terminal=True,
            )

            for batch_id, vehicle_index in enumerate(terminal_vehicle_indices):
                terminal_predicted_s[vehicle_index] = (
                    terminal_ego_s_traffic[batch_id] + terminal_prediction[batch_id][0]
                )
                terminal_predicted_v[vehicle_index] = (
                    terminal_ego_v_traffic[batch_id] + terminal_prediction[batch_id][1]
                )
                ego_preview_s, ego_preview_v = terminal_prediction_to_vehicle_preview(
                    terminal_prediction=terminal_prediction[batch_id],
                    preview_steps=PREVIEW_STEPS,
                )
                sumo_sim_manager.sumo_veh[vehicle_index].update_vehicle_future_states_preview(
                    ego_preview_s,
                    ego_preview_v,
                    sim_step=sumo_sim_manager.step,
                    preview_dt=PREVIEW_DT,
                    source="terminal_fcn",
                    is_relative=True,
                )
                sumo_sim_manager.sumo_veh[vehicle_index].assignTargetAcceleration(
                    acc_prediction[batch_id],
                    v_max=30,
                )
        
        if USING_NEURAL_NETWORK:
            acc_traffic_step_t = FCN_control.step_forward(s_vt=np.array(s_vt_traffic), pv_vt=np.array(pv_vt_traffic),
                                                          s_st=np.array(s_st_traffic), pv_st=np.array(pv_st_traffic),
                                                          s_at=np.array(s_at_traffic), pv_at=np.array(pv_at_traffic),
                                                          use_prediction_horizon=True, sim_t=sim_t, lambda_smooth=8.0)
            sumo_sim_manager.sumo_veh[1].assignTargetAcceleration(acc_traffic_step_t[0], v_max=30)
            for i in range(1, num_veh):
                sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc_traffic_step_t[i-1], v_max=30)
        if USING_IDM:
            acc_traffic_step_t = IDM_control.IDM_acceleration(front_v=np.array([pv_vt_traffic]),
                                                              ego_v=np.array([s_vt_traffic]),
                                                              front_s=np.array([pv_st_traffic]),
                                                              ego_s=np.array([s_st_traffic]))
            sumo_sim_manager.sumo_veh[1].assignTargetAcceleration(acc_traffic_step_t[0][0], v_max=30)
            for i in range(1, num_veh):
                sumo_sim_manager.sumo_veh[i].assignTargetAcceleration(acc_traffic_step_t[0][i-1], v_max=30)
        if args.animate_preview and args.preview_vehicle_index < num_veh:
            selected_vehicle = sumo_sim_manager.sumo_veh[args.preview_vehicle_index]
            preceding_vehicle = sumo_sim_manager.sumo_veh[args.preview_vehicle_index - 1]
            distance_headway_preview, speed_gap_preview = (
                selected_vehicle.build_preview_features_from_preceding_vehicle(
                    preceding_vehicle=preceding_vehicle,
                    preview_steps=PREVIEW_STEPS,
                    preview_dt=PREVIEW_DT,
                    sim_step=sumo_sim_manager.step,
                    use_current_ego_state=True,
                )
            )
            update_preview_animation(
                preview_animation=preview_animation,
                sim_t=sim_t,
                vehicle_index=args.preview_vehicle_index,
                distance_headway_preview=distance_headway_preview,
                speed_gap_preview=speed_gap_preview,
            )
        # Add power consumption
        Power_t.append(P_t)
        ego_state_t = sumo_sim_manager.sumo_veh[1].getVehicleStates()
        pv_state_t = sumo_sim_manager.sumo_veh[0].getVehicleStates()
        
        ego_v.append(ego_state_t[1])
        pv_v.append(pv_state_t[1])
        
        # Add traffic flow speed
        avg_spd_t = np.mean(np.array(Spd_t))
        Avg_spd_traffic.append(avg_spd_t)
        
        # Add traffic density
        traffic_rho = traffic_density_measurement(lead_s=lead_s, end_s=end_s, num_vehicles=num_veh)
        Avg_density_traffic.append(traffic_rho)
        veh_sim_t.append(sim_t)

        # This is the comparable runtime value for every controller: one
        # complete simulation/control step, rather than one controller call.
        runtime_dt = time.perf_counter() - step_start_time
        runtime_record.append(runtime_dt)

        if args.print_level != "quiet":
            print('Full SUMO step runtime is: ', str(round(runtime_dt * 1000, 3)),
                  'ms. Distance:', str(round(veh_1_dist_t, 1)), 'm.', end='\r')
        
        if args.logging_sim:
            data_logger(sim_t=sim_t, ego_a=veh_1_acc_t, ego_v=veh_1_spd_t, ego_s=veh_1_dist_t,
                        pv_a=veh_0_acc_t, pv_v=veh_0_spd_t, pv_s=veh_0_dist_t, filename= args.leading_speed_profile + "_" + controller_name + ".csv")
        
        time.sleep(0.001)
    
    traci.close(True)
    if preview_animation is not None and plt.fignum_exists(preview_animation["figure"].number):
        plt.ioff()
    
    veh_sim_t = np.array(veh_sim_t)
    
    Energy_t = np.sum(np.array(Power_t) * 0.1, axis=0)
    
    runtime_record = np.array(runtime_record)
    max_runtime = np.max(runtime_record)
    runtime_record_valid = runtime_record != max_runtime # Remove the maximum runtime, which is usually the first step
    runtime_record = runtime_record[runtime_record_valid] 
    
    print('Average runtime is: ', str(round(np.mean(runtime_record) * 1000, 4)), 'ms')
    print('Max runtime is: ', str(round(np.max(runtime_record) * 1000, 4)), 'ms')
    print('Min runtime is: ', str(round(np.min(runtime_record) * 1000, 4)), 'ms')
    print('Runtime standard deviation is: ', str(round(np.std(runtime_record) * 1000, 4)), 'ms')
    if explicit_diagnostics["evaluations"]:
        print(
            'Explicit MPC regions: ',
            str(len(explicit_control.regions)),
            '; coverage: ',
            str(round(100.0 * explicit_diagnostics["region_hits"] /
                      explicit_diagnostics["evaluations"], 2)),
            '%; OSQP fallbacks: ',
            str(explicit_diagnostics["fallbacks"]),
            '; CBF overrides: ',
            str(explicit_diagnostics["cbf_overrides"]),
        )
    
    # Record runtime
    filename =  "Runtime_" + controller_name + ".csv"
    with open(filename, "a") as f:
        writer = csv.writer(f)
        writer.writerow([str(num_veh),
                         str(round(np.mean(runtime_record) * 1000, 4)),
                         str(round(np.min(runtime_record) * 1000, 4)), 
                         str(round(np.max(runtime_record) * 1000, 4)), 
                         str(round(np.std(runtime_record) * 1000, 4))])
    
    if args.plot_result:
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

        plt.figure(3)
        plt.plot(veh_sim_t, ego_v, '-k')
        plt.plot(veh_sim_t, pv_v, '-b')
        plt.legend(['Ego vehicle', 'Preceding vehicle'])
        plt.xlabel('Time [s]', fontsize=20)
        plt.ylabel('Ego vehicle speed [m/s]', fontsize=20)

        plt.show()
