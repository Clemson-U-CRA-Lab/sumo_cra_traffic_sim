import math
import csv
import numpy as np
import scipy

class IDM():
    def __init__(self, a, b, s0, v0, T):
        self.a = a
        self.b = b
        self.s0 = s0
        self.v0 = v0
        self.T = T

    def IDM_acceleration(self, front_v, ego_v, front_s, ego_s):
        s_safe = self.s0 + ego_v * self.T + front_v * \
            (ego_v - front_v) / (2 * (self.a * self.b)**0.5)
        acc = self.a * (1 - (ego_v / self.v0) ** 4 -
                        (s_safe / (front_s - ego_s - 5)) ** 2)
        acc = np.clip(acc, -3, 3)
        return acc

def delta_yaw_correction(delta_yaw):
    if delta_yaw > math.pi:
        delta_yaw = delta_yaw - 2*math.pi
    elif delta_yaw < -math.pi:
        delta_yaw = delta_yaw + 2*math.pi
    else:
        delta_yaw = delta_yaw
    return delta_yaw

def vehicle_coordinate_transformation(goal_pose, vehicle_pose):
    dx = goal_pose[0] - vehicle_pose[0]
    dy = goal_pose[1] - vehicle_pose[1]
    v_yaw = delta_yaw_correction(goal_pose[2] - vehicle_pose[2])
    v_x = dx * math.cos(vehicle_pose[2]) + dy * math.sin(vehicle_pose[2])
    v_y = dy * math.cos(vehicle_pose[2]) - dx * math.sin(vehicle_pose[2])
    v_goal_pose = np.array([v_x, v_y, v_yaw])
    return v_goal_pose

def vehicle_coordinate_transformation_3D(goal_pose, vehicle_pose):
    dx = goal_pose[0] - vehicle_pose[0]
    dy = goal_pose[1] - vehicle_pose[1]
    v_x = dx * math.cos(vehicle_pose[3]) + dy * math.sin(vehicle_pose[3])
    v_y = dy * math.cos(vehicle_pose[3]) - dx * math.sin(vehicle_pose[3])
    
    dxdy = (dx**2 + dy**2)**0.5
    dz = goal_pose[2] - vehicle_pose[2]
    v_z = dz * math.cos(vehicle_pose[4]) - dxdy * math.sin(vehicle_pose[4])
    
    v_yaw = delta_yaw_correction(goal_pose[3] - vehicle_pose[3])
    v_pitch = goal_pose[4] - vehicle_pose[4]
    
    return np.array([v_x, v_y, v_z, v_yaw, v_pitch])

def global_path_reader(global_path_name):
    with open(global_path_name) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1]), float(
        point[2]), float(point[3])) for point in path_points]
    path_points_pos_x = [float(point[0]) for point in path_points]
    path_points_pos_y = [float(point[1]) for point in path_points]
    path_point_pos_yaw = [float(point[2]) for point in path_points]
    path_point_pos_s = [float(point[3]) for point in path_points]
    global_path = np.transpose(np.array(
        [path_points_pos_x, path_points_pos_y, path_point_pos_yaw, path_point_pos_s]))
    return global_path

def global_path_reader2(global_path_name):
    with open(global_path_name) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1]), \
                    float(point[2]), float(point[3]), float(point[4]), \
                        float(point[5]), float(point[6])) for point in path_points]
    path_points_pos_x = [float(point[1]) for point in path_points]
    path_points_pos_y = [float(point[2]) for point in path_points]
    path_point_pos_yaw = [float(point[4]) for point in path_points]
    path_point_pos_time = [float(point[0]) for point in path_points]
    path_point_pos_velocity = [float(point[5]) for point in path_points]
    
    path_s = []
    path_s.append(0)
    for i in range(1, len(path_points_pos_x)):
        s = path_s[i-1] + np.linalg.norm([(path_points_pos_x[i]-path_points_pos_x[i-1]) , (path_points_pos_y[i]-path_points_pos_y[i-1])])
        path_s.append(s)
        
    global_path = np.transpose(np.array(
        [path_point_pos_time, path_points_pos_x, path_points_pos_y, path_s, path_point_pos_yaw, path_point_pos_velocity]))
    return global_path

def global_path_reader_mixed_reality(global_path_name):
    with open(global_path_name) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1]), float(
        point[2]), float(point[3]), float(point[4]), float(point[5])) for point in path_points]
    path_points_pos_x = [float(point[0]) for point in path_points]
    path_points_pos_y = [float(point[1]) for point in path_points]
    path_point_pos_z = [float(point[2]) for point in path_points]
    path_point_pos_yaw = [float(point[3]) for point in path_points]
    path_point_pos_pitch = [float(point[4]) for point in path_points]
    path_point_pos_dist = [float(point[5]) for point in path_points]
    global_path = np.transpose(np.array(
        [path_points_pos_x, path_points_pos_y, path_point_pos_z, path_point_pos_yaw, path_point_pos_pitch, path_point_pos_dist]))
    return global_path

def global_spd_profile_reader(speed_profile_name):
    with open(speed_profile_name) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1])) for point in path_points]
    path_points_t = [float(point[0]) for point in path_points]
    path_points_spd = [float(point[1]) for point in path_points]
    spd_profile = np.transpose(np.array([path_points_t, path_points_spd]))
    return spd_profile

def driving_cycle_spd_profile_reader(driving_cycle_profile_name):
    with open(driving_cycle_profile_name) as f:
        data_pointes = [tuple(line) for line in csv.reader(f)]
    data_pointes = [(float(point[0]), float(point[1]), float(point[2]), float(point[3])) for point in data_pointes]
    data_point_t = [float(point[0]) for point in data_pointes]
    data_point_spd = [float(point[1]) for point in data_pointes]
    data_point_acc = [float(point[2]) for point in data_pointes]
    data_point_dist = [float(point[3]) for point in data_pointes]
    spd_profile = np.transpose(np.array([data_point_t, data_point_spd, data_point_acc, data_point_dist]))
    return spd_profile

def TTCi_estimate(ego_v, front_v, front_s):
    ttc_i = (ego_v - front_v) / front_s
    return ttc_i

def traffic_online_MPC_control_step(veh_0_acc_t, veh_0_spd_t, veh_0_dist_t,
                                    veh_1_acc_t, veh_1_spd_t, veh_1_dist_t,
                                    veh_2_acc_t, veh_2_spd_t, veh_2_dist_t,
                                    veh_3_acc_t, veh_3_spd_t, veh_3_dist_t,
                                    sim_t, record_t, front_v_t, online_MPC_control):
    
    # Find leading vehicle's driving cycle
    cycle_vs = np.empty(32)
    cycle_vs.fill(np.nan)
    
    for i in range(32):
        t_id = np.argmin(np.abs([record_t - (i * 0.1 + sim_t)]))
        cycle_vs[i] = front_v_t[t_id]
    
    cycle_ss = scipy.integrate.cumulative_trapezoid(cycle_vs, dx=0.1) + veh_0_dist_t
    print(f"{bcolors.OKGREEN}Pred S for 1st vehicle:\{[f'{x:.2f}' for x in cycle_ss]}{bcolors.ENDC}")
    print(f"{bcolors.OKBLUE}Pred V for 1st vehicle:\{[f'{x:.2f}' for x in cycle_vs]}{bcolors.ENDC}")

    veh_1_pred_s, veh_1_pred_v, acc_1 = online_MPC_control.svs.setCommand_SUMO(t = sim_t, ego_s=veh_1_dist_t, ego_v=veh_1_spd_t, ego_a=veh_1_acc_t,
                                                   pv_s=veh_0_dist_t, pv_v=veh_0_spd_t, pv_a=veh_0_acc_t, cycle_ss=cycle_ss,
                                                   cycle_vs=cycle_vs)
    
    print(f"{bcolors.OKGREEN}Pred S for 2nd vehicle:\{[f'{x:.2f}' for x in veh_1_pred_s]}{bcolors.ENDC}")
    print(f"{bcolors.OKBLUE}Pred V for 2nd vehicle:\{[f'{x:.2f}' for x in veh_1_pred_v]}{bcolors.ENDC}")

    veh_2_pred_s, veh_2_pred_v, acc_2 = online_MPC_control.svs.setCommand_SUMO(t = sim_t, ego_s=veh_2_dist_t, ego_v=veh_2_spd_t, ego_a=veh_2_acc_t,
                                                   pv_s=veh_1_dist_t, pv_v=veh_1_spd_t, pv_a=veh_1_acc_t, cycle_ss=veh_1_pred_s,
                                                   cycle_vs=veh_1_pred_v)
    
    print(f"{bcolors.OKGREEN}Pred S for 3rd vehicle:\{[f'{x:.2f}' for x in veh_2_pred_s]}{bcolors.ENDC}")
    print(f"{bcolors.OKBLUE}Pred V for 3rd vehicle:\{[f'{x:.2f}' for x in veh_2_pred_v]}{bcolors.ENDC}")

    veh_3_pred_s, veh_3_pred_v, acc_3 = online_MPC_control.svs.setCommand_SUMO(t = sim_t, ego_s=veh_3_dist_t, ego_v=veh_3_spd_t, ego_a=veh_3_acc_t,
                                                   pv_s=veh_2_dist_t, pv_v=veh_2_spd_t, pv_a=veh_2_acc_t, cycle_ss=veh_2_pred_s,
                                                   cycle_vs=veh_2_pred_v)
    
    
    return [acc_1, acc_2, acc_3]



import numpy as np
import scipy.integrate

def traffic_online_MPC_control_step_nVeh(nVehicleStatesMatrix, 
                                         sim_t, record_t, 
                                         front_v_t, online_MPC_control, 
                                         simStep, mpc_dt=0.5, 
                                         mpc_ref_stages=50, 
                                         verbose=False,
                                         useFirstVehCyclePreview=False,
                                         outputUsedCycleforFront=False):
    """
    Perform online MPC control step for multiple vehicles.
    
    :param states: List of vehicle states [[acc, spd, dist, lane, pos], [..], ...] in order of leading to trailing.
    :param sim_t: Current simulation time.
    :param record_t: time of profile
    :param front_v_t: Speed profile of the leading vehicle.
    :param online_MPC_control: MPC controller object.
    :return: List of computed accelerations for each vehicle except the leader.
    """
    num_vehicles = len(nVehicleStatesMatrix)
    accelerations = {}
    preds_s ={}
    preds_v = {}
    cycle_dt = mpc_dt
    # cycle_dt = simStep

    # Compute leading vehicle's driving cycle
    cycle_vs = np.full(mpc_ref_stages, np.nan)
    tPred = np.full(mpc_ref_stages, np.nan)
    for i in range(mpc_ref_stages):
        t_id = np.argmin(np.abs(record_t - (i * cycle_dt + sim_t)))
        cycle_vs[i] = front_v_t[t_id]
        tPred[i] = record_t[t_id]

    cycle_ss = scipy.integrate.cumulative_trapezoid(cycle_vs, dx=cycle_dt, initial=0) + nVehicleStatesMatrix[0][3]
    
    # Iterate over states (excluding the leader)
    prev_pred_s, prev_pred_v = cycle_ss, cycle_vs
    for i in range(1, num_vehicles):
        
        if i == 1:
            ego_acc, ego_spd, ego_dist = nVehicleStatesMatrix[i][1:4]
            pv_acc, pv_spd, pv_dist = nVehicleStatesMatrix[i-1][1:4]
            
            pred_s, pred_v, acc, pred_t = online_MPC_control.svs.setCommand_SUMO(
                                                t=sim_t, 
                                                ego_s=ego_dist, ego_v=ego_spd, ego_a=ego_acc,
                                                pv_s=pv_dist, pv_v=pv_spd, pv_a=pv_acc,
                                                cycle_ss=prev_pred_s, cycle_vs=prev_pred_v,
                                                cycle_dt=cycle_dt, n_refs=mpc_ref_stages,
                                                preview=useFirstVehCyclePreview # no preds for nv0
            )
            preds_s[nVehicleStatesMatrix[i][0]] = pred_s
            preds_v[nVehicleStatesMatrix[i][0]] = pred_v
            accelerations[nVehicleStatesMatrix[i][0]] = acc
        else:   
            ego_acc, ego_spd, ego_dist = nVehicleStatesMatrix[i][1:4]
            pv_acc, pv_spd, pv_dist = nVehicleStatesMatrix[i-1][1:4]
            
            pred_s, pred_v, acc, pred_t = online_MPC_control.svs.setCommand_SUMO(
                                                t=sim_t, 
                                                ego_s=ego_dist, ego_v=ego_spd, ego_a=ego_acc,
                                                pv_s=pv_dist, pv_v=pv_spd, pv_a=pv_acc,
                                                cycle_ss=prev_pred_s, cycle_vs=prev_pred_v,
                                                cycle_dt=mpc_dt, n_refs=32,
                                                preview=True
            )
            preds_s[nVehicleStatesMatrix[i][0]] = pred_s
            preds_v[nVehicleStatesMatrix[i][0]] = pred_v
            accelerations[nVehicleStatesMatrix[i][0]] = acc

        if verbose:
            if i == 1:
                print(f"{bcolors.WARNING}===MPC Ref Cycle of nv{i-1} for nv{i}==={bcolors.ENDC}")
                print(f"{bcolors.OKCYAN}Cycle  T for nv{i-1}: {len(tPred)},{[f'{x:.2f}' for x in tPred]}{bcolors.ENDC}")
                print(f"{bcolors.OKGREEN}Cycle  S for nv{i-1}: {len(prev_pred_s)}, {[f'{x:.2f}' for x in prev_pred_s]}{bcolors.ENDC}")
                print(f"{bcolors.OKBLUE}Cycle  V for nv{i-1}: {len(prev_pred_v)}, {[f'{x:.2f}' for x in prev_pred_v]}{bcolors.ENDC}")
            print(f"{bcolors.WARNING}===MPC Solved for nv{i}==={bcolors.ENDC}")
            print(f"{bcolors.OKCYAN}Pred T for nv{i}: {len(pred_t)},{[f'{x:.2f}' for x in pred_t]}{bcolors.ENDC}")
            print(f"{bcolors.OKGREEN}Pred S for nv{i}: {len(pred_s)}, {[f'{x:.2f}' for x in pred_s]}{bcolors.ENDC}")
            print(f"{bcolors.OKBLUE}Pred V for nv{i}: {len(pred_v)}, {[f'{x:.2f}' for x in pred_v]}{bcolors.ENDC}")

        prev_pred_s, prev_pred_v = pred_s, pred_v
        
    if outputUsedCycleforFront:
        return accelerations, preds_s, preds_v, cycle_ss, cycle_vs
    
    return accelerations, preds_s, preds_v


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    FAIL_RED =  '\033[91m'
    HEADER_MAGENTA = '\033[95m'
    WARNING_YELLOW = '\033[93m'
