import math
import csv
import numpy as np
import scipy
import traci
from _controller import IDM

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
        acc = np.clip(acc, -6, 4)
        return acc

class SUMO_Traffic_Light():
    def __init__(self, s_TL, t_TL, status_TL, red_duration, amber_duration, green_duration):
        self.s_TL = s_TL
        self.status_TL = status_TL
        self.TL_timing = t_TL
        self.RD_duration = red_duration
        self.AM_duration = amber_duration
        self.GR_duration = green_duration
    
    def TL_update(self, TL_id):
        # Update traffic signal timing
        TL_t = self.TL_timing[TL_id]
        # Check the status and duration
        TL_status = self.status_TL[TL_id]
        # Update traffic light status
        if TL_status == 0:
            if TL_t > self.RD_duration:
                TL_status = 2
                TL_t = 0.0
            else:
                TL_status = 0
                TL_t += 0.1
        elif TL_status == 1:
            if TL_t > self.AM_duration:
                TL_status = 0
                TL_t = 0.0
            else:
                TL_status = 1
                TL_t += 0.1
        elif TL_status == 2:
            if TL_t > self.GR_duration:
                TL_status = 1
                TL_t = 0.0
            else:
                TL_status = 2
                TL_t += 0.1
        else:
            print("Something is wrong about this traffic light...(-_-`)")
            
        self.TL_timing[TL_id] = TL_t
        self.status_TL[TL_id] = TL_status

class SUMO_vehicles():
    def __init__(self, vehicle_ID, init_s, init_lane, route_ID, lane_change_mode, sumo_brake):
        self.ID = vehicle_ID
        self.pv_ID = None
        self.v = 0.0
        self.a = 0.0
        self.s = init_s
        self.init_dist = init_s
        self.lane_ID = init_lane
        self.pTL_s = None
        self.pTL_id = None
        self.pv_s_prev = None
        self.pv_v_prev = None

        traci.vehicle.add(self.ID, route_ID, typeID = 'electricCar', departLane=str(self.lane_ID), departPos=self.s, departSpeed=20)
        if self.s > 200:
            traci.vehicle.moveToXY(self.ID, edgeID="E2_0", laneIndex=0, x=self.s - 200,y=0)
        elif self.s > 1000:
            traci.vehicle.moveToXY(self.ID, edgeID="E3_0", laneIndex=0, x=self.s - 1000,y=0)
        traci.vehicle.setParameter(objectID=self.ID, key='vClass', value='evehicle')
        traci.vehicle.setLaneChangeMode(vehID=self.ID, laneChangeMode=lane_change_mode)
        if sumo_brake:
            traci.vehicle.setSpeedMode(vehID=self.ID, speedMode=96)
    
    def update_preceding_traffic_light(self, TL_s):
        # Find the traffic that is in front of the traffic light
        [_, _, ego_s] = self.getVehicleStates()
        TL_s = np.array(TL_s)
        TL_id = np.where(TL_s > ego_s)[0]
        if len(TL_id) > 0:
            self.pTL_id = TL_id[0].tolist()
            self.pTL_s = TL_s[TL_id[0]].tolist() - ego_s
        else:
            self.pTL_s = None
            self.pTL_id = None
    
    def getVehicleStates(self):
        veh_v_t = traci.vehicle.getSpeed(vehID=self.ID)
        veh_s_t = traci.vehicle.getDistance(vehID=self.ID) + self.init_dist
        veh_a_t = traci.vehicle.getAcceleration(vehID=self.ID)
        
        return [veh_a_t, veh_v_t, veh_s_t]
    
    def getVehicleTrafficStates(self):
        veh_v_t = traci.vehicle.getSpeed(vehID=self.ID)
        veh_s_t = traci.vehicle.getLanePosition(vehID=self.ID)
        veh_a_t = traci.vehicle.getAcceleration(vehID=self.ID)
        
        return [veh_a_t, veh_v_t, veh_s_t]
    
    def assignTargetSpeed(self, tgt_spd):
        traci.vehicle.setSpeed(vehID=self.ID, speed=tgt_spd)
    
    def assignTargetAcceleration(self, tgt_acc, v_max):
        [_, v, _] = self.getVehicleStates()
        if v - v_max > 0:
            tgt_acc = np.min([tgt_acc, 0.0])
        
        self.a = self.a + 1.0 * (tgt_acc - self.a)
        
        traci.vehicle.setAcceleration(vehID=self.ID, acceleration=self.a, duration=0.1)
    
    def assignLaneChangeMode(self, mode):
        traci.vehicle.setLaneChangeMode(vehID=self.ID, laneChangeMode=mode)
    
    def update_vehicle_future_states_preview(self, pv_s, pv_v):
        self.pv_s_prev = pv_s
        self.pv_v_prev = pv_v
    
    def get_electricity_power(self):
        electric_consumption = traci.vehicle.getElectricityConsumption(vehID=self.ID)
        return electric_consumption

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

def driving_cycle_state_preview_searching(sim_t, record_t, front_v_t, mpc_dt, front_s_init):
    # Find leading vehicle's driving cycle
    cycle_vs = np.empty(32)
    cycle_vs.fill(np.nan)
    
    for i in range(32):
        t_id = np.argmin(np.abs([record_t - (i * mpc_dt + sim_t)]))
        cycle_vs[i] = front_v_t[t_id]
    
    cycle_ss = scipy.integrate.cumulative_trapezoid(cycle_vs, dx=mpc_dt) + front_s_init
    
    return cycle_vs, cycle_ss
    

def traffic_online_MPC_control_step(veh_0_acc_t, veh_0_spd_t, veh_0_dist_t,
                                    veh_1_acc_t, veh_1_spd_t, veh_1_dist_t,
                                    sim_t, online_MPC_control, record_t, 
                                    front_v_t, mpc_dt, pv_object, ego_object, 
                                    leading_preview=False):
    
    IDM_brake = IDM(a=3, b=5, s0=8, v0=30, T=5)
    
    if leading_preview:
        cycle_vs, cycle_ss = driving_cycle_state_preview_searching(sim_t=sim_t, record_t=record_t, front_v_t=front_v_t, mpc_dt=mpc_dt, front_s_init=veh_0_dist_t)
    else:
        cycle_vs = []#pv_object.pv_v_prev
        cycle_ss = []#pv_object.pv_s_prev
        
    veh_1_pred_s, veh_1_pred_v, a_MPC = online_MPC_control.svs.setCommand_SUMO(t = sim_t, ego_s=veh_1_dist_t, ego_v=veh_1_spd_t, ego_a=veh_1_acc_t,
                                                   pv_s=veh_0_dist_t, pv_v=veh_0_spd_t, pv_a=veh_0_acc_t, cycle_ss=cycle_ss, cycle_vs=cycle_vs, cycle_dt=mpc_dt)
    if leading_preview:
        ego_object.update_vehicle_future_states_preview(np.array(veh_1_pred_s) - 5.0, veh_1_pred_v)
    
    # Compute intelligent driver model control
    # ttc_i = TTCi_estimate(ego_v=veh_1_spd_t, front_v=veh_0_spd_t, front_s=veh_0_dist_t - veh_1_dist_t)
    # s_a_IDM = IDM_brake.IDM_acceleration(front_v=veh_0_spd_t, ego_v=veh_1_spd_t, front_s=veh_0_dist_t, ego_s=veh_1_dist_t)
    # 
    # det = ((ttc_i > 0.15) + (veh_0_dist_t - veh_1_dist_t < 15)).astype(bool)
    # IDM_w = det.astype(float)
    # ego_a_tgt = IDM_w * s_a_IDM + (1.0 - IDM_w) * a_MPC
    
    return [a_MPC]

def engine_power_estimation(ego_v, ego_a):
    m = 2218 # Vehicle weights
    rho = 1.293 # Air density
    Cd = 0.28 # Drag coefficient
    A = 2.84 # Frontal area
    mu = 0.015 # Rolling resistance
    g = 9.8 # Gravity
    
    f_aero = 0.5 * A * Cd * rho * ego_v ** 2
    f_roll = mu * m * g
    f_inertia = ego_a * m
    
    P = (f_aero + f_roll + f_inertia) * ego_v
    P = np.max([P, 0])
    
    return P

def data_logger(sim_t, ego_a, ego_v, ego_s, pv_a, pv_v, pv_s, filename):
    with open(filename, "a") as f:
        writer = csv.writer(f)
        writer.writerow([sim_t, ego_a, ego_v, ego_s, pv_a, pv_v, pv_s])
        
def traffic_density_measurement(lead_s, end_s, num_vehicles):
    rho = num_vehicles / ((lead_s - end_s) / 1000)
    return rho

def average_speed_measurement(veh_v_t):
    avg_veh_v_t = np.mean(np.array(veh_v_t))
    return avg_veh_v_t