import math
import csv
import numpy as np

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