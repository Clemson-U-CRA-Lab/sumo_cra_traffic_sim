#! /usr/bin/env python3

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
from utils import *
from _controller import *
from _constants import *
import numpy as np

class SumoSim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo-gui"
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = len(self.vehID_list)
        self.step = 0
        
    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
    
    def getVehicleStates1(self, vehicle_ID, returnStatesNum=4):
        dtype = [('vehicle_ID', 'U20'), ('speed', 'f4'), ('lane_position', 'f4'), ('lane_ID', 'U20'), ('acceleration', 'f4'), ('position', '2f4')]
        stateVector = np.zeros(1, dtype=dtype)
        stateVector['vehicle_ID'] = vehicle_ID
        if vehicle_ID  in self.vehID_list:
            stateVector['speed'] = traci.vehicle.getSpeed(vehID=vehicle_ID)
            stateVector['lane_position'] = traci.vehicle.getLanePosition(vehID=vehicle_ID)
            stateVector['lane_ID'] = traci.vehicle.getLaneID(vehID=vehicle_ID)
            stateVector['acceleration'] = traci.vehicle.getAcceleration(vehID=vehicle_ID)
            stateVector['position'] = traci.vehicle.getPosition(vehID=vehicle_ID)
            if returnStatesNum == 4:
                return stateVector[['vehicle_ID', 'acceleration', 'speed', 'lane_position', 'lane_ID',]]
            elif returnStatesNum == 5:
                return stateVector[['vehicle_ID', 'acceleration', 'speed', 'lane_position', 'lane_ID', 'position']]
            else:
                print(f" Error !!! : Invalid put states requested: {returnStatesNum}")
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            stateVector['speed'] = 0.0
            stateVector['lane_position'] = 0.0
            stateVector['lane_ID'] = .0
            stateVector['acceleration'] = .0
            stateVector['position'] = .0
            return stateVector
        

    def getVehicleStates(self, vehicle_ID, returnStatesNum=4):
        if vehicle_ID  in self.vehID_list:
            veh_spd_t = traci.vehicle.getSpeed(vehID=vehicle_ID)
            veh_dist_t = traci.vehicle.getLanePosition(vehID=vehicle_ID)
            veh_lane_t = traci.vehicle.getLaneID(vehID=vehicle_ID)
            veh_acc_t = traci.vehicle.getAcceleration(vehID=vehicle_ID)
            veh_pos_t = traci.vehicle.getPosition(vehID=vehicle_ID)
            if returnStatesNum == 4:
                return [vehicle_ID, veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t]
            elif returnStatesNum == 5:
                return [vehicle_ID, veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t, veh_pos_t]
            
            else:
                print(f" Error !!! : Invalid put states requested: {returnStatesNum}")
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            stateVector = [0.0]*(returnStatesNum+1)
            stateVector[0] = vehicle_ID
            return stateVector


    def update_realCAV_in_sumo(veh="mache", spd=0, pos=None, dist=None):
        try:
            if pos != None:
                traci.vehicle.moveToXY(vehID=veh, edgeID="76146229#1", laneIndex="0", x=pos[0], y=pos[1])
            if spd != None:
                traci.vehicle.setSpeed(vehID=veh, speed=spd)
            print(f"{veh}: Position {pos}, Speed {spd}")
        except traci.TraCIException as e:
            print(f"Error updating vehicle:{veh}'s states  in sim")
    
    def assignAcceleration(self, vehicle_ID, tgt_acc, dt):
        if vehicle_ID in self.vehID_list:
            traci.vehicle.setAcceleration(vehID=vehicle_ID, acceleration=tgt_acc, duration=dt)
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            
    def assignTargetSpeed(self, vehicle_ID, tgt_spd):
        if vehicle_ID in self.vehID_list:
            traci.vehicle.setSpeed(vehID=vehicle_ID, speed=tgt_spd)
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
    
    def simulationStepForward(self):
        traci.simulationStep()
        self.vehID_list = traci.vehicle.getIDList()
        self.step += 1
