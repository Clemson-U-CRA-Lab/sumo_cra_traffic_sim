#! /usr/bin/env python3

'''
SUMO Manager class to control simulations along with VIL and V2X etc in main node

Prakhar Gupta
'''

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
        self.sumoBinaryNoGUI = "/usr/bin/sumo"
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = len(self.vehID_list)
        self.step = 0
        
    def start_Sumo(self, gui=True):
        if gui:
            sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        else:
            sumoCmd = [self.sumoBinaryNoGUI, "-c", self.sumoconfig]
        traci.start(sumoCmd)
        

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
                print(f"{bcolors.FAIL} Error !!! : Invalid put states requested: {returnStatesNum}{bcolors.ENDC}")
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            stateVector = [0.0]*(returnStatesNum+1)
            stateVector[0] = vehicle_ID
            return stateVector


    def update_realCAV_in_sumo(self, veh="nv2", spd=0.0, pos=None, dist=None, verbose=False):
        try:
            if pos != None:
                traci.vehicle.moveToXY(vehID=veh, edgeID="76146229#1", laneIndex="0", x=pos[0], y=pos[1])
            if spd != None:
                traci.vehicle.setSpeed(vehID=veh, speed=spd)
            if verbose:
                print(f"{bcolors.ENDC}{veh}: Position {pos}, Speed {spd}{bcolors.ENDC}")
        except traci.TraCIException as e:
            print(f"{bcolors.FAIL}Error updating vehicle:{veh}'s states  in sim{bcolors.ENDC}")
    
    def update_CAV_in_sumo(self, veh="nv1", spd=0.0, pos=None, dist=None, verbose=False):
        try:
            if pos != None:
                traci.vehicle.moveToXY(vehID=veh, edgeID="76146229#1", laneIndex="0", x=pos[0], y=pos[1])
            if spd != None:
                traci.vehicle.setSpeed(vehID=veh, speed=spd)
            if verbose:
                print(f"{bcolors.ENDC}{veh}: Position {pos}, Speed {spd}{bcolors.ENDC}")
        except traci.TraCIException as e:
            print(f"{bcolors.FAIL}Error updating vehicle:{veh}'s states  in sim{bcolors.ENDC}")


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
