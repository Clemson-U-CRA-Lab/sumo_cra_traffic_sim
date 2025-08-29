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
import xml.etree.ElementTree as ET

from x2v_constants import *

class SumoSim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo-gui"
        self.sumoBinaryNoGUI = "/usr/bin/sumo"
        self.sumoconfig = sumo_config_name
        self.update_sumoConfig(SIM_STEP)
        self.vehID_list = []
        self.num_veh = len(self.vehID_list)
        self.step = 0
        
    def start_Sumo(self, gui=True):
        if gui:
            sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        else:
            sumoCmd = [self.sumoBinaryNoGUI, "-c", self.sumoconfig]
        traci.start(sumoCmd)

        # 96 - no checks, 0 - most chcks off but speed limit adhered
        # this should run only when the vehicles are already in network otherwise it wont update anything bcause we are iterating through vehicle in vehicle_list.
        # for vehID in traci.vehicle.getIDList():
            # traci.vehicle.setMinGap(vehID, 0.1)
            # traci.vehicle.setSpeed(vehID, 0.0)
            # traci.vehicle.setSpeedMode(vehID, 96)
            # traci.vehicle.setAccel(vehID, 10)
            # traci.vehicle.setDecel(vehID, 10)
            # traci.vehicle.setEmergencyDecel(vehID, 10)
        

    def update_sumoConfig(self, SimStepLength):
        # Parse the XML file
        tree = ET.parse(self.sumoconfig)
        root = tree.getroot()

        # Find the 'time' element and update 'step-length' attribute
        for time_elem in root.findall("time"):
            step_length_elem = time_elem.find("step-length")
            if step_length_elem is not None:
                step_length_elem.set("value", str(SimStepLength))

        # Save the modified file
        tree.write(self.sumoconfig)

    def getVehicleStates(self, vehicle_ID, returnStatesNum=4):
        if vehicle_ID  in self.vehID_list:
            veh_spd_t = traci.vehicle.getSpeed(vehID=vehicle_ID)
            veh_dist_t = traci.vehicle.getLanePosition(vehID=vehicle_ID)
            veh_lane_t = traci.vehicle.getLaneID(vehID=vehicle_ID)
            veh_acc_t = traci.vehicle.getAcceleration(vehID=vehicle_ID)
            veh_pos_t = traci.vehicle.getPosition(vehID=vehicle_ID)
            if returnStatesNum == 4:
                return [veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t]
            elif returnStatesNum == 5:
                # [acc, spd, s, l, [x,y]]
                return [vehicle_ID, veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t, veh_pos_t]
            
            else:
                print(f"{bcolors.FAIL} Error !!! : Invalid put states requested: {returnStatesNum}{bcolors.ENDC}")
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            stateVector = [0.0]*(returnStatesNum+1)
            stateVector[0] = vehicle_ID
            return stateVector

    
    def update_CAV_in_sumo(self, veh="nv1", spd=0.0, pos=None, dist=None, edge="76146229#1", verbose=False):
        try:
            if pos != None:
                traci.vehicle.moveToXY(vehID=veh, edgeID=edge , laneIndex="0", x=pos[0], y=pos[1])
            if spd != None:
                traci.vehicle.setSpeed(vehID=veh, speed=spd)
            if dist != None:
                traci.vehicle.moveTo(vehID=veh, laneID='76146229#1_0', pos=dist)
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


    def add_traj(self, vehID, 
                 preds_s, 
                 edgeID="76146229#1", laneId=0, 
                 colorChoice=(255,0,0,100), 
                 fill=False, layer=2):
        pred_traj = []
        for s in preds_s:
            try:
                x,y = traci.simulation.convert2D(edgeID=edgeID, pos=s, laneIndex=laneId)
                pred_traj.append((x,y))
            except:
                if f"{vehID}_traj" in traci.polygon.getIDList():
                    # print("vis trajectory extending beyong route... not plotting")
                    traci.polygon.remove(f"{vehID}_traj")
                    pred_traj = []
                break
        if pred_traj:
            if f"{vehID}_traj" not in traci.polygon.getIDList():
                traci.polygon.add(
                    polygonID=f"{vehID}_traj",
                    shape=pred_traj,
                    color=colorChoice,  # Red with transparency
                    fill=fill,  # No fill
                    polygonType="trajectory",
                    layer=layer,
                    lineWidth=2.0  # Make it more visible
                )
            else:
                traci.polygon.setShape(
                    polygonID=f"{vehID}_traj",
                    shape=pred_traj
                )
    
    def add_traj_leader(self, vehID, 
                        leader_s, record_t, front_v_t, 
                        sim_t, 
                        pred_dt=MPC_DT, mpc_ref_stages=MPC_REF_STAGES,
                        edgeID="76146229#1", 
                        laneId=0, colorChoice=(255,0,0,100), fill=False, layer=2):
        
        cycle_vs = np.empty(mpc_ref_stages)
        cycle_vs.fill(np.nan)
        
        for i in range(MPC_REF_STAGES):
            t_id = np.argmin(np.abs([record_t - (i * pred_dt + sim_t)]))
            cycle_vs[i] = front_v_t[t_id]
        
        preds_s = scipy.integrate.cumulative_trapezoid(cycle_vs, dx=pred_dt) + leader_s
        
        pred_traj = []
        for s in preds_s:
            try:
                x,y = traci.simulation.convert2D(edgeID=edgeID, pos=s, laneIndex=laneId)
                pred_traj.append((x,y))
            except:
                
                if f"{vehID}_traj" in traci.polygon.getIDList():
                    # print("vis trajectory extending beyong route... not plotting")
                    traci.polygon.remove(f"{vehID}_traj")
                    pred_traj = []
                break
        
        if pred_traj:
            if f"{vehID}_traj" not in traci.polygon.getIDList():
                traci.polygon.add(
                    polygonID=f"{vehID}_traj",
                    shape=pred_traj,
                    color=colorChoice,  # Red with transparency
                    fill=fill,  # No fill
                    polygonType="trajectory",
                    layer=layer,
                    lineWidth=2.0  # Make it more visible
                )
            else:
                traci.polygon.setShape(
                    polygonID=f"{vehID}_traj",
                    shape=pred_traj
                )
        