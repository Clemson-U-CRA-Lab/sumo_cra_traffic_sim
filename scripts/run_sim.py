import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt

class sumo_sim():
    def __init__(self, sumo_config_name):
        self.sumoBinary = "/usr/bin/sumo-gui"
        self.sumoconfig = sumo_config_name
        self.vehID_list = []
        self.num_veh = len(self.vehID_list)
        self.step = 0
        
    def start_Sumo(self):
        sumoCmd = [self.sumoBinary, "-c", self.sumoconfig]
        traci.start(sumoCmd)
    
    def getVehicleStates(self, vehicle_ID):
        if vehicle_ID  in self.vehID_list:
            veh_spd_t = traci.vehicle.getSpeed(vehID=vehicle_ID)
            veh_dist_t = traci.vehicle.getDistance(vehID=vehicle_ID)
            veh_lane_t = traci.vehicle.getLaneID(vehID=vehicle_ID)
            veh_acc_t = traci.vehicle.getAcceleration(vehID=vehicle_ID)
            return [veh_acc_t, veh_spd_t, veh_dist_t, veh_lane_t]
        else:
            print(vehicle_ID + " doesn't exist in the traffic")
            return [0.0, 0.0, 0.0, 0.0]
    
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

if __name__=="__main__":
    step = 0

    veh_0_dist = []
    veh_0_spd = []
    veh_0_lane = []
    veh_0_acc = []
    veh_0_sim_t = []
    
    sumo_sim_manager = sumo_sim(sumo_config_name="sumo/CMI/cmi.sumocfg")
    sumo_sim_manager.start_Sumo()
    
    while sumo_sim_manager.step < 600:
        sumo_sim_manager.simulationStepForward()
    
    traci.close(False)