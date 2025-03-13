import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
import time
import socket
import struct

# SUMO configuration
sumoBinary  = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "sumo/v2x/v2x.sumocfg"]
veh_0 = 'veh0'

# TCP Socket Setup
HOST = '127.0.0.1'  # Server address (localhost in this case)
PORT = 5000         # Port to connect to

# Function to get vehicle info (position and speed) for veh_2 from TCP server
def get_veh2_info():
    # Create a TCP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Connect to the server
        s.connect((HOST, PORT))

        # Send a request to the server to get veh_2 data
        s.sendall(b'GET_VEH2_INFO')

        # Receive data from the server (assuming position and speed are sent as floats)
        data = s.recv(1024)
        
        # Parse the data (assuming position and speed are sent as two floats)
        veh2_position, veh2_speed = struct.unpack('ff', data)  # 'ff' means two floats
        
        return veh2_position, veh2_speed

# Start the SUMO simulation
traci.start(sumoCmd)
step = 0

# Data lists for veh_0 (main vehicle)
veh_0_dist = []
veh_0_spd = []
veh_0_lane = []
veh_0_acc = []
veh_0_sim_t = []

# Loop through the simulation steps
while step < 200:
    traci.simulationStep()
    vehicle_list = traci.vehicle.getIDList()
    print(vehicle_list)
    
    # Check if veh_0 is in the vehicle list
    if veh_0 in vehicle_list:
        veh_0_pos_t = traci.vehicle.getPosition(vehID=veh_0)
        veh_0_spd_t = traci.vehicle.getSpeed(vehID=veh_0)
        veh_0_dist_t = traci.vehicle.getDistance(vehID=veh_0)
        veh_0_lane_t = traci.vehicle.getLaneID(vehID=veh_0)
        veh_0_acc_t = traci.vehicle.getAcceleration(vehID=veh_0)
        veh_0_t = traci.simulation.getTime()

        # Record the data
        veh_0_spd.append(veh_0_spd_t)
        veh_0_dist.append(veh_0_dist_t)
        veh_0_lane.append(veh_0_lane_t)
        veh_0_acc.append(veh_0_acc_t)
        veh_0_sim_t.append(veh_0_t)
    
    # Retrieve veh_2's info from TCP server every step (you can modify this as needed)
    if step == 50:
        print("Retrieving veh_2 information from TCP socket...")
        veh2_position, veh2_speed = get_veh2_info()

        # Assuming veh_2 has a different ID and we want to set its position and speed in the simulation
        if "veh_2" in vehicle_list:
            # Update the position and speed of veh_2 using the received information
            traci.vehicle.moveToXY("veh_2", veh2_position[0], veh2_position[1], angle=0)
            traci.vehicle.setSpeed("veh_2", veh2_speed)
            print(f"Updated veh_2: Position {veh2_position}, Speed {veh2_speed}")
    
    step += 1
    time.sleep(0.1)

# Close the SUMO simulation
traci.close(False)

# Plot the data for veh_0 (main vehicle)
plt.figure(1)
plt.subplot(2,2,1)
plt.plot(veh_0_sim_t, veh_0_spd)
plt.xlabel('Time [s]')
plt.ylabel('Vehicle speed [m/s]')

plt.subplot(2,2,2)
plt.plot(veh_0_sim_t, veh_0_lane)
plt.xlabel('Time [s]')
plt.ylabel('Lane ID')

plt.subplot(2,2,3)
plt.plot(veh_0_sim_t, veh_0_acc)
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')

plt.subplot(2,2,4)
plt.plot(veh_0_sim_t, veh_0_dist)
plt.xlabel('Time [s]')
plt.ylabel('Distance [m]')
plt.show()
