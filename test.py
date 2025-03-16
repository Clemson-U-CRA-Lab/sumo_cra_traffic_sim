# This script will run on the RoadSide computer - sends sumo info to MachE

import os
import sys
import traci
import traci.constants as tc
import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
import time
import socket
import struct
import serial

# TCP Socket Setup as client
SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # Server IP address (modify as necessary)
SERVER_PORT = 7002  # Server port
ARRAY_SIZE = 4 # Define the size of the array
TIMEOUT = 30  # Timeout
server_address = (SERVER_IP, SERVER_PORT)

# SUMO configuration
sumoBinary  = "/usr/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "sumo/v2x/v2x.sumocfg"]
nv0 = 'nv0'

def send_nv_info(client_socket, server_address, nv_spd, nv_pos):

    # Prepare an array of two numbers (e.g., [12, 34])
    numbers_to_send = [nv_spd, nv_pos[0], nv_pos[1], nv_pos[2]]
    # Send the array of numbers to the server
    message = struct.pack(f'<{ARRAY_SIZE}f', *numbers_to_send)  # Packing the list as a series of integers as little endian
    client_socket.sendto(message, server_address)

    # time.sleep(.2)

# Function to get vehicle info (position and speed) for MachE from TCP server
def recv_machE_info(client_socket):

    print("Retrieving MachE information from TCP with RSU...")

    # Receive data from the server (assuming position and speed are sent as floats)
    data = client_socket.recv(16)
    # print(data, len(data))
    
    # Parse the data (assuming position and speed are sent as  floats)
    spd, pos_x, pos_y, pos_phi = struct.unpack(f'<{ARRAY_SIZE}f', data)  # 'ff' means two floats
    
    return spd, [pos_x, pos_y, pos_phi]

def main():

 

    # Start the SUMO simulation
    traci.start(sumoCmd)
    step = 0

    # Loop through the simulation steps
    while step < 200:
        traci.simulationStep()
        vehicle_list = traci.vehicle.getIDList()
        print(vehicle_list)
        
        # Check if nv0 is in the vehicle list
        if nv0 in vehicle_list:
            nv0_pos_t = traci.vehicle.getPosition(vehID=nv0)
            nv0_spd_t = traci.vehicle.getSpeed(vehID=nv0)
            nv0_dist_t = traci.vehicle.getDistance(vehID=nv0)
            nv0_lane_t = traci.vehicle.getLaneID(vehID=nv0)
            nv0_acc_t = traci.vehicle.getAcceleration(vehID=nv0)
            nv0_t = traci.simulation.getTime()

            # send_nv_info(client_socket, server_address, nv_spd=nv0_spd_t, nv_pos=[nv0_pos_t[0], nv0_pos_t[1], nv0_dist_t])
            print("Sent: ",nv0_spd_t,  [nv0_pos_t[0], nv0_pos_t[1], nv0_dist_t])
            # send_nv_info(client_socket, server_address, nv_spd=.4, nv_pos=[.3, .2, .2])
        
        
        step += 1
        time.sleep(0.5)

    # Close the SUMO simulation
    traci.close(False)

        
if __name__ == '__main__':
    main()
