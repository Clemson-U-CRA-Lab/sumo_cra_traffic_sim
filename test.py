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
nv = 'nv1' # the car that stalls

def send_nv_info(client_socket, server_address, nv_spd, nv_pos):

    # Prepare an array of two numbers (e.g., [12, 34])
    numbers_to_send = [nv_spd, nv_pos[0], nv_pos[1], nv_pos[2]]
    print("Sent to RSU: ", numbers_to_send)
    # Send the array of numbers to the server
    message = struct.pack(f'<{ARRAY_SIZE}f', *numbers_to_send)  # Packing the list as a series of integers as little endian
    client_socket.sendto(message, server_address)

# Function to get vehicle info (position and speed) for MachE from TCP server
def recv_machE_info(client_socket):

    # Receive data from the server (assuming position and speed are sent as floats)
    data = client_socket.recv(16)    
    # Parse the data (assuming position and speed are sent as  floats)
    spd, pos_x, pos_y, pos_phi = struct.unpack(f'<{ARRAY_SIZE}f', data)  # 'ff' means two floats
    print("Received from RSU: ", spd, pos_x, pos_y, pos_phi)

    return spd, [pos_x, pos_y, pos_phi]

def update_mache_in_sumo(veh="mache", spd=0, pos=None):
    try:
        if pos != None:
            traci.vehicle.moveToXY(vehID=veh, edgeID="76146229#1", laneIndex="0", x=pos[0], y=pos[1])
        if spd != None:
            traci.vehicle.setSpeed(vehID=veh, speed=spd)
        print(f"{veh}: Position {pos}, Speed {spd}")
    except traci.TraCIException as e:
        print(f"Error updating vehicle:{veh}'s states  in sim")


def main():

    # Create an IPv6 TCP socket
    # client_socket = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    # client_socket.connect((SERVER_IP, SERVER_PORT, 0, 2))  # Scope ID (2) may need to be changed for your network interface
    # client_socket.settimeout(TIMEOUT)

    # Start the SUMO simulation
    traci.start(sumoCmd)
    step = 0

    # Loop through the simulation steps
    while step < 200:
        traci.simulationStep()
        vehicle_list = traci.vehicle.getIDList()
        # print(vehicle_list)
        
        # Check if nv0 is in the vehicle list
        if nv in vehicle_list:
            nv_pos = traci.vehicle.getPosition(vehID=nv)
            nv_spd = traci.vehicle.getSpeed(vehID=nv)
            nv_dist = traci.vehicle.getDistance(vehID=nv)
            # nv0_lane_t = traci.vehicle.getLaneID(vehID=nv0)
            # nv0_acc_t = traci.vehicle.getAcceleration(vehID=nv0)
            # nv0_t = traci.simulation.getTime()

        
        if step == 10:
            update_mache_in_sumo(veh=nv, spd=0)

        # Retrieve mache's info from TCP server every step
        if step >= 0:

            mache_pos = [0,0,0]
            mache_spd = 0

            # Assuming veh_2 has a different ID and we want to set its position and speed in the simulation
            if "mache" in vehicle_list:
                # Update the position and speed of veh_2 using the received information
                traci.vehicle.moveToXY(vehID="mache", edgeID="76146229#1", laneIndex="0", x=mache_pos[0], y=mache_pos[1])
                traci.vehicle.setSpeed("mache", mache_spd)
                print(f"Mache: Position {mache_pos}, Speed {mache_spd}")
        
        step += 1
        time.sleep(0.2)

    # Close the SUMO simulation
    traci.close(False)

        
if __name__ == '__main__':
    main()
