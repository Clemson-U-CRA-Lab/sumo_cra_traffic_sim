SIM_ARRAY_SIZE = 71  # Define the size of the array
VEH_ARRAY_SIZE = 68 # 7 for simple msg
# for 32 long orizon, sim_array_size = 71, veh_arr_size = 68.
BYTE_SIZE = 4 #8 tested with obu/rsu

# TCP Socket Setup
SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
SERVER_PORT = 7002  # Server port

# same machine testing only
# SERVER_IP = 'localhost'
# SERVER_PORT = 7002

# for direct to nuvo
# SERVER_IP = 'fe80::71c1:df07:f5d:c1f0' 
# SERVER_PORT = 7003

# TCP Socket Setup
TIMEOUT = 5  # Timeout
INTERFACE_SCOPE_ID = 7
MESSAGE_BYTE_LENGTH = BYTE_SIZE*VEH_ARRAY_SIZE

# SUMO params
SIM_STEP = 0.05
END_TIME = 75.0 #90
SUMO_ACC_DT = 3.0 # for traci.setAcceleration() in sumo
SUMO_CONFIG = "v2x_2veh.sumocfg" # which config to use
STALLTIME = 135.0

# Ref for front vehicle
CYCLE_DT = 0.1
CYCLE_STAGES = 100
# 0.1, 100 works well. 160 makes 16 second ghorizon ref.

# MPC config - dont change DT
MPC_DT = 0.5
MPC_REF_STAGES = 32
BOOL_USE_FRONT_PRVIEW = True # use preview of front's intention for ego's mpc?
