# TCP Socket Setup
# for via cohda RSU.
# SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
# SERVER_PORT = 7002  # Server port

# same machine testing only
# SERVER_IP = 'localhost'
# SERVER_PORT = 7002

# for direct to nuvo
SERVER_IP = 'fe80::71c1:df07:f5d:c1f0' 
SERVER_PORT = 7003

# TCP Socket Setup
TIMEOUT = 5  # Timeout
INTERFACE_SCOPE_ID = 6

# SUMO params and run params
SIM_STEP = 0.1
END_TIME = 90.0 #90
SUMO_ACC_INTEGRATE_DT = 3.0 # for traci.setAcceleration() in sumo
SUMO_CONFIG = "v2x_2veh.sumocfg" # which config to use
STALLTIME = 135.0

# Ref for front vehicle
REF_CYCLE_DT = 0.2
REF_CYCLE_STAGES = 32 # 32, 100
# 0.1, 100 works well. 160 makes 16 second ghorizon ref.

# MPC config - dont change DT
MPC_DT = 0.5
MPC_REF_STAGES = 32 
BOOL_USE_FRONT_PREVIEW = True   # use preview of front's intention for ego's mpc?
# If True, its intention sharing , if False, setPred() is used

# For V2X and X2V interfaces
COMMS_FREQ = 10 #Hz - No use going above 1/SIM_STEP frequency.

SIM_ARRAY_SIZE = 7+2*REF_CYCLE_STAGES  # Define the size of the array
VEH_ARRAY_SIZE = 68 # 7 for simple msg
# for 32 long horizon, sim_array_size = 71, veh_arr_size = 68.
# for 100 stages, sim_array_size = 207
BYTE_SIZE = 4 #8 tested with obu/rsu
MESSAGE_BYTE_LENGTH = BYTE_SIZE*VEH_ARRAY_SIZE
INTERFACE_SCOPE_ID = 6


# Attack:
BOOL_ATTACK = True
ATTACK_INTENSITY = 30 # how old a frame:
# 30@10hz, 15@10hz, 5@10hz, 3@10hz, 30@100hz
DELAY_SECONDS = 2
ATTACK_START_TIME = 19.0 # seconds into the sim when attack starts
ATTACK_ACTIVE = False
