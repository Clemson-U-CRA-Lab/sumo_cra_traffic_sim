SIM_ARRAY_SIZE = 71  # Define the size of the array
VEH_ARRAY_SIZE = 68 # 7 for simple msg
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

# STEP LENGTH: Sumo Time and rospy time
SIM_STEP = 0.05
END_TIME = 60.0 #90

MPC_DT = 0.5
MPC_REF_STAGES = 32
BOOL_USE_FRONT_PRVIEW = False # use preview of front's intention for ego's mpc?
