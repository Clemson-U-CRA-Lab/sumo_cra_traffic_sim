SIM_ARRAY_SIZE = 71  # Define the size of the array
VEH_ARRAY_SIZE = 68 # 7 for simple msg
BYTE_SIZE = 4 #8 tested with obu/rsu

# TCP Socket Setup
SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address (modify as necessary)
SERVER_PORT = 7002  # Server port
TIMEOUT = 5  # Timeout

MESSAGE_BYTE_LENGTH = BYTE_SIZE*VEH_ARRAY_SIZE

# same machine testing only
# SERVER_IP = 'localhost'
# SERVER_PORT = 7005


# STEP LENGTH: Sumo Time and rospy time:
SIM_STEP = 0.05
