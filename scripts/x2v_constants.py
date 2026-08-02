####################################
# COMMUNICATIONS SETUP
####################################
_comms_type = "UDP" # "TCP" or "UDP"
_test_type = "VIACOHDA" # "SAME MACHINE" or "DIRECT", "VIACOHDA"

if _comms_type == "TCP":
    if _test_type == "SAMEMACHINE":
        # same machine testing only
        SERVER_IP = 'localhost'  # RSU IP address
        SERVER_PORT = 7002
    if _test_type == "DIRECT":
        # for direct to nuvo
        SERVER_IP = 'fe80::e3a4:179b:896b:ad81'
        SERVER_PORT = 7003
    elif _test_type == "VIACOHDA":
        SERVER_IP = 'fe80::6e5:48ff:fe30:0820'  # RSU IP address
        SERVER_PORT = 7002  # Server port
    else:
        raise ValueError("Invalid test type. Choose valid test conneciton chain type")

    # TCP Socket Setup
    TIMEOUT = 5  # Timeout
    INTERFACE_SCOPE_ID = 6 # for ipv6 - update evey time you connect the cable.

elif _comms_type == "UDP":
    # udp
    if _test_type == "SAMEMACHINE":
        # same machine testing only
        RSPC_IPV4 = '192.168.74.170'
        TARGET_IP = '192.168.74.170'
        RX_UDP_PORT = 9101 # RSPC recvs from MAHCE on this.
        TX_UDP_PORT = 9102 # RSPC sends to this on mache.
    elif _test_type == "DIRECT":
        RSPC_IPV4 = '192.168.74.170'
        MACHEPC_IPV4 = '192.168.74.169'
        TARGET_IP = MACHEPC_IPV4 # for direct conection.
        RX_UDP_PORT = 9101 # RSPC recvs from MAHCE on this.
        TX_UDP_PORT = 9102 # RSPC sends to this on mache.
    elif _test_type == "VIACOHDA":   
        RSPC_IPV4 = '192.168.74.170'
        RSU_IPV4 = '192.168.74.200'
        TARGET_IP = RSU_IPV4
        # UDP Socket Setup
        RX_UDP_PORT = 9004 # rsu recvs from obu on this
        TX_UDP_PORT = 9002 # rsu sends to obu from this

    TIMEOUT = 5  # Timeout

else:
    raise ValueError("Invalid comms type. Choose 'TCP' or 'UDP'.")


#####################################
# RUN SETUP
#####################################


# indoor or outdoor VIL?:
BOOL_TEST_WITHOUT_GPS = True

# SUMO params and run params
SIM_STEP = 0.1
END_TIME = 90.0 #90
SUMO_ACC_INTEGRATE_DT = 3.0 # for traci.setAcceleration() in sumo
SUMO_CONFIG = "v2x_2veh.sumocfg" # which config to use

# Ref for front vehicle
REF_CYCLE_DT = 0.2
REF_CYCLE_STAGES = 32 # 32, 100
# 0.1, 100 works well. 160 makes 16 second ghorizon ref.

# MPC config - dont change DT
MPC_DT = 0.5 # dont change
MPC_REF_STAGES = 32 
BOOL_USE_FRONT_PREVIEW = False   # use preview of front's intention for ego's mpc?
# If True, its intention sharing , if False, setPred() is used

# For V2X and X2V interfaces
COMMS_FREQ = 10 #Hz - No use going above 1/SIM_STEP frequency.

SIM_ARRAY_SIZE = 7+2*REF_CYCLE_STAGES  # Define the size of the array
VEH_ARRAY_SIZE = 68 # 7 for simple msg
# for 32 long horizon, sim_array_size = 71, veh_arr_size = 68.
# for 100 stages, sim_array_size = 207
BYTE_SIZE = 4 #8 tested with obu/rsu
MESSAGE_BYTE_LENGTH = BYTE_SIZE*VEH_ARRAY_SIZE

# Stalling nv0 manually
DEMO_STALL_NV0 = False
STALLTIME = 831.0
STALLENDTIME = STALLTIME + 5.0

# Attack details
BOOL_ATTACK = False
ATTACK_TYPE = "SPOOF_ENERGENCY" # "REPLAY", "CUTOFF", "SPOOF_EMERGENCY"
ATTACK_START_TIME = 31.0 # seconds into the sim when attack starts
ATTACK_END_TIME = ATTACK_START_TIME + 15.0
ATTACK_ACTIVE = False

# ATTACK_INTENSITY = 30 # how old a frame: - 
# 30@10hz, 15@10hz, 5@10hz, 3@10hz, 30@100hz (for ITS paper, v4.py)
DELAY_SECONDS = 0.0 # for tcp only
