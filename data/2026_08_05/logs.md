# 06 aug 2026

3 vehicle runs with sumo2V_v5_nvN.py & dummy_samemachine_mpc_vehicle.py

x2v constants used:
BOOL_ATTACK = True
ATTACK_START_TIME = 10.0 # seconds into the sim when attack starts
ATTACK_ACTIVE = False
DELAY_SECONDS = 2.0 # for tcp , and now udp_sendDelay interface too.
SUMO_CONFIG = "v2x.sumocfg" # which config to use


Run order used for pred/prev/FbStop controllers:

runs:
    - Run X : DELAY_SEC/ATTACK_LAUNCH_TIME
    - run0: 0.0/10
    - run1: 0.1/10
    - run2: 0.2/10
    - run3: 0.5/10
    - run4: 0.75/10
    - run5: 1.0/10
    - run6: 1.5/10
    - run7: 2.0/10
    - run8: 0.5/20
    - run9: 1.0/20
    - run10: 1.5/20
    - run11: 2.0/20
    - run12: 2.0/19
    - run13: 2.0/22
