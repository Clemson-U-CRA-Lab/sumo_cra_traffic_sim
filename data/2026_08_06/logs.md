6 aug 2026
VIL runs final for TVT - R1 

scripts used: 
- sumo2V_v5_nvN.py
- rosrun mach_e_control pcc_mpc_node.py or pcc_mpc_node_fallback.py
rosrun veh2x_interface v2x_Udp_periodic_node.py


Run Order: 

3 veh - pred

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


3 veh - prev

2 veh - pred

2 veh - prev

2 veh - fbStop

3 veh  - fbStop



