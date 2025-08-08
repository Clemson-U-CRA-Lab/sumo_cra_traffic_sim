# sumo_cra_traffic_sim
Workspace for SUMO-based traffic simulation framework


Scripts:

1. runSim.py
1. sumo2V_v0_4floats
2. sumo2V_v1, sumo2V_v1_nv1
3. sumo2V_v2, sumo2V_v2_nv1, 
    sumo2V_v2_nv1_sim - for testing indoors without gps
    sumoOnly_v2_nv1_sim - has no tcp

    ---> debugged mpc - 5 Aug 2025:
4. sumo2V_v3, sumo2V_v3_nv1
    sumo2V_v3_nv1 - testingwithoutGPS = true:  for testing indoors without gps wiht sumo
    sumoOnly_v3 - has no tcp - purely sumo - nv0, nv1
    sumoOnly_v3_3veh - has no tcp, purely sumo, no0,nv1,nv2
    sumoOnly_v3_nVeh - take snumber of cvehicles from sumo vehicle list, basedd on .cfg file `SUMO_CONFGIG`


TODO:
1. Pred, preview with new (eraleir it wasnt updating speed mode) chcks off.
2. 2 vehicle, 
2. 3 vehicle config - the mpc ref of 0.5 when passed t 3rd vehicle, does poorly. but if i pass 0.1 dt, then its fine..
3. with stall, without stall
4. with attack - 3 intensities.
