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
    sumoOnly_v3_nVeh - take snumber of cvehicles from sumo vehicle list, basedd on .cfg file `SUMO_CONFIG`

5. sumo2V_v4_nv1.py - 

6. sumo2V_v5_nv1.py - with UDP
    sumo2V_v5_nv1_demo - with UDP interface, with attack emulation.


TODO:
1. Pred, preview with new (eraleir it wasnt updating speed mode) chcks off.
2. 2 vehicle, 
3. 3 vehicle config - the mpc ref of 0.5 when passed t 3rd vehicle, does poorly. but if i pass in 0.1 dt, then its fine..
4. with stall, without stall
5. with attack - 3 intensities.

6. Preview tcp msg segmentation.delay with 100 length. - 
    - v2x rev loop - 
    - x2v recv loop - 
    - sendall()
    - sumo pace timer - sending truly periodic
    - v2x send in main loop instead of allback - will miss messages but will not see bursts. (OR is this desirable for my results?)
    - then test with atacks. and pcaps


    - threaded sends / recv are too good.. 

    - maybe just switch back to original interface and test on bench with 32*0.2 preview.
    - 

    - Try UDP
    - Try cv2x

