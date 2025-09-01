LOGS

DIRECT --->
10hz COMMS AT RSPC, 20hzcomms at vheicle
sumo2V_v4_nv1.py

ULC max acc = 2.0, max decel = 2.0

            r1
                - pred
                - no attack
                - 2.06PM
                - 60 sec


r2  - pred
    - no attack
    - no colls
    - 90 sec
    - 2.11pm

r3 - pred
    - attack 2sec, start at 20sec (2/20)
    - 2.15pm
    - 1 coll

r4  - preview
    - no attack 
    - 2.20pm
    - no colls
    
r5  - prev
    - attack 2/20
    - 2.24
    - 1 coll

            r6  - prev
                - rerun r5 with ulc decel 5, same behavior
                - 2.28pm


            r7  - rerun r3
                - 2.35pm
                - 1 coll at end (larger than r3)

r8  - pred 
    - 1/20
    - 2.38pm
    - no coll

r9  - pred
    - 0.5/20
    - 2.41pm
    - no colls

r10 - prev
    - 1/20
    - no colls
    - 2.45pm

r11 - prev
    - 0.5/20
    - no colls
    - 2.48pm


-----> at 10 sec -- use all of these

r12 - prev
    - 2/0
    - 2 colls. first mild
    - 2.51pm

r13 - prev
    - 2/10
    - 1 coll at end [DUE to downhill]
    - 2.54pm

            r14 - r13 rerun
                - 2.58pm

r15 - r14 but route 2
    - prev, 2/10
    - no colls!! <--
    - 3.04pm

r16 - pred
    - 2/10
    - route2
    - 2 colls (mid, end)
    - 3.24pm

r17 - pred
    - 2/10
    - route1
    - 1 coll at end
    - 3.39pm

            r18 - rerun r16
                - 1 coll at end
                - 3.44pm

r19 - pred
    - 1/10
    - route 1
    -   3.50pm
    - no colls

r19 - pred
    - 1/10
    - route 2
    -  3.53 pm
    - no colls

r20 - pred
    - 0.5/10
    - rout 1
    - 3.55pm
    - no colls

r21  - prev
    - rou2
    - 1/10
    - 3.59pm
    - no colls

r22 - prev
    - 0.5/10
    - rou1
    - 4.02pm
    -  no colls

r23 - prev
    - 0.5/10
    - rou2
    - 4.05pm
    - no colls


r24 - prev
    - 0.2/10
    - rou2
    - no colls
    - 4.21pm

r25 - pred
    - 0.2/10
    - rou1
    - 4.25pm
    - no colls

r26 - prev
    - 0.1/10
    - rou2
    - 4.28PM
    - NO COLLS

r27 - pred
    - 0.1/10
    - rou1
    - no colls
    - 4.30pm


-----> at 22 sec (use these to reinforce)
   
r28 - pred
    - 2/22
    - rou2
    - no colls, 2 close calls
    - 4.34pm


r29 - preview
    - 2/22
    - rou2
    - no close calls
    - 4.37pm

----> at 19 sec (use these to reinforce)

r30 - pred
    - 2/19
    - rou1
    - 1 coll at end
    - 4.42pm


r31 - preview
    - 2/19
    - rou2
    - no coll (may have touched at 91 seconds)
    - 4.45pm



    
TODO:


1. rerun 2/20 prev on route 2. downhill caused the collision at end.
2. rerun 1/20 0.5/20 prev on route 2

3. run 0.75/10  for both

3. run fallback strategy

