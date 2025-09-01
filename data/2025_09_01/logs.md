TODO:


1. rerun 2/20 prev on route 2. downhill caused the collision at end.
2. rerun 1/20 0.5/20 prev on route 2

3. run 0.75/10 , 1.5/10 for both

3. run fallback strategy




------> 20 sec


r1  - prev 
    - 2/20
    - rou2
    - no colls!
    - 5.26pm

r2  - prev
    - 1/20
    - rou2
    - 5.30pm

r3  - prev
    - 0.5/20
    - rou2
    - 


rXX - pred
    - 2/20
    - rou2
    - 7.22pm

------> 10 sec


r4  - pred 
    - 0.75/10
    - rou1
    - 5.38pm
    - no colls

r5  - prev
    - 0.75/10
    - rou2
    -5.41
    - NO COLLS

r6  - pred 
    - 1.5/10
    - rou1
    - 5.43pm
    - no coll

r7  - prev
    - 1.5/10
    - rou2
    - 5.46pm
    - no colls

-----------------------
-------> fallback controller

---> CARRY mode [no colls unless noted here]

r8  - prev+fallback
    - rou2
    - 0.1/10
    - 5.59pm

r9  - prev+fallback
    - rou2
    - 0.2/10
    - 6.02pm

r10  - prev+fallback
    - rou2
    - 0.5/10
    - 6.05

r11  - prev+fallback
    - rou2
    - 0.75/10
    - 6.09pm

r12  - prev+fallback
    - rou2
    - 1/10
    - 6.12pm

r13  - prev+fallback
    - rou2
    - 1.5/10
    - 6.15pm

r14  - prev+fallback
    - rou2
    - 2/10
    - 1 close call
    - 6.17pm

r14a - prev+fallback
    - rou1
    - 2/10
    - 1 mid coll, 1 end coll
    - 6.23pm

r15  - prev+fallback
    - rou2
    - 2/20
    - 1 mid coll
    - 6.20pm


r16  - prev+fallback
    - rou2
    - 1/20
    - 6.26pm

r17  - prev+fallback
    - rou2
    - 0.5/20
    - 6.29pm


r18  - prev+fallback
    - rou2
    - 2/22
    - mid coll (gap -0.02)
    - 6.31pm

r19  - prev+fallback
    - rou2
    - 2/19
    - mid coll (-0.05)
    - 6.34pm



---> STOP mode

r20  - prev+fallback
    - rou2
    - 0.1/10
    - 6.37pm

r21  - prev+fallback
    - rou2
    - 0.2/10
    - 6.40pm

r22 - prev+fallback
    - rou2
    - 0.5/10
    - 6.43pm

r23  - prev+fallback
    - rou2
    - 0.75/10
    - 6.46pm

r24  - prev+fallback
    - rou2
    - 1/10
    - 6.48pm

r25  - prev+fallback
    - rou2
    - 1.5/10
    - 6.51pm

r26 - prev+fallback
    - rou2
    - 2/10
    - 6.54pm
 

r27  - prev+fallback
    - rou2
    - 2/20
    - 6.57pm


r28  - prev+fallback
    - rou2
    - 1/20
    - 7pm

r29  - prev+fallback
    - rou2
    - 0.5/20
    - 7.02pm


r30  - prev+fallback
    - rou2
    - 2/22
    - 7.05pm

r31  - prev+fallback
    - rou2
    - 2/19
    - 7.08pm


 ---------------------------------

 ----->  Base PRed with Intention Fallback.

r  - pred_prevfallback
    - rou2
    
    - 0.1/10
    - 

r  - pred_prevFallback
    - rou2
    - 0.2/10
    - 

r - pred_prevFallback
    - rou2
    - 0.5/10
    - 

r3  - pred_prevFallback
    - rou2
    - 0.75/10
    - 

r  - pred_prevFallback
    - rou2
    - 1/10
    - 

r  - pred_prevFallback
    - rou2
    - 1.5/10
    - 

r - pred_prevFallback
    - rou2
    - 2/10
    - 
 

r  - pred_prevFallback
    - rou2
    - 2/20
    - 


r  - pred_prevFallback
    - rou2
    - 1/20
    - 

r  - pred_prevFallback
    - rou2
    - 0.5/20
    - 


r  - pred_prevFallback
    - rou2
    - 2/22
    - 

r  - pred_prevFallback
    - rou2
    - 2/19
    - 
