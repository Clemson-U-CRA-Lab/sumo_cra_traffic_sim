TODO:


1. periodic interface on both sides

    DIRECT:
    Attack and no attack :
        - 10 hz comms
        - 100hz comms
        - all frame elay levels
        - pred, preview


    via COHDA:
    Attack and no attack:
        - 10, 100 hz comms
        - all frame delays
        - ped, preview
    
2. async on rspc side.




LOGS:

# with cohda

10 hz -->

r1 -  pred, 10hz, attack 30 frames
    15.40pm
        - collided

r6 - r1 rerun
    - 2 colls
    - 4.06pm
    - pretty similar to r1


r2 - pred, no attack, 3.48pm

r3 - preview, no attack, 3.54

r4- preview, attack 30@10hz, 3.58pm
    - 2 collided

r5 - r4 rerun
    - 2 colls
    - 4.03pm
    - similar to r4

r7 - r1 rerun
    - attack 30@10
    - 4.15pm
    - 2 colls

100 Hz -->

r8 - pred
    - attack 30@100
    - 2 colls
    - 4.19pm

r9 - preview
    -attack 30@100
    - 2 colls
    - 4.22pm

r10 - preview
    - no attack
    - 100hz
    - no colls
    - 4.30pm

r11 - pred
    - no attack
    - 100hz
    - no colls
    - 4.33pm


r12 - pred, 20@100
    - 2 colls
    - 4.45pm

r13 - pred, 10@100
    - 1 small coll
    - 4.47pm

r14 - pred, 50@100
    - 4 colls
    - 4.51pm

r15 - preview, 20@100
    - 4.54pm
    - 2 colls

r16 - preview, 10@100
    - 2 small coll
    - 5pm

r17 - preview, 50@100
    - 5.03pm
    - 2 colls

10 hz -->

r18 - preview 3@10
    - no colls
    - 5.08pm

r19 - preview 5@10
    - no colls
    - 5.13pm

r20 - preview 10@10
    - no colls
    - 5.17pm

r21 - pred 3@10
    - no colls
    - 5.20

r22 - pred 5@10
    - no colls
    - 5. 23pm

r23 - pred 10@10
    - 1 coll small
    - 5.26pm

r24 - pred 15@10
    - no colls
    - 5.30pm

r25 - pred 20@10   
    - 2 colls
    - 5.33pm

r26 - preview 20@10
    - 2colls
    - 5.35pm    

# direct

10 Hz -->

r27 - preview
    - no attack
    - no colls
    - 5.46

r28 - preview
    - 20@10 attack
    - 1 small coll, 1  big coll
    - 5.58

r29 - preview
    - 15@10 attack
    - no colls
    - 6.02pm

r30 - preview
    - 5@10 attack
    - no colls
    - 6.06pm
    
r31 - preview
    - 2@10 attack
    - 6.08pm

r32 - pred
    - NO attack
    - 6.12

r33 - pred
    - 20@10 attack
    - no coll
    - 6.25pm

r34 - pred, 15@10
    - no coll
    - 6.28pm

r35 - pred, 10@10
    - no colls
    - 6.32pm

r36 - pred, 5@10
    - no colls
    - 6.36pm

100 Hz -->

r37 - pred, 50@100
    - 2 colls
    - 6.41pm

r38 - pred, 30@100
    - 2 colls
    - 6.44pm

r39 - preview, 50@100
    - 2 colls
    - 6.47pm

r40 - preview, 30@100
    - 2 colls
    - 6.50pm

 50Hz -->

r41 - preview, 30@50
    - 2 colls
    - 6.56pm

async interface --->

r42 - preview
    - no attack
    - no colls
    - 7pm

r43 - preview
    - 30 frames
    - 7.04pm
    - 2 colls

r44 - preview
    - 20 frames
    - 2 colls
    - 7.14pm

r45 - preview
    - 10 frames
    - no colls
    - 7.17pm

rXX - preview
    - 5 frames

r47 - pred
    - 30 frames
    - 2 colls
    - 7.20pm

r48 - pred
    - 10 frames
    - no colls
    - 7.23pm
    - - so no need to test 20 becuase this anyway beats preview


---> both sides async

r49 - pred
    - no attack
    - no colls


r50 - pred
    - 20 frames attack
    - no colls



# RERUN indoors
Direct -->

30@50 pred - 2 colls

50@100 pred 
    - 2 colls
    - (same as r37 but in sim to erify)

30@100 pred 
    - 2 colls
    - (same as r38 but in sim to erify)
