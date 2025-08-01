30 July 2025


on vehicle - stattionary:
r1 : 4.02pm sumo2V_v2_nv1.py , standind in lab

NUM_FAST = 100      # fast flooding threads
NUM_SLOW = 200      # slow-loris threads
ATTACK_DURATION = 3  # seconds to run attack

DELAY_BETWEEN_SENDS = 0.4  # flood delay (sec)
LORIS_INTERVAL = (2, 5)   # slow loris drip interval range (sec)

r1A: 4.05pm

on-vehicle runs: AT CMI

r2: cmi - 4.24pm, with pcap
r2w: cmi without pcap 4.29pm
r2_1: repaet it. 4.33pm

r2A: 4.39. 
r2wA: without pcap, with attack. 4.44

### should record llc channel pcap too.

r2_2: recorded two pcaps. - for eth0 and llc - 5.04pm
r2_2A: same but with attack, only 1 attack at 8sec 2 pcaps - 5.12pm



-> with preview. 2pcaps

r3: 5.22pm
r3w: no pcaps, 5.26pm
r3_1: 5.29pm, repeat becoz pdeal mess up.

r3A: 5.35pm, with 2 attacks
r3wA: 5.40pm , 2 attacks.


--> PRED

ATTACK_DURATION = 8  # seconds to run attack
NUM_SLOW 600
r4A: 5.45pm, 1 long attack

8 sec, 600, delay betwen sends 0.01
r5A: 5.56pm, 1 attck still

8 sec, 300 slows, delay 0.01
r6A: 6.03pm

8 sec, 300 slows, 300 fasts, delayy 0.01
r7A: 6.06pm