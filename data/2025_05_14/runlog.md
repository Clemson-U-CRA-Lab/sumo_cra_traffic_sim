r1
    - baseline with stall @ 45 sec
    - SIMT-STEP for AccIntegrateDT in sim side

obd logging

r2
    - r1 without stall
    - MPC dt for integrate
    - 9.29am
    - why did it overtake?

r3:
    - attack r2
    - 300 sockets: connection was reset after a while. not immediately after attack
    - 9.33am
    - comms cut out

r4:
    - 200 sockets: didnt cut off thorugout
    - 9.39am

r5: 
    - repeat r2
    - 9.43am

r6:
    - 300 sockets
    - immediate attack launch
    - some connections were rest throughout.. but delays actually reduce..?
    - same behavior as in r3
    - 9.47am

r7:
    - 500 sockets
    - 9.52am
    - some resets midway

r7-repeat
    - 9.55
    - some resets midway
    - many timed out at the end

r8:
    - 750 sockets
    - 9.59
    - immediate- rsu reports too many open files
    - immedite - erno 104 connetion reset by peer seen on rspc attack, then timed out
    - comms continue throughout
    -

r8-rep
    - 10.02
    - 

r9:
    - 950 sockets
    - - immediate- rsu reports too many open files
    - immedite - erno 104 connetion reset by peer seen on rspc attack, then timed out

r10:
    - r9 but wiht ulimit 2000 at rsu
    - 10.14am
    - only 4 conn resets seen at attack side, in end, timed out for a bunch
    - no error snoticed on rsu side

r11:
    - 1900 socks
    - 10.17am - garbage
    - 10.20
    - RSU-OBU commms stop within a few seconds
    - too many files open
    - connections reset then timed out
    - no csv

r12:
    - r11 repeat
    - but rsu ulimit 3000, attack uliimit 10k
    - 10.25
    - sam ebehavior.. 
    - runs for 10 seconds then obu-rsu comms 
    - no csv

r13:
    - 950 sock
    - 10.28
    - runs for a bit then obu-rsu cuts off
    - no csv

r13:
    - 750 socks
    - 10.32
    - rsu-obu cuts off
    - no csv

r14:
    - 500 socks
    - 10.34
    - runs

r15:
    - 600 socks
    - 10.37
    - rsu-obu cuts off after a few sconds nicely.
    - no dcsv

r16:
    - 400 socks
    - 10.39

r17:
    - 550 socks
    - 10.47
    - with runtime csv
    - time outs only at end

r18:
    - r15 with runtime csv
    - 10.51
    - runs for some reasn..

r19:
    - with runtime csv
    - 900 socks
    - 10.54

r20:
    - with runtime csv
    - 1800 socks
    - 10.57
    - fails! finally..



