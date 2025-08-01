run1 - 
    - pred for ego
    - acc from GPS is sent back to Sumo to assignAcc

run2
    - pred for ego
    - gps pos and spd was sent back to sumo and set in sumo "updateCAV..()"
    - sumo detected collision in gui - is it really? verfiy +1.75

run3 (run1 repeat to finish)
    - pred for ego
    - acc from GPS is sent back to Sumo to assignAcc

run4
    - preview for ego
    - spd pos from gpd to sumo - using this for all  next..

run5
    - pred for ego
    - with cohda

run6
    - preview for ego
    - withc cohda

run7
    - preview wfor ego
    - cohda rsu on pavement

run8
    - pred for ego
    - cohda on pavement

run9 
    - pred for ego
    - cohda pavement
    - accInt : MPC_DT

run10:
    - dos on run5

run11
    - run10 but later attack

run12:
    - run10 but another point of run is atacked.
    - basically last received positions keep 