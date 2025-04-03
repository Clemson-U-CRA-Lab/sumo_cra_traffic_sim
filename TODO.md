TODO:

7. multiple tcp sockets
    - threading for multiple client.
    - Lets target RSU server to be attacked.
    - threading needs to be in C code to handle connect clients?

1. sync is issue - maybe syncedSocketInterface is better to maintian speed of execution
        - async or sync SockInt node? - theyboth work in sim.
        - sync socketInt will probably work better in vehicle
        - synced socket also works ok on vehicle.



                # lead vehicle pred: 0.1 sec = SIM_STEP, 
        # Correct preview to lead vehicle. OK ll good 
        # when the preds are passed to 2nd vehicle mpc ... something goes wrong.. PRED or PREVIEW?
        # MPC output predicts in 0.5 dt step ---WWRONG

        - input t traj has wrong time
        - horizon is too short