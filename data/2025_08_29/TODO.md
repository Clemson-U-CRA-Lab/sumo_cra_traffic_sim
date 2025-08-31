TODO:


# ANALYSIS

Why r37 not match the indoor vil rerun?
    - mpc freq 20 hz, but sim updates 10 hz. So thats diffeenet between indoorVIL and outdoorVIL
    - **The braking in sumo is not realized as harsh. Because of this, it collides in simVIL but not on vehicle because I see som epretty harsh braking on vehicle.**
    - Can make it collide if low dec magnitue allowed at ulc. - hacky way
    - but if preview also colliding in real - then this wont get he results i need.
    - 

- I think the way I ran the attack delays form sim2V script -- i ran all attakcs wrong. it doesnt make sense.


- There is no use sending from RSPC at more than 10 hz, becasuse sim is at 10 hz.
- 

    DO:
1. doe sdelay in sim2V actually delays the vehicle?
2. Does fllback startegy help?



todo


1. run sendDealy1 with sumo2vv4_nv1 on vehicle. in sim it looks like it runs into the fornt ehicle for pred. for 2 second delay.
    - not run for preview in sim yet.
2. if it doenst collide  on real vehicle, limit decel on mache.

