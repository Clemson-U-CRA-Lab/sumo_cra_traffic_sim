#!/bin/bash
veh_num=0
leading_vehicle_profile="Hwy"
echo "Enter controller type (MPC, IDM, NN): "
read controller
for i in {1..30}; do
    echo "Running iteration $i"
    veh_num=$((i * 10))
    echo "Using controller: $controller"
    echo "Number of vehicles: $veh_num"
    python3 scripts/run_sim.py --num_sv $veh_num $leading_vehicle_profile $controller 
    if [ $? -ne 0 ]; then
        echo "Error occurred during iteration $i"
        exit 1
    fi
    clear
done
