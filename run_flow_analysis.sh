#!/bin/bash
veh_num=0
controller="NN"
for i in {1..25}; do
    echo "Running iteration $i"
    veh_num=$((i * 5))
    echo "Using controller: $controller"
    echo "Number of vehicles: $veh_num"
    python3 scripts/traffic_flow_model_testing.py --num_sv $veh_num $controller
    if [ $? -ne 0 ]; then
        echo "Error occurred during iteration $i"
        exit 1
    fi
done
