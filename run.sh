#!/bin/bash

num_uav=10
# num_ue=30
num_bs=20

algorithm="iuavbs"

if [ $# -ne 0 ]
then
    algorithm=$1
else
  algorithm=${PWD##*/}          # to assign to a variable
fi

if [ $algorithm = "competing" ] || [ $algorithm = "iuavs" ]
then
    num_uav=10
elif [ $algorithm = "classic" ] || [ $algorithm = "none" ]
then
    num_uav=0
fi

sed -i -r "s/(numUAVs =) [0-9]{1,2}/\1 ${num_uav}/g" scratch/drones_and_stuff.cc
sed -i -r "s/(numStaticCells =) [0-9]{1,2}/\1 ${num_bs}/g" scratch/drones_and_stuff.cc
sed -i -r "s/(handover_policy =) \"\w+\"/\1 \"${algorithm}\"/g" scratch/drones_and_stuff.cc

for seed in $(seq 10)
do
    for num_ue in 30 60 90
    do
        sed -i -r "s/(numUes =) [0-9]{1,2}/\1 ${num_ue}/g" scratch/drones_and_stuff.cc
        
        ./waf --run "scratch/drones_and_stuff --seedValue=$seed" > simul_${algorithm}_${seed}_${num_ue} 2>&1
    done
done
