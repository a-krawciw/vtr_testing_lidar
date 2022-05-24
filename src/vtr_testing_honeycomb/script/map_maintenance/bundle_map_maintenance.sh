#!/bin/bash

## Automatically run odometry and localization on all the sequences
MODULE=$1

## Define the following environment variables VTH=VTR Testing honeycomb
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
export VTRHDATA=${VTRDATA}/utias_parkinglot
export VTRHRESULT=${VTRTEMP}/testing/utias_parkinglot
mkdir -p ${VTRHRESULT}

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

for RUN_ID in {0..30}; do
  echo "[COMMAND] run id ${RUN_ID}"
  bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/map_maintenance.sh ${MODULE} ${RUN_ID}
  if [[ $? -ne 0 ]]; then
    break
  fi
done
