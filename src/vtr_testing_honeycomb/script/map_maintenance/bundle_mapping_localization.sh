#!/bin/bash

## Automatically run odometry and localization on all the sequences

## Define the following environment variables VTH=VTR Testing honeycomb
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
export VTRHDATA=${VTRDATA}/utias_parkinglot
export VTRHRESULT=${VTRTEMP}/testing/utias_parkinglot
mkdir -p ${VTRHRESULT}

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

ODO_INPUT=rosbag2_2021_08_23-09_25_37
NUM_FRAMES=1700

LOC_INPUTS=(
  ## First set of runs
  rosbag2_2021_08_23-10_26_27
  rosbag2_2021_08_23-11_29_30
  rosbag2_2021_08_23-12_58_58
  rosbag2_2021_08_23-13_29_07
  rosbag2_2021_08_23-14_28_57
  rosbag2_2021_08_23-15_29_05
  rosbag2_2021_08_23-16_30_52
  rosbag2_2021_08_23-16_59_14
  rosbag2_2021_08_23-17_58_24
  rosbag2_2021_08_23-18_56_46
  ## Second set of runs
  rosbag2_2021_08_23-09_57_54
  rosbag2_2021_08_23-10_55_40
  rosbag2_2021_08_23-11_21_12
  rosbag2_2021_08_23-11_58_52
  rosbag2_2021_08_23-14_58_51
  rosbag2_2021_08_23-15_59_26
  rosbag2_2021_08_23-18_30_31
  ## Third set of runs
  rosbag2_2021_08_25-19_04_35
  rosbag2_2021_08_25-19_31_44
  rosbag2_2021_08_25-19_57_09
  rosbag2_2021_08_25-20_15_07
  rosbag2_2021_08_25-20_31_50
  rosbag2_2021_08_25-20_56_58
  ## The following two are not used in the paper
  # rosbag2_2021_08_23-12_27_23 # odometry bad, end location different
  # rosbag2_2021_08_23-13_58_45 # odometry bad, end location different
  # rosbag2_2021_08_23-17_28_49 # odometry bad, end location different
  # rosbag2_2021_08_25-20_43_58 # long stop
  # rosbag2_2021_08_25-21_05_06 # odometry drift
  # rosbag2_2021_08_25-21_13_00 # odometry drift at the end
)

# Run odometry
echo "[COMMAND] run odometry"
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/odometry.sh ${ODO_INPUT} ${NUM_FRAMES}

# Run localization
for LOC_INPUT in "${LOC_INPUTS[@]}"; do
  echo "[COMMAND] running localization with input ${LOC_INPUT}"
  bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/localization.sh ${ODO_INPUT} ${LOC_INPUT}
done
