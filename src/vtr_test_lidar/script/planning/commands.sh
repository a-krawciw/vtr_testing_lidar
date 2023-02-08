echo "This script contains instructions to run all tests, do not run this script directly."
exit 1

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash                   # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/honeycomb.rviz # launch rviz

## Then in another terminal, launch rqt_reconfigure for control
## current supported dynamic reconfigure parameters: control_test.play and controL_test.delay_millisec
source /opt/ros/galactic/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure

############################################################
#### Now start another terminal and run testing scripts ####

## NOTE: for all experiments below, change the following in config file:
#    1. map_voxel_size in config file set to 0.1 - so that we get better estimate of normals

## Terminal Setup (Run Following Once)

## Define the following environment variables VTRH=VTR Honeycomb
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
export VTRHDATA=${VTRDATA}/utias_multiple_terrain/parkinglot
export VTRHRESULT=${VTRTEMP}/testing/utias_multiple_terrain/parkinglot_planning
mkdir -p ${VTRHRESULT}

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

## First use commands from terrain_assessment/commands.sh to generate a map of parking lot dataset using ODO_INPUT below!!
## Then copy the generated results from utias_multiple_terrain/parkinglot to utias_multiple_terrain/parkinglot_planning
## config file:
##   enable the safe corridor module

# parking lot
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUT=rosbag2_2021_11_01-18_10_03

# Run localization
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/planning/planning.sh ${ODO_INPUT} ${LOC_INPUT}
