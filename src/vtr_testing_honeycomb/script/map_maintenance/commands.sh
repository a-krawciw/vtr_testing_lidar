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

## Terminal Setup (Run Following Once)

## Define the following environment variables VTRH=VTR Honeycomb
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
export VTRHDATA=${VTRDATA}/utias_parkinglot
export VTRHRESULT=${VTRTEMP}/testing/utias_parkinglot
mkdir -p ${VTRHRESULT}

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=rosbag2_2021_08_23-09_25_37
NUM_FRAMES=1700

LOC_INPUT=rosbag2_2021_08_23-09_57_54

## Some use flags
##   --prefix 'gdb -ex run --args'

## Using ONE of the following commands to launch a test
# (TEST 1) Perform data preprocessing on a sequence (e.g. keypoint extraction)
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/test_preprocessing.sh ${ODO_INPUT}
# (TEST 2) Perform odometry on a sequence
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/odometry.sh ${ODO_INPUT} ${NUM_FRAMES}
# (TEST 3) Perform localization on a sequence (only run this after TEST 2)
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT}

# (TEST 4) Intra Exp Merging
MODULE=intra_exp_merging
RUN_ID=0
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/map_maintenance.sh ${MODULE} ${RUN_ID}

# bundled version
MODULE=intra_exp_merging && bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/bundle_map_maintenance.sh ${MODULE}
MODULE=dynamic_detection && bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/bundle_map_maintenance.sh ${MODULE}
MODULE=inter_exp_merging && bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/bundle_map_maintenance.sh ${MODULE}

# plot global map
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/plot_map_maintenance.sh
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/plot_memap_maintenance.sh
