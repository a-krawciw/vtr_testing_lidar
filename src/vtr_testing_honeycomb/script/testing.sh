## Explain how to test lidar pipeline

## First launch RViz for visualization
source /opt/ros/galactic/setup.bash  # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/lidar.rviz  # launch rviz

## Now start another terminal and run testing scripts

## Define the following environment variables VTH=VTR Testing Honeycomb
export VTRHROOT=/ext0/ASRL/vtr_testing_honeycomb/src/vtr_testing_honeycomb
export VTRHDATA=${VTRDATA}/utias_20211230_multiple_terrain/parkinglot
export VTRHRESULT=${VTRTEMP}/testing/multienv
mkdir -p ${VTRHRESULT}

source ${VTRSRC}_utils/ros2/install/setup.bash

# parkinglot static
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUT=rosbag2_2021_11_01-18_10_03
# parkinglot with you
ODO_INPUT=rosbag2_2022_01_09-19_00_08
LOC_INPUT=rosbag2_2022_01_09-18_55_25
# backyard
ODO_INPUT=rosbag2_2021_12_30-14_31_27
LOC_INPUT=rosbag2_2021_12_30-14_37_51
# dome inside
ODO_INPUT=rosbag2_2022_01_30-22_34_29
LOC_INPUT=rosbag2_2022_01_30-22_36_16

# Source the VTR environment with the testing package
source ${VTRHROOT}/../../install/setup.bash

## Some use flags
##   --prefix 'gdb -ex run --args'



## Perform preprocessing on a sequence
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_preprocessing  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT}



## Perform odometry on a sequence
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_odometry  \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT}




## Copy-past for localization
rm -r ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}

## Perform localization on a sequence directly (with a specified point map version)
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRHDATA}/${LOC_INPUT}

## Perform localization + planning on a sequence directly (with a specified point map version)
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization_planning \
  --ros-args -p use_sim_time:=true -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRHDATA}/${LOC_INPUT}




## Perform offline tasks
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_dynamic_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_inter_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_terrain_assessment \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_intra_exp_merging \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_change_detection \
  --ros-args  -r __ns:=/vtr  --params-file ${VTRHROOT}/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p run_id:=1