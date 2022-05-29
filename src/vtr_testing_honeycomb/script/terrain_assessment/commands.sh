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

## changes to the configs:
# 1. map_voxel_size in config file set to 0.1 - so that we get better estimate of normals

## Terminal Setup (Run Following Once)

## Define the following environment variables VTRH=VTR Honeycomb
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
export VTRHDATABASE=${VTRDATA}/utias_multiple_terrain
export VTRHRESULTBASE=${VTRTEMP}/testing/utias_multiple_terrain
mkdir -p ${VTRHRESULTBASE}

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

## Choose one from the following

# parking lot
export VTRHDATA=${VTRHDATABASE}/parkinglot
export VTRHRESULT=${VTRHRESULTBASE}/parkinglot
mkdir -p ${VTRHRESULT}
NUM_FRAMES=10000
ODO_INPUT=rosbag2_2021_11_01-18_05_58
LOC_INPUTS=(
  rosbag2_2021_11_01-18_10_03
  rosbag2_2021_11_01-18_14_04
  rosbag2_2021_11_01-18_18_34
)

# mars dome
export VTRHDATA=${VTRHDATABASE}/marsdome
export VTRHRESULT=${VTRHRESULTBASE}/marsdome
mkdir -p ${VTRHRESULT}
NUM_FRAMES=10000
ODO_INPUT=rosbag2_2022_01_30-22_34_29
LOC_INPUTS=(
  rosbag2_2022_01_30-22_36_16
  # rosbag2_2022_01_30-22_38_28  # localization fails for this run do not use it
)

# grove
export VTRHDATA=${VTRHDATABASE}/grove
export VTRHRESULT=${VTRHRESULTBASE}/grove
mkdir -p ${VTRHRESULT}
NUM_FRAMES=10000
ODO_INPUT=rosbag2_2021_12_30-14_31_27
LOC_INPUTS=(
  rosbag2_2021_12_30-14_37_51
  rosbag2_2021_12_30-14_44_45
  rosbag2_2021_12_30-14_54_14
  rosbag2_2021_12_30-15_00_37
  rosbag2_2021_12_30-15_07_01
)

## Use the following command to run odometry and localization

# Run odometry
echo "[COMMAND] run odometry"
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/odometry.sh ${ODO_INPUT} ${NUM_FRAMES}

# Run localization
for LOC_INPUT in "${LOC_INPUTS[@]}"; do
  echo "[COMMAND] running localization with input ${LOC_INPUT}"
  bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/localization.sh ${ODO_INPUT} ${LOC_INPUT}
done

# Run map maintenance
MODULES=(
  intra_exp_merging
  dynamic_detection
  inter_exp_merging
)
for MODULE in "${MODULES[@]}"; do
  for RUN_ID in {0..30}; do
    echo "[COMMAND] run id ${RUN_ID}"
    bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/map_maintenance.sh ${MODULE} ${RUN_ID}
    if [[ $? -ne 0 ]]; then
      break
    fi
  done
done

# Plot global map
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/plot_map_maintenance.sh
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/map_maintenance/plot_memap_maintenance.sh

# Run terrain assessment
# remember to set number of fake obstacles and types of obstacles in the config file, for now we use:
#   objs: ["human", "cone", "box_small", "box_large"]
#   rand_objs: 20
#   rand_xrange: [-0.0, 10.0]
#   rand_yrange: [-5.0, 5.0]
#   rand_zrange: [-0.5, -0.1]
MODULE=change_detection_fake
RUN_ID=0
bash ${VTRHROOT}/src/vtr_testing_honeycomb/script/terrain_assessment/terrain_assessment.sh ${MODULE} ${RUN_ID}

# Plot terrain assessment
export VTRHROOT=/home/yuchen/ASRL/vtr_testing_lidar
source ${VTRHROOT}/install/setup.bash
source /home/yuchen/ASRL/venv/bin/activate
# this needs results without obstacles (assume folder named main.no_fake_obstacle)
python /ext0/ASRL/vtr_testing_lidar/scripts/plot_terrain_roughness_slope.py \
  --path /home/yuchen/ASRL/temp/testing/utias_multiple_terrain \
  --dest /ext0/ASRL/vtr_testing_lidar/scripts/results
# this needs results with obstacles (assume folder named main.fake_obstacle)
python /ext0/ASRL/vtr_testing_lidar/scripts/plot_terrain_assessment_pr.py \
  --path /home/yuchen/ASRL/temp/testing/utias_multiple_terrain \
  --dest /ext0/ASRL/vtr_testing_lidar/scripts/results
