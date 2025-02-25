#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source $VTRSRC/main/install/setup.bash
source $VTRROOT/vtr_testing_lidar/install/setup.bash


graph_dir=$VTRROOT/ghost_planning/graphs

#data splits



trap exit SIGINT


ros2 run vtr_test_lidar vtr_test_lidar_create_costmap --ros-args --params-file $VTRROOT/vtr_testing_lidar/src/vtr_test_lidar/config/ouster_lasersam_test.yaml \
	-p data_dir:=$graph_dir \
	-p model_dir:=$VTRMODELS \
	-p name:=MarsLoop



