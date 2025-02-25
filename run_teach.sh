#!/usr/bin/env bash

source $VTRSRC/main/install/setup.bash
source $VTRROOT/vtr_testing_lidar/install/setup.bash


graph_dir=$VTRROOT/ghost_planning/graphs


trap exit SIGINT


ros2 run vtr_test_lidar process_sequence --ros-args --params-file $VTRROOT/vtr_testing_lidar/src/vtr_test_lidar/config/ouster_lasersam_test.yaml \
	-p sequence_dir:=$VTRDATA/FinalDataset/MarsLoop \
	-p data_dir:=$graph_dir \
	-p model_dir:=$VTRMODELS \
	-p name:=MarsLoop \
	-p storage_type:="sqlite3"


