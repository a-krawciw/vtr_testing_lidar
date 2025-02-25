#!/usr/bin/env bash

source $VTRSRC/main/install/setup.bash
source $VTRROOT/vtr_testing_lidar/install/setup.bash


graph_dir=$VTRROOT/laser_vision/graphs
model_dir=$VTRROOT/laser_vision/checkpoints

#data splits
test_sequences=(ForestLoop)

mars_vids="['4294967311', '4294967665', '8589935218', '8589934607', '8589934721', '4294967829', '8589935116', '4294967625', '4294967796', '4294967850', '8589935060', '4294967700', '8589934845', '8589934691', '8589934830', '8589935192', '4294967610', '4294967431', '8589934896', '8589935054', '8589934630', '8589935261', '4294967799', '8589935042', '4294967457', '4294967831', '8589935240', '8589935004', '8589934824', '4294967896', '4294967852', '4294967407', '8589934803', '4294967656', '4294967785', '8589934984', '8589934911', '8589935247', '4294967956', '4294967947', '4294967903', '4294967332', '8589934756', '4294967783', '8589934994', '4294967752', '4294967576', '4294967746', '8589934857', '4294967475']"

long_loop_vids="['8589934763', '8589935010', '8589934655', '4294967668', '4294967812', '8589934851', '4294967971', '4294967583', '8589934878', '4294967922', '4294967360', '4294967586', '4294967758', '8589935218', '4294967762', '8589934816', '4294967611', '8589934598', '8589934729', '8589934985', '4294967579', '8589935126', '8589935266', '4294967696', '4294967693', '8589934653', '4294967492', '4294967950', '4294967444', '4294967545', '4294967646', '4294967712', '4294967650', '4294967851', '8589935090', '4294967761', '8589934808', '4294967722', '8589935261', '8589935078', '4294967846', '8589934738', '4294967843', '4294967496', '4294967665', '8589934893', '8589934920', '4294967568', '4294967333', '8589935076']"

trap exit SIGINT



#ros2 run vtr_test_lidar process_sequence --ros-args --params-file $VTRROOT/vtr_testing_lidar/src/vtr_test_lidar/config/ouster_lasersam_test.yaml \
	#-p sequence_dir:=$VTRDATA/ouster_dataset/MarsDay2 \
	#-p data_dir:=$graph_dir \
	#-p model_dir:=$VTRMODELS \
	#-p name:=MarsDay2 /
	#-p localization.cd_test.vertices_in_use:="$mars_vids"


#ros2 run vtr_test_lidar process_sequence --ros-args --params-file $VTRROOT/vtr_testing_lidar/src/vtr_test_lidar/config/ouster_lasersam_test.yaml \
#	-p sequence_dir:=$VTRDATA/ouster_dataset/LongLoop2 \
#	-p data_dir:=$graph_dir \
#	-p model_dir:=$VTRMODELS \
#	-p name:=LongLoop2 \
#	-p localization.cd_test.vertices_in_use:="$long_loop_vids"

ros2 run vtr_test_lidar process_sequence --ros-args --params-file $VTRROOT/vtr_testing_lidar/src/vtr_test_lidar/config/ouster_lasersam_test.yaml \
	-p sequence_dir:=$VTRDATA/ouster_dataset/LongLoop \
	-p data_dir:=$graph_dir \
	-p model_dir:=$VTRMODELS \
	-p name:=LongLoop \


