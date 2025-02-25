#!/usr/bin/env bash

source $VTRSRC/main/install/setup.bash
source $VTRROOT/vtr_testing_lidar/install/setup.bash

map_voxels=(0.15)
live_voxels=(0.1)
sequences=(short_loop long_loop parking_lot mars_dome_indoor flat_dome far_loop library_lot garden_path)

graph_dir=$VTRROOT/laser_vision/graphs
model_dir=$VTRROOT/laser_vision/checkpoints

#data splits
train_sequences=(MarsLoop)
val_sequences=()
test_sequences=()

trap exit SIGINT


echo "Training"
for sequence in ${train_sequences[@]}; do

    for map_voxel_size in ${map_voxels[@]}; do 
	    for nn_voxel_size in ${live_voxels[@]}; do
	        (
	        echo "Map: " $map_voxel_size " Live: " $nn_voxel_size
	        
	        #Run VTR to generate pose graphs for the train / test sets.
	        #Note this only needs to be run once, changes to the neural network do not effect this stage.
	        graph_name=${sequence}-m${map_voxel_size}-f${nn_voxel_size}
	        echo $graph_name
	        
	        ros2 run vtr_test_lidar process_sequence --ros-args --params-file src/vtr_test_lidar/config/ouster.yaml \
	        -p sequence_dir:=$VTRDATA/ouster_dataset/$sequence \
	        -p data_dir:=$graph_dir \
	        -p name:=$graph_name \
	        -p "odometry.mapping.map_voxel_size":=$map_voxel_size \
	        -p "odometry.intra_exp_merging.map_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.frame_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.nn_voxel_size":=$nn_voxel_size \
	        -p "reversed_path":=false
	        )
	        

	    done
    done

done




