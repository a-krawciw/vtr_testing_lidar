#!/usr/bin/env bash

source $VTRSRC/main/install/setup.bash
source $VTRROOT/vtr_testing_lidar/install/setup.bash

map_voxels=(0.1 0.2 0.3)
live_voxels=(0.15)
sequences=(short_loop long_loop parking_lot mars_dome_indoor flat_dome far_loop library_lot garden_path)

graph_dir=$VTRROOT/range_paper/graphs
model_dir=$VTRROOT/range_paper/checkpoints

#data splits
train_sequences=(MarsLoop)
val_sequences=(LongLoop)
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
	        -p sequence_dir:=$VTRDATA/FinalDataset/$sequence \
	        -p data_dir:=$graph_dir \
	        -p name:=$graph_name \
	        -p "odometry.mapping.map_voxel_size":=$map_voxel_size \
	        -p "odometry.intra_exp_merging.map_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.frame_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.nn_voxel_size":=$nn_voxel_size \
	        -p "reversed_path":=false
	        )
	        
	        #python3 $VTRROOT/range_diff/src/train.py -g $graph_dir/$graph_name -o $model_dir/$graph_name
	        #mv $model_dir/$graph_name/checkpoint_epoch10.pth $model_dir/$graph_name/final.pth

	    done
    done

done

wait

echo "Testing"
for sequence in ${test_sequences[@]}; do

    for map_voxel_size in ${voxels[@]}; do 
	    for nn_voxel_size in ${voxels[@]}; do
	        echo "Map: " $map_voxel_size " Live: " $nn_voxel_size
	        
	        graph_name=${sequence}-m${map_voxel_size}-f${nn_voxel_size}
	        echo $graph_name
	        
	        ros2 run vtr_test_lidar process_sequence --ros-args --params-file src/vtr_test_lidar/config/ouster.yaml \
	        -p sequence_dir:=$VTRDATA/FinalDataset/$sequence \
	        -p data_dir:=$graph_dir \
	        -p name:=$graph_name \
	        -p "odometry.mapping.map_voxel_size":=$map_voxel_size \
	        -p "odometry.intra_exp_merging.map_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.frame_voxel_size":=$map_voxel_size \
	        -p "preprocessing.filtering.nn_voxel_size":=$nn_voxel_size
	        
	        python3 $VTRROOT/range_diff/src/evaluate.py -g $graph_dir/$graph_name -c $model_dir/$graph_name/final.pth > $model_dir/$graph_name/output.txt
	    done
	done
done



