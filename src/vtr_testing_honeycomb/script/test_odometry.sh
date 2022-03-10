## this script assumes the following environment variables are set:
##   VTRHROOT VTRHDATA VTRHRESULT
## example usage: test_odometry.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1

# Log
echo "Running odometry on sequence ${ODO_INPUT}, storing result to ${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_odometry \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRHROOT}/src/vtr_testing_honeycomb/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT}
