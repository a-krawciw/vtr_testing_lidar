# Get arguments
MODULE=$1
RUN_ID=$2

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_${MODULE} \
  --ros-args -r __ns:=/vtr \
  --params-file ${VTRHROOT}/src/vtr_testing_honeycomb/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/main \
  -p run_id:=${RUN_ID}

exit $?
