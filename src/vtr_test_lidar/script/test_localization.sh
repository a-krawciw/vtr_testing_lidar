## this script assumes the following environment variables are set:
##   VTRHROOT VTRHDATA VTRHRESULT
## example usage: test_localization.sh boreas-2021-09-02-11-42 boreas-2021-09-07-09-35

# Get arguments
ODO_INPUT=$1
LOC_INPUT=$2

# Log
echo "Running localization on sequence ${LOC_INPUT} to reference sequence ${ODO_INPUT}, storing result to ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}"

# Source the VTR environment with the testing package
source ${VTRHROOT}/install/setup.bash

rm -r ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTRHRESULT}/${ODO_INPUT}/${ODO_INPUT}/* ${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_honeycomb vtr_testing_honeycomb_localization \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${VTRHROOT}/src/vtr_testing_honeycomb/config/honeycomb.yaml \
  -p data_dir:=${VTRHRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRHDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRHDATA}/${LOC_INPUT}
