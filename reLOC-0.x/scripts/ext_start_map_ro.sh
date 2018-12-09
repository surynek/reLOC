ENCODING=$1
MAP=$2
ROBOTS=$3
RATIOS=$4

./ext_map_time_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
#./ext_map_variables_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
#./ext_map_sats_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
./ext_map_cost_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
./ext_map_makespan_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
./ext_map_ratio_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
#./ext_map_CNF_ro.sh $ENCODING $MAP $ROBOTS $RATIOS
