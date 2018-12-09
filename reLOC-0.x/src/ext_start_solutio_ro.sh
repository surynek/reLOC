SIZE=$1
VARIANT=$2
ROBOTS=$3
RATIOS=$4

./ext_grid_time_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
#./ext_grid_variables_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
#./ext_grid_sats_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
./ext_grid_ratio_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
./ext_grid_makespan_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
./ext_grid_cost_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS
#./ext_grid_CNF_ro.sh $SIZE $VARIANT $ROBOTS $RATIOS