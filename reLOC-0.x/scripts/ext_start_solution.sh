SIZE=$1
VARIANT=$2
ROBOTS=$3

./ext_grid_time.sh $SIZE $VARIANT $ROBOTS
./ext_grid_variables.sh $SIZE $VARIANT $ROBOTS
./ext_grid_sats.sh $SIZE $VARIANT $ROBOTS
#./ext_grid_CNF.sh $SIZE $VARIANT $ROBOTS