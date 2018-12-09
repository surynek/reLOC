MAP=$1
ROBOTS=$2

./exp_mapgen_robot.sh $MAP $ROBOTS

#./exp_mapsol_sub_robot.sh mdd* 1.5 $MAP $ROBOTS &
#./exp_mapsol_sub_robot.sh mdd* 1.1 $MAP $ROBOTS &
#./exp_mapsol_sub_robot.sh mdd* 1.05 $MAP $ROBOTS &
#./exp_mapsol_sub_robot.sh mdd* 1.01 $MAP $ROBOTS &
#./exp_mapsol_comp_robot.sh comp 100 $MAP $ROBOTS &
./exp_mapsol_sub_robot_ro.sh mdd* ratios $MAP $ROBOTS &