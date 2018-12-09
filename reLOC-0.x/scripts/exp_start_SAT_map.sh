MAP=$1
ROBOTS=$2
DISTANCES=$3

./exp_mapgen.sh $MAP $ROBOTS $DISTANCES

#./exp_mapsol_mdd.sh mdd $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh admdd++ $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh bmdd $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh bcmdd $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_comp.sh $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_uni.sh plural $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_mdd.sh mdd* $MAP $ROBOTS $DISTANCES &
#./exp_mapsol_sub.sh mdd* 1.05 $MAP $ROBOTS $DISTANCES $
#./exp_mapsol_sub.sh mdd* 1.10 $MAP $ROBOTS $DISTANCES $