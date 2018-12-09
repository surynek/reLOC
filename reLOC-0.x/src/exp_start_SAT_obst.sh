SIZE=$1
ROBOTS=$2
OBSTACLES=$3

./exp_obstgen.sh $SIZE $ROBOTS $OBSTACLES

./exp_obstsol_mdd.sh mdd $SIZE $ROBOTS $OBSTACLES &
#./exp_obstsol_mdd.sh bnomdd $SIZE $ROBOTS $OBSTACLES &
#./exp_obstsol_mdd.sh bmdd $SIZE $ROBOTS $OBSTACLES &
#./exp_obstsol_mdd.sh bnomdd $SIZE $ROBOTS $OBSTACLES &
