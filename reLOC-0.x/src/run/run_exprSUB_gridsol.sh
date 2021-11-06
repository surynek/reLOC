SEED_LIST=`cat seeds`
SUBOPT=$1

for SEED in $SEED_LIST;
do
    ./exprSUB_gridsol.sh 8 mdd++ robots_08x08 $SEED $SUBOPT &
    ./exprSUB_gridsol.sh 16 mdd++ robots_16x16 $SEED $SUBOPT &
    ./exprSUB_gridsol.sh 32 mdd++ robots_32x32 $SEED $SUBOPT &
done
