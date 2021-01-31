SEED_LIST=`cat seeds`

for SEED in $SEED_LIST;
do
    ./exprMI_gridsol.sh 8 mdd++ robots_08x08 $SEED &
    ./exprMI_gridsol.sh 16 mdd++ robots_16x16 $SEED &
    ./exprMI_gridsol.sh 32 mdd++ robots_32x32 $SEED &
done
