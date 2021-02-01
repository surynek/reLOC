SEED_LIST=`cat seeds`

for SEED in $SEED_LIST;
do
    ./exprMI_gridgen.sh 8 0.1 robots_08x08 $SEED &
    ./exprMI_gridgen.sh 16 0.1 robots_16x16 $SEED &
    ./exprMI_gridgen.sh 32 0.1 robots_32x32 $SEED &
done
