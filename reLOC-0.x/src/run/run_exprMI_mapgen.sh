SEED_LIST=`cat seeds`

for SEED in $SEED_LIST;
do
    ./exprMI_mapgen.sh brc202d robots_brc202d $SEED &
    ./exprMI_mapgen.sh den520d robots_den520d $SEED &
    ./exprMI_mapgen.sh ost003d robots_ost003d $SEED &
done
