SEED_LIST=`cat seeds`

for SEED in $SEED_LIST;
do
    ./exprMI_mapsol.sh brc202d mdd++ robots_brc202d $SEED &
    ./exprMI_mapsol.sh den520d mdd++ robots_den520d $SEED &
    ./exprMI_mapsol.sh ost003d mdd++ robots_ost003d $SEED &
done
