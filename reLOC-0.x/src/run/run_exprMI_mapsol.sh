SEED_LIST=`cat seeds`

for SEED in $SEED_LIST;
do
    ./exprMI_mapsol.sh brc202d robots_brc202d $SEED &
    ./exprMI_mapsol.sh den520d robots_den520d $SEED &
    ./exprMI_mapsol.sh ost003d robots_ost003d $SEED &
done
