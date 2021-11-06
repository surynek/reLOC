SEED_LIST=`cat seeds`
SUBOPT=$1

for SEED in $SEED_LIST;
do
    ./exprSUB_mapsol.sh brc202d mdd++ robots_brc202d $SEED $SUBOPT &
    ./exprSUB_mapsol.sh den520d mdd++ robots_den520d $SEED $SUBOPT &
    ./exprSUB_mapsol.sh ost003d mdd++ robots_ost003d $SEED $SUBOPT &
done
