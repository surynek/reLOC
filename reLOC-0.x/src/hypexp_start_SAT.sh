DIM=$1
ALEN=$2
ROBOTS=$3

./hypexp_hypgen.sh $DIM $ALEN $ROBOTS

#./hypexp_hypsol_make.sh $DIM $ALEN Hmatching $ROBOTS &
#./hypexp_hypsol_make.sh $DIM $ALEN Hsimplicial $ROBOTS &
#./hypexp_hypsol_make.sh $DIM $ALEN idHmatching $ROBOTS &
#./hypexp_hypsol_make.sh $DIM $ALEN idHsimplicial $ROBOTS &
#./hypexp_hypsol_make.sh $DIM $ALEN mmdd $ROBOTS &
#./hypexp_hypsol_make.sh $DIM $ALEN idmmdd $ROBOTS &

#./hypexp_hypsol_cost.sh $DIM $ALEN idmdd $ROBOTS &
#./hypexp_hypsol_cost.sh $DIM $ALEN mdd $ROBOTS &