SIZE=$1
ALEN=$2
ROBOTS=$3

./netexp_netgen_bgu.sh $SIZE $ALEN $ROBOTS

#./netexp_netsol_make.sh $SIZE $ALEN Hmatching $ROBOTS &
#./netexp_netsol_make.sh $SIZE $ALEN Hsimplicial $ROBOTS &
#./netexp_netsol_make.sh $SIZE $ALEN idHmatching $ROBOTS &
#./netexp_netsol_make.sh $SIZE $ALEN idHsimplicial $ROBOTS &
#./netexp_netsol_make.sh $SIZE $ALEN mmdd $ROBOTS &
#./netexp_netsol_make.sh $SIZE $ALEN idmmdd $ROBOTS &

#./netexp_netsol_cost.sh $SIZE $ALEN idmdd $ROBOTS &
#./netexp_netsol_cost.sh $SIZE $ALEN mdd $ROBOTS &