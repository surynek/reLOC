SIZE=$1
OBSTACLES=$2

./exp_gridgen.sh $SIZE $OBSTACLES

#./exp_gridsol.sh $SIZE decomposed
#./exp_gridsol.sh $SIZE independent
./exp_gridsol_whca.sh $SIZE plural