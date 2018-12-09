SIZE=$1
OBSTACLES=$2

./exp_gridgen.sh $SIZE $OBSTACLES

./exp_gridsol.sh $SIZE Hmatching &
./exp_gridsol.sh $SIZE Hdifferential &
./exp_gridsol.sh $SIZE Hadvanced &
./exp_gridsol.sh $SIZE decomposed &
./exp_gridsol.sh $SIZE independent &