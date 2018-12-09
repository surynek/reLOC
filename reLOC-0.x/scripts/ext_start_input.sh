SIZE=$1
OBSTACLES=$2
LEVEL=$3

./ext_gridgen.sh $SIZE $OBSTACLES $LEVEL
./ext_grid_literals.sh $SIZE
./ext_grid_variables.sh $SIZE
./ext_grid_clauses.sh $SIZE
