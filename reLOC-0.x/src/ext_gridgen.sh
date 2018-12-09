seeds=`cat seeds`

SIZE=$1
MAX_ROBOTS=$((SIZE * SIZE / 2))	
OBSTACLES=$2
LEVEL=$3

ROBOT_LIST=`cat $4`

echo $ROBOT_LIST

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS
echo 'obstacles  =' $OBSTACLES

for ROBOTS in $ROBOT_LIST;
do
  for seed in $seeds; do
    echo $seed
    echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability='$OBSTACLES' --seed='$seed' --cnf-level='$LEVEL' --cnf-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cnf'
    ./gridgen_reLOC '--x-size='$SIZE '--y-size='$SIZE '--N-robots='$ROBOTS '--obstacle-probability='$OBSTACLES '--seed='$seed '--cnf-level='$LEVEL '--cnf-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cnf'
  done
done
