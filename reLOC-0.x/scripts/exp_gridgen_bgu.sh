seeds=`cat seeds`

SIZE=$1
MAX_ROBOTS=$((SIZE * SIZE / 2))	
OBSTACLES=$2
ROBOT_LIST=`cat $3`

echo $ROBOT_LIST

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS
echo 'obstacles  =' $OBSTACLES

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
#    echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability='$OBSTACLES' --seed='$seed' --bgu-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.bgu'
    ./gridgen_reLOC --walk '--x-size='$SIZE '--y-size='$SIZE '--N-robots='$ROBOTS '--obstacle-probability='$OBSTACLES '--seed='$seed '--bgu-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.bgu'
  done
done
