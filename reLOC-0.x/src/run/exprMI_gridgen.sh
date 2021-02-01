SIZE=$1
ROBOT_LIST=`cat $3`
OBSTACLES=$2
SEED=$4

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS
echo 'obstacles  =' $OBSTACLES

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  echo '  ' $SEED
  echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability='$OBSTACLES' --seed='$SEED' --multirobot-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$SEED'.cpf'  
  ../gridgen_reLOC --walk '--x-size='$SIZE '--y-size='$SIZE '--N-robots='$ROBOTS '--obstacle-probability='$OBSTACLES '--seed='$SEED '--multirobot-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$SEED'.cpf'
done
