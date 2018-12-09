seeds=`cat seeds`

SIZE=$1
VARIANT=$2

MAX_ROBOTS=$((SIZE * SIZE / 2))	

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS

ROBOT_LIST=`cat $3`

echo $ROBOT_LIST

for ROBOTS in $ROBOT_LIST;
do
  for seed in $seeds; do
    echo $ROBOTS,$seed
    grep "TIME" 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$VARIANT'.txt'
  done
done
