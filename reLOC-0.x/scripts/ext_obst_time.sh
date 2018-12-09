seeds=`cat seeds`

SIZE=$2
ROBOTS=$3
ENCODING=$1

echo 'grid size  =' $SIZE
echo 'robots =' $ROBOTS

OBST_LIST=`cat $4`

echo 'obstacles = ' $OBST_LIST

for OBST in $OBST_LIST;
do
  for seed in $seeds; do
    echo $OBST,$seed
    grep "TIME" 'obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'_'$ENCODING'.txt'
  done
done
