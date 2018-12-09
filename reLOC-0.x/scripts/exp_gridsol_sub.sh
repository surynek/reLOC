seeds=`cat seeds`

SIZE=$1
ENCODING=$2
RATIO=$3

ROBOT_LIST=`cat $4`

echo $ROBOT_LIST

echo 'grid size  =' $SIZE
echo 'ecoding    =' $ENCODING
echo 'ratio      =' $RATIO

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./solver_reLOC '--input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf' --cost-limit=16777216 --layer-limit=16777216 --makespan-limit=16777216 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--suboptimal-ratio='$RATIO '--output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out' > 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
  done
done
