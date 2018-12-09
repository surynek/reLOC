seeds=`cat seeds`

SIZE=$1
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
    ./solver_reLOC '--input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf' --completion=complete --cost-limit=16777216 --layer-limit=16777216 --makespan-limit=16777216 --total-timeout=512 --minisat-timeout=512 '--output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_comp.out' > 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_comp.txt'
  done
done
