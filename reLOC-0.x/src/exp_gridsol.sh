seeds=`cat seeds`

SIZE=$1
MAX_ROBOTS=$((SIZE * SIZE / 2))	
ENCODING=$2

ROBOT_LIST=`cat $3`

echo $ROBOT_LIST

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS
echo 'ecoding    =' $ENCODING

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
#    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --makespan-limit=64 --layer-limit=64 --total-timeout=256 --minisat-timeout=256 --encoding='$ENCODING' --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
    ./solver_reLOC '--input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf' --makespan-limit=65536 --layer-limit=65536 --total-timeout=300 --minisat-timeout=300 '--encoding='$ENCODING '--output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out' > 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
  done
done
