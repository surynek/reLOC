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
  for seed in $seeds; do
    echo $seed
    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --makespan-limit=16 --layer-limit=16 --total-timeout=256 --minisat-timeout=256 --encoding='$ENCODING' --completion=whca --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_whca.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_whca.txt'
    ./solver_reLOC '--input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf' --makespan-limit=65536 --layer-limit=65536 --total-timeout=300 --minisat-timeout=300 '--encoding='$ENCODING --completion=whca '--output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_whca.out' > 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_whca.txt'
  done
done
