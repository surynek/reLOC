seeds=`cat seeds`

SIZE=$1
ENCODING=$2

ROBOT_LIST=`cat $3`

echo $ROBOT_LIST

echo 'grid size =' $SIZE
echo 'ecoding   =' $ENCODING

for ROBOTS in $ROBOT_LIST;
do
  for seed in $seeds; do
    echo $seed
    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --makespan-limit=16 --layer-limit=16 --total-timeout=256 --minisat-timeout=256 --encoding='$ENCODING' --completion=unirobot --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_uni.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_uni.txt'
    ./solver_reLOC '--input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf' --makespan-limit=65536 --layer-limit=65536 --total-timeout=300 --minisat-timeout=300 '--encoding='$ENCODING --completion=unirobot '--output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_uni.out' > 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'_uni.txt'
  done
done
