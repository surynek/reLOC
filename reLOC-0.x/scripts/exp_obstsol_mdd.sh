seeds=`cat seeds`

SIZE=$2
ROBOTS=$3
ENCODING=$1

OBST_LIST=`cat $4`

echo $OBST_LIST

echo 'grid size  =' $SIZE
echo 'robots     =' $ROBOTS
echo 'obstacles  =' $OBST_LIST
echo 'ecoding    =' $ENCODING

for OBST in $OBST_LIST;
do
  echo $OBST
  for seed in $seeds; do
    echo '  ' $seed
#    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --makespan-limit=64 --layer-limit=64 --total-timeout=256 --minisat-timeout=256 --encoding='$ENCODING' --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
    ./solver_reLOC '--input-file=obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'.cpf' --makespan-limit=65536 --layer-limit=65536 --cost-limit=65536 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--output-file=obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'_'$ENCODING'.out' > 'obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'_'$ENCODING'.txt'
  done
done
