seeds=`cat seeds`

SIZE=$1
ROBOTS=$2
OBST_LIST=`cat $3`

echo 'grid size  =' $SIZE
echo 'robots     =' $ROBOTS
echo 'obstacles  =' $OBST_LIST

for OBST in $OBST_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
#    echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability='$OBSTACLES' --seed='$seed' --multirobot-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf'
    ./gridgen_reLOC --walk '--x-size='$SIZE '--y-size='$SIZE '--N-robots='$ROBOTS '--N-obstacles='$OBST '--seed='$seed '--multirobot-file=obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'.cpf' '--bgu-file=obst_'$SIZE'x'$SIZE'_r'$ROBOTS'_o'$OBST'_'$seed'.bgu'
  done
done
