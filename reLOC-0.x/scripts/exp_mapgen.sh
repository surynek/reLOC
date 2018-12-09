seeds=`cat seeds`

MAP=$1
ROBOTS=$2
DISTANCE_LIST=`cat $3`

echo 'map =' $MAP
echo 'robots =' $ROBOTS
echo 'distances  =' $DISTANCE_LIST

for DISTANCE in $DISTANCE_LIST;
do
  echo $DISTANCE
  for seed in $seeds; do
    echo '  ' $seed
#    echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability='$OBSTACLES' --seed='$seed' --multirobot-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf'
    ./gridgen_reLOC '--distance='$DISTANCE '--N-robots='$ROBOTS '--seed='$seed '--multirobot-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'.cpf' '--map-file=../maps/'$MAP '--bgu-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'.bgu'
  done
done
