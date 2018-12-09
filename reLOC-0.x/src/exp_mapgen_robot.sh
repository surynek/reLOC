seeds=`cat seeds`

MAP=$1
ROBOT_LIST=`cat $2`

echo 'map =' $MAP
echo 'robots =' $ROBOT_LIST

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./gridgen_reLOC --walk '--map-file=../maps/'$MAP '--N-robots='$ROBOTS '--obstacle-probability='$OBSTACLES '--seed='$seed '--multirobot-file=map_'$MAP'_r'$ROBOTS'_'$seed'.cpf' '--bgu-file=map_'$MAP'_r'$ROBOTS'_'$seed'.bgu'
  done
done
