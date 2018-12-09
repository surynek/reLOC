seeds=`cat seeds`

MAP=$2
ROBOTS=$3
ENCODING=$1

DISTANCE_LIST=`cat $4`

echo $DISTANCE_LIST

echo 'distances =' $DISTANCE_LIST
echo 'map  =' $MAP
echo 'robots =' $ROBOTS
echo 'ecoding =' $ENCODING

for DISTANCE in $DISTANCE_LIST;
do
  echo $DISTANCE
  for seed in $seeds; do
    echo '  ' $seed
#    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --cost-limit=65536 --total-timeout=300 --minisat-timeout=300 --encoding='$ENCODING' --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
    ./solver_reLOC '--input-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'.cpf' --cost-limit=65536 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--output-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_'$ENCODING'.out' > 'map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_'$ENCODING'.txt'
  done
done
