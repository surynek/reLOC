seeds=`cat seeds`

MAP=$3
ROBOTS=$4
ENCODING=$1
RATIO=$2

DISTANCE_LIST=`cat $5`

echo $DISTANCE_LIST

echo 'distances =' $DISTANCE_LIST
echo 'map       =' $MAP
echo 'robots    =' $ROBOTS
echo 'ecoding   =' $ENCODING
echo 'ratio     =' $RATIO

for DISTANCE in $DISTANCE_LIST;
do
  echo $DISTANCE
  for seed in $seeds; do
    echo '  ' $seed
#    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --cost-limit=65536 --total-timeout=300 --minisat-timeout=300 --encoding='$ENCODING' --suboptimal-ratio='$RATIO' --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
    ./solver_reLOC '--input-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'.cpf' --cost-limit=16777216 --layer-limit=16777216 --makespan-limit=16777216 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--suboptimal-ratio='$RATIO '--output-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_'$ENCODING'.out' > 'map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_'$ENCODING'.txt'
  done
done
