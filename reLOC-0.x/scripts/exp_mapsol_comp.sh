seeds=`cat seeds`

MAP=$1
ROBOTS=$2

DISTANCE_LIST=`cat $3`

echo $DISTANCE_LIST

echo 'distances =' $DISTANCE_LIST
echo 'map  =' $MAP
echo 'robots =' $ROBOTS

for DISTANCE in $DISTANCE_LIST;
do
  echo $DISTANCE
  for seed in $seeds; do
    echo '  ' $seed
#    echo './solver_reLOC --input-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cpf --cost-limit=65536 --total-timeout=300 --minisat-timeout=300 --completion=complete --output-file=grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_comp.out > grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'_comp.txt'
    ./solver_reLOC '--input-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'.cpf' --cost-limit=65536 --total-timeout=512 --minisat-timeout=512 --completion=complete '--output-file=map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_comp.out' > 'map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_comp.txt'
  done
done
