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
    grep "TIME" 'map_'$MAP'_r'$ROBOTS'_d'$DISTANCE'_'$seed'_'$ENCODING'.txt'
  done
done
