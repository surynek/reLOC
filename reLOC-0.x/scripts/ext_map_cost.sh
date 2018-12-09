seeds=`cat seeds`

MAP=$2
ENCODING=$1
ROBOT_LIST=`cat $3`

echo 'map  =' $MAP
echo 'robots =' $ROBOT_LIST
echo 'ecoding =' $ENCODING


for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    grep "total cost" 'map_'$MAP'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
  done
done
