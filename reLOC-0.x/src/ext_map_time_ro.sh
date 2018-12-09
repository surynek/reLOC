seeds=`cat seeds`

MAP=$2
ENCODING=$1
ROBOT_LIST=`cat $3`
RATIOS_LIST=`cat $4`

echo 'map  =' $MAP
echo 'robots =' $ROBOT_LIST
echo 'ecoding =' $ENCODING


for ROBOTS in $ROBOT_LIST;
do
  for RATIO in $RATIOS_LIST;
  do
    echo $ROBOTS,$RATIO
    for seed in $seeds; do
      echo '  ' $seed
      grep "ratio" 'map_'$MAP'_r'$ROBOTS'_s'$RATIO'_'$seed'_'$ENCODING'.txt'
    done
  done
done
