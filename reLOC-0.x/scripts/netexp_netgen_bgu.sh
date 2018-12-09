seeds=`cat seeds`

SIZE=$1
ROBOT_LIST=`cat $3`
ALEN=$2

echo 'network size  =' $SIZE
echo 'alen          =' $ALEN

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./netgen_reLOC --walk '--x-size='$SIZE '--y-size='$SIZE '--alen='$ALEN '--N-robots='$ROBOTS '--seed='$seed '--bgu-file=network_'$SIZE'x'$SIZE'x'$ALEN'_r'$ROBOTS'_'$seed'.bgu'
  done
done
