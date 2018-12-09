seeds=`cat seeds`

DIMMENSION=$1
ROBOT_LIST=`cat $3`
ALEN=$2

echo 'hyper-cube dimmension  =' $DIMMENSION
echo 'alen                   =' $ALEN

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./hypergen_reLOC --walk '--dimmension='$DIMMENSION '--alen='$ALEN '--N-robots='$ROBOTS '--seed='$seed '--bgu-file=hyper_'$DIMMENSION'x'$ALEN'_r'$ROBOTS'_'$seed'.bgu'
  done
done
