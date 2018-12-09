seeds=`cat seeds`

SIZE=$1
ALEN=$2
ENCODING=$3

ROBOT_LIST=`cat $4`

echo $ROBOT_LIST

echo 'network size  =' $SIZE
echo 'alen          =' $ALEN
echo 'ecoding       =' $ENCODING

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./solver_reLOC '--input-file=network_'$SIZE'x'$SIZE'x'$ALEN'_r'$ROBOTS'_'$seed'.cpf' --makespan-limit=65536 --cost-limit=65536 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--output-file=network_'$SIZE'x'$SIZE'x'$ALEN'_r'$ROBOTS'_'$seed'_'$ENCODING'.out' > 'network_'$SIZE'x'$SIZE'x'$ALEN'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
  done
done
