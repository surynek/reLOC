seeds=`cat seeds`

DIM=$1
ALEN=$2
ENCODING=$3

ROBOT_LIST=`cat $4`

echo $ROBOT_LIST

echo 'hyper-cube dimmension  =' $DIM
echo 'alen                   =' $ALEN
echo 'ecoding                =' $ENCODING

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  for seed in $seeds; do
    echo '  ' $seed
    ./solver_reLOC '--input-file=hyper_'$DIM'x'$ALEN'_r'$ROBOTS'_'$seed'.cpf' --makespan-limit=65536 --cost-limit=65536 --total-timeout=512 --minisat-timeout=512 '--encoding='$ENCODING '--output-file=hyper_'$DIM'x'$ALEN'_r'$ROBOTS'_'$seed'_'$ENCODING'.out' > 'hyper_'$DIM'x'$ALEN'_r'$ROBOTS'_'$seed'_'$ENCODING'.txt'
  done
done
