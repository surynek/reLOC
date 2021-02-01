MAP=$1
ENCODING=$2
ROBOT_LIST=`cat $3`
SEED=$4
TIMEOUT=`cat timeout`

echo $ROBOT_LIST
echo 'ecoding    =' $ENCODING

for ROBOTS in $ROBOT_LIST;
do
    echo $ROBOTS
    echo '  ' $SEED
    
    echo '../insolver_reLOC --input-file=map_'$MAP'_r'$ROBOTS'_'$SEED'.cpf --total-timeout='$TIMEOUT' --minisat-timeout='$TIMEOUT '--encoding='$ENCODING' --output-file=map_'$MAP'_r'$ROBOTS'_'$SEED'_'$ENCODING'.out > map_'$MAP'_r'$ROBOTS'_'$SEED'_'$ENCODING'.txt'
    ../insolver_reLOC '--input-file=map_'$MAP'_r'$ROBOTS'_'$SEED'.cpf' '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT '--encoding='$ENCODING '--output-file=MAP_'$MAP'_r'$ROBOTS'_'$SEED'_'$ENCODING'.out' > 'map_'$MAP'_r'$ROBOTS'_'$SEED'_'$ENCODING'.txt'
done
