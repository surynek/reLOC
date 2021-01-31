MAP=$1
ROBOT_LIST=`cat $2`
SEED=$3

for ROBOTS in $ROBOT_LIST;
do
  echo $ROBOTS
  echo '  ' $SEED
  echo './gridgen_reLOC --map-file='$MAP'.map' '--N-robots='$ROBOTS '--seed='$SEED '--multirobot-file=map_'$MAP'_r'$ROBOTS'_'$SEED'.cpf'  
  ../gridgen_reLOC --walk '--map-file='$MAP'.map' '--N-robots='$ROBOTS '--seed='$SEED '--multirobot-file=map_'$MAP'_r'$ROBOTS'_'$SEED'.cpf'
done
