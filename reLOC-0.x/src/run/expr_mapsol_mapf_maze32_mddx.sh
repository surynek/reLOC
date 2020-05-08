ROBOT_LIST=`cat robots_mazes`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving maze32 instance with '$ROBOTS' agents ...'
    ../insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=mddx '--input-file=maze32_a'$ROBOTS'_'$SEED'.cpf' '--output-file=mapf-mddx_maze32_a'$ROBOTS'_'$SEED'.out' > 'mapf-mddx_maze32_a'$ROBOTS'_'$SEED'.txt'
  done
done
