ROBOT_LIST=`cat robots_mazes`
SEED_LIST=`cat seeds_10`
SIZE=16
TIMEOUT=`cat timeout`

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do            
    echo 'Solving maze32 instance with '$ROBOTS' agents ...'
    ../insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT --encoding=mddu '--input-file=maze128_a'$ROBOTS'_'$SEED'.cpf' '--output-file=mapf-mddu_maze128_a'$ROBOTS'_'$SEED'.out' > 'mapf-mddu_maze128_a'$ROBOTS'_'$SEED'.txt'
  done
done
