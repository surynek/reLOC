ROBOT_LIST=`cat robots_mazes`
SEED_LIST=`cat seeds_10`
SIZE=16

for ROBOTS in $ROBOT_LIST;
do
  for SEED in $SEED_LIST;	
  do                
    echo 'Generating map maze128 with '$ROBOTS' agents ...'
   ../gridgen_reLOC --map-file=../../maps/maze-128-128-1.map '--N-robots='$ROBOTS '--multirobot-file=maze128_a'$ROBOTS'_'$SEED'.cpf'
  done
done
