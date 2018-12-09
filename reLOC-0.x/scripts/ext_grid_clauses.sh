seeds=`cat seeds`

SIZE=$1
MAX_ROBOTS=$((SIZE * SIZE / 2))	

echo 'grid size  =' $SIZE
echo 'max robots =' $MAX_ROBOTS

for ((ROBOTS=1; ROBOTS <= MAX_ROBOTS; ROBOTS++))
do
  for seed in $seeds; do
    echo $ROBOTS,$seed
    grep "clauses" 'grid_'$SIZE'x'$SIZE'_r'$ROBOTS'_'$seed'.cnf'
  done
done
