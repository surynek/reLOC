seeds=`cat puzzseeds`

SIZE=$1
ROBOTS=$((SIZE * SIZE - 1))	
ENCODING=$2

echo 'grid size =' $SIZE
echo 'robots    =' $ROBOTS
echo 'ecoding   =' $ENCODING

for seed in $seeds; do
  echo $seed
  echo './solver_reLOC --input-file=puzz_'$SIZE'x'$SIZE'_'$seed'.cpf --makespan-limit=1280 --layer-limit=1280 --total-timeout=25600 --minisat-timeout=25600 --encoding='$ENCODING' --output-file=puzz_'$SIZE'x'$SIZE'_'$seed'_'$ENCODING'.out > puzz_'$SIZE'x'$SIZE'_'$seed'_'$ENCODING'.txt'
  ./solver_reLOC '--input-file=puzz_'$SIZE'x'$SIZE'_'$seed'.cpf' --makespan-limit=1280 --layer-limit=1280 --total-timeout=25600 --minisat-timeout=25600 '--encoding='$ENCODING '--output-file=puzz_'$SIZE'x'$SIZE'_'$seed'_'$ENCODING'.out' > 'puzz_'$SIZE'x'$SIZE'_'$seed'_'$ENCODING'.txt'
done
