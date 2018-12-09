seeds=`cat puzzseeds`

SIZE=$1
ROBOTS=$((SIZE * SIZE - 1))

echo 'grid size =' $SIZE
echo 'robots    =' $ROBOTS

for seed in $seeds; do
    echo $seed
    echo './gridgen_reLOC --x-size='$SIZE' --y-size='$SIZE' --N-robots='$ROBOTS' --obstacle-probability=0.0 --seed='$seed' --multirobot-file=puzz_'$SIZE'x'$SIZE'_'$seed'.cpf'
    ./gridgen_reLOC '--x-size='$SIZE '--y-size='$SIZE '--N-robots='$ROBOTS '--obstacle-probability=0.0 --seed='$seed '--multirobot-file=puzz_'$SIZE'x'$SIZE'_'$seed'.cpf'
done
