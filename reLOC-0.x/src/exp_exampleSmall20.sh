FILENAME_LIST=`cat exampleSmall20_names.txt`

for FILENAME in $FILENAME_LIST;
do
    echo $FILENAME
    ./insolver_reLOC '--dibox-input='$FILENAME --encoding=mdd++ --directed --minisat-timeout=300 --total-timeout=300 --output-file=solution.txt > $FILENAME'.out'
done
