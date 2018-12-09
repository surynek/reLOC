FILENAME_LIST=`cat exampleSmall20_names.txt`

for FILENAME in $FILENAME_LIST;
do
    echo $FILENAME
done

for FILENAME in $FILENAME_LIST;
do
    grep "total trajectory" $FILENAME'.out'
done

for FILENAME in $FILENAME_LIST;
do
    grep "machine TIME" $FILENAME'.out'
done
