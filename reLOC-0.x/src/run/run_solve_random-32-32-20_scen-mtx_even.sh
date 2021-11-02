PREFIX='random-32-32-20'

SCENARIOS_FILE="scenarios_"$PREFIX
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for SCENARIO in $SCENARIOS_LIST;
do
    './expr_'$PREFIX'_solve-mtx_even.sh' $SCENARIO &
done
