PREFIX="maze-128-128-10"

SCENARIOS_FILE="scenarios_"$PREFIX
ROBOTS_FILE="robots_"$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Extracting '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
        LINE_OUTPUT=`grep -s "machine TIME " 'out-mtx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'`
	if [ -z "$LINE_OUTPUT" ]
          then echo "CPU/machine TIME (seconds) = 1024.0"
          else echo $LINE_OUTPUT
        fi
    done
done
