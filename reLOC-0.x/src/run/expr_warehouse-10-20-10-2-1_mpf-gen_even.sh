PREFIX="warehouse-10-20-10-2-1"
SCENARIO_PREFIX=$PREFIX"-even"

ROBOTS_FILE='robots_'$PREFIX
SCENARIOS_FILE='scenarios_'$PREFIX

ROBOTS_LIST=`cat $ROBOTS_FILE`
SCENARIOS_LIST=`cat $SCENARIOS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    for SCENARIO in $SCENARIOS_LIST;
    do
	echo 'Generating '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' kruhobots ...'
	../moviscen_convert_boOX '--input-movi-map-file='$PREFIX'.map' '--input-movi-scen-file='$SCENARIO_PREFIX'-'$SCENARIO'.scen' '--output-mpf-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.mpf' '--output-cpf-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.cpf' '--N-agents='$ROBOTS
    done
done
