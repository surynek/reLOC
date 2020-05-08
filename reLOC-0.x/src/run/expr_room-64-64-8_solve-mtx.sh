SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="room-64-64-8"
SCENARIO_PREFIX=$PREFIX"-random"

ROBOTS_FILE='robots_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    ../insolver_reLOC '--total-timeout='$TIMEOUT  '--minisat-timeout'=$TIMEOUT '--input-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.cpf' '--encoding=mddx' '--output-file=solution-mtx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt' > 'out-mtx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'
done
