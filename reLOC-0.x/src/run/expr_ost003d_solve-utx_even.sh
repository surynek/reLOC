SCENARIO=$1

TIMEOUT=`cat timeout`
PREFIX="ost003d"
SCENARIO_PREFIX=$PREFIX"-even"

ROBOTS_FILE='robots_'$PREFIX
ROBOTS_LIST=`cat $ROBOTS_FILE`

for ROBOTS in $ROBOTS_LIST;
do
    echo 'Solving '$PREFIX' scenario '$SCENARIO' MAPF instance with '$ROBOTS' agents ...'
    ../insolver_reLOC '--total-timeout='$TIMEOUT '--minisat-timeout='$TIMEOUT  '--input-file='$PREFIX'-'$SCENARIO'_a'$ROBOTS'.cpf' '--encoding=mdd' '--output-file=solution-utx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt' > 'out-utx_'$PREFIX'-'$SCENARIO'_a'$ROBOTS'.txt'
done
