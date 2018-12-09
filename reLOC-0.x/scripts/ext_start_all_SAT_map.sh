ENCODING=mdd

./ext_start_map.sh $ENCODING den520d.map 16 distances16 > 'solution_den520_r16_'$ENCODING'.txt'
#./ext_start_map.sh $ENCODING den520d.map 24 distances24 > 'solution_den520_r24_'$ENCODING'.txt'
./ext_start_map.sh $ENCODING den520d.map 32 distances32 > 'solution_den520_r32_'$ENCODING'.txt'

./ext_start_map.sh $ENCODING ost003d.map 16 distances16 > 'solution_ost003_r16_'$ENCODING'.txt'
#./ext_start_map.sh $ENCODING ost003d.map 24 distances24 > 'solution_ost003_r24_'$ENCODING'.txt'
./ext_start_map.sh $ENCODING ost003d.map 32 distances32 > 'solution_ost003_r32_'$ENCODING'.txt'

./ext_start_map.sh $ENCODING brc202d.map 16 distances16 > 'solution_brc202d_r16_'$ENCODING'.txt'
#./ext_start_map.sh $ENCODING brc202d.map 24 distances24 > 'solution_brc202d_r24_'$ENCODING'.txt'
./ext_start_map.sh $ENCODING brc202d.map 32 distances32 > 'solution_brc202d_r32_'$ENCODING'.txt'

