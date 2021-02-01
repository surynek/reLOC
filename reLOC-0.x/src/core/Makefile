all: debug

debug:	libCore_reLOC_dbg.a 

optimized:	libCore_reLOC_opt.a 

libCore_reLOC_dbg.a:	types.o_dbg reloc.o_dbg hierarch.o_dbg encodings_direct.o_dbg encodings_binary.o_dbg encodings_mdd.o_dbg encodings_derived.o_dbg encodings_extract.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg 
	ar r libCore_reLOC_dbg.a types.o_dbg reloc.o_dbg hierarch.o_dbg encodings_direct.o_dbg encodings_binary.o_dbg encodings_mdd.o_dbg encodings_derived.o_dbg encodings_extract.o_dbg multirobot.o_dbg search.o_dbg cnf.o_dbg compress.o_dbg statistics.o_dbg graph.o_dbg complete.o_dbg

types.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h types.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -otypes.o_dbg types.cpp

reloc.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oreloc.o_dbg reloc.cpp

hierarch.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hierarch.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ohierarch.o_dbg hierarch.cpp

encodings_direct.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_direct.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oencodings_direct.o_dbg encodings_direct.cpp

encodings_binary.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_binary.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oencodings_binary.o_dbg encodings_binary.cpp

encodings_mdd.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_mdd.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oencodings_mdd.o_dbg encodings_mdd.cpp

encodings_derived.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_derived.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oencodings_derived.o_dbg encodings_derived.cpp

encodings_extract.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_extract.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -oencodings_extract.o_dbg encodings_extract.cpp

multirobot.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h multirobot.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -omultirobot.o_dbg multirobot.cpp

search.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h search.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -osearch.o_dbg search.cpp

cnf.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h cnf.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ocnf.o_dbg cnf.cpp

compress.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h compress.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ocompress.o_dbg compress.cpp

statistics.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h statistics.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ostatistics.o_dbg statistics.cpp

graph.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h graph.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ograph.o_dbg graph.cpp

complete.o_dbg:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h complete.cpp
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -ocomplete.o_dbg complete.cpp


libCore_reLOC_opt.a:	types.o_opt reloc.o_opt hierarch.o_opt encodings_direct.o_opt encodings_binary.o_opt encodings_mdd.o_opt encodings_derived.o_opt encodings_extract.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt 
	ar r libCore_reLOC_opt.a types.o_opt reloc.o_opt hierarch.o_opt encodings_direct.o_opt encodings_binary.o_opt encodings_mdd.o_opt encodings_derived.o_opt encodings_extract.o_opt multirobot.o_opt search.o_opt cnf.o_opt compress.o_opt statistics.o_opt graph.o_opt complete.o_opt

types.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h types.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -otypes.o_opt types.cpp

reloc.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h reloc.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oreloc.o_opt reloc.cpp

hierarch.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h hierarch.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ohierarch.o_opt hierarch.cpp

encodings_direct.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_direct.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oencodings_direct.o_opt encodings_direct.cpp

encodings_binary.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_binary.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oencodings_binary.o_opt encodings_binary.cpp

encodings_mdd.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_mdd.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oencodings_mdd.o_opt encodings_mdd.cpp

encodings_derived.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_derived.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oencodings_derived.o_opt encodings_derived.cpp

encodings_extract.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h encodings_extract.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -oencodings_extract.o_opt encodings_extract.cpp

multirobot.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h multirobot.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -omultirobot.o_opt multirobot.cpp

search.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h search.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -osearch.o_opt search.cpp

cnf.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h cnf.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocnf.o_opt cnf.cpp

compress.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h compress.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocompress.o_opt compress.cpp

statistics.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h statistics.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ostatistics.o_opt statistics.cpp

graph.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h graph.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ograph.o_opt graph.cpp

complete.o_opt:	../../src/version.h ../../src/defs.h ../../src/config.h ../../src/compile.h ../../src/result.h types.h reloc.h hierarch.h multirobot.h search.h cnf.h compress.h statistics.h graph.h complete.h complete.cpp
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -ocomplete.o_opt complete.cpp


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output.txt *.cnf *.pddl _compress/* *.out *.rbt _simplify/*
