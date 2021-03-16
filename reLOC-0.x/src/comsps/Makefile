all: debug

debug:	libComsps_reLOC_dbg.a 

optimized:	libComsps_reLOC_opt.a 

libComsps_reLOC_dbg.a:	Options.o_dbg Solver.o_dbg System.o_dbg 
	ar r libComsps_reLOC_dbg.a Options.o_dbg Solver.o_dbg System.o_dbg

Options.o_dbg:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Options.cc
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -I../../src/comsps -oOptions.o_dbg Options.cc

Solver.o_dbg:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Solver.cc
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -I../../src/comsps -oSolver.o_dbg Solver.cc

System.o_dbg:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h System.cc
	g++ -Wall -Wextra -pedantic -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -Wno-class-memaccess -g -std=c++0x -c -I../../src -I../../src/core -I../../src/comsps -oSystem.o_dbg System.cc


libComsps_reLOC_opt.a:	Options.o_opt Solver.o_opt System.o_opt 
	ar r libComsps_reLOC_opt.a Options.o_opt Solver.o_opt System.o_opt

Options.o_opt:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Options.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -I../../src/comsps -oOptions.o_opt Options.cc

Solver.o_opt:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h Solver.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -I../../src/comsps -oSolver.o_opt Solver.cc

System.o_opt:	Alg.h Alloc.h Dimacs.h Heap.h IntTypes.h Map.h Options.h ParseUtils.h Queue.h Solver.h SolverTypes.h Sort.h System.h Vec.h XAlloc.h System.cc
	g++ -std=c++0x -c -w -O9 -mtune=native -I../../src -I../../src/core -I../../src/comsps -oSystem.o_opt System.cc


clean:
	rm -f -r *~ *.o_dbg *.o_opt *.a *.bak output.txt *.cnf *.pddl _compress/* *.out *.rbt _simplify/*
