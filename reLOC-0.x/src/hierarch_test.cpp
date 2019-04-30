/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.20-kruh                               */
/*                                                                            */
/*                      (C) Copyright 2019 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* hierarch_test.cpp / 0.20-kruh_059                                          */
/*----------------------------------------------------------------------------*/
//
// Hierarchical version of relocation problem - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "defs.h"
#include "hierarch.h"
#include "compile.h"

#include "hierarch_test.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_hierarchical_instance_1(void)
    {
	printf("Hierarchical relocation instance test 1 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(4);
	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 0);

	sHierarchicalArrangement initial(4, 2, 2);
	initial.place_Object(1, 3);
	initial.place_Object(2, 1);

	initial.place_Agent(1, 0);
	initial.place_Agent(2, 1);

	sHierarchicalArrangement goal(4, 2, 2);
	goal.place_Object(1, 1);
	goal.place_Object(2, 3);

	goal.place_Agent(1, 1);
	goal.place_Agent(2, 0);

	sHierarchicalRelocationInstance hierarchical_1(environment, initial, goal);

	hierarchical_1.to_Screen();
	hierarchical_1.to_Screen_domainPDDL();
	hierarchical_1.to_Screen_problemPDDL();

	hierarchical_1.to_File_domainPDDL("hierarchical.pddl");
	hierarchical_1.to_File_problemPDDL("hierarchical_01.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Hierarchical relocation instance test 1 ... finished\n");
    }


    void test_hierarchical_instance_2(void)
    {
	printf("Hierarchical relocation instance test 2 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(4);
	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 0);

	sHierarchicalArrangement initial(4, 3, 3);
	initial.place_Object(1, 3);
	initial.place_Object(2, 1);
	initial.place_Object(3, 0);

	initial.place_Agent(1, 0);
	initial.place_Agent(2, 1);
	initial.place_Agent(3, 2);

	sHierarchicalArrangement goal(4, 3, 3);
	goal.place_Object(1, 1);
	goal.place_Object(2, 3);
	goal.place_Object(3, 2);

	goal.place_Agent(1, 1);
	goal.place_Agent(2, 0);
	goal.place_Agent(3, 3);

	sHierarchicalRelocationInstance hierarchical_2(environment, initial, goal);

	hierarchical_2.to_Screen();
	hierarchical_2.to_Screen_domainPDDL();
	hierarchical_2.to_Screen_problemPDDL();

	hierarchical_2.to_File_domainPDDL("hierarchical.pddl");
	hierarchical_2.to_File_problemPDDL("hierarchical_02.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Hierarchical relocation instance test 2 ... finished\n");
    }


    void test_hierarchical_instance_3(void)
    {
	printf("Hierarchical relocation instance test 3 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(6);
	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 0);
	environment.add_Edge(1, 4);
	environment.add_Edge(4, 5);
	environment.add_Edge(5, 2);

	sHierarchicalArrangement initial(6, 4, 2);
	initial.place_Object(1, 3);
	initial.place_Object(2, 1);
	initial.place_Object(3, 0);
	initial.place_Object(4, 5);

	initial.place_Agent(1, 0);
	initial.place_Agent(2, 1);

	sHierarchicalArrangement goal(6, 4, 2);
	goal.place_Object(1, 1);
	goal.place_Object(2, 3);
	goal.place_Object(3, 5);
	goal.place_Object(4, 2);

	goal.place_Agent(1, 1);
	goal.place_Agent(2, 0);

	sHierarchicalRelocationInstance hierarchical_3(environment, initial, goal);

	hierarchical_3.to_Screen();
	hierarchical_3.to_Screen_domainPDDL();
	hierarchical_3.to_Screen_problemPDDL();

	hierarchical_3.to_File_domainPDDL("hierarchical.pddl");
	hierarchical_3.to_File_problemPDDL("hierarchical_03.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Hierarchical relocation instance test 3 ... finished\n");
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    test_hierarchical_instance_1();
    test_hierarchical_instance_2();
    test_hierarchical_instance_3();

    return 0;
}
