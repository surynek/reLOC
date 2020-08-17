/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                      (C) Copyright 2019 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* multirobot_test.cpp / 0.21-robik_041                                       */
/*----------------------------------------------------------------------------*/
//
// Multirobot coordinated path-finding - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "multirobot.h"
#include "statistics.h"
#include "graph.h"
#include "search.h"

#include "multirobot_test.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_robot_arrangement_1(void)
    {
	printf("Robot arrangement test 1 ...\n");

	sRobotArrangement arrangement_1(16, 10);

	arrangement_1.to_Screen();

	arrangement_1.place_Robot(1, 3);
	arrangement_1.place_Robot(2, 4);
	arrangement_1.place_Robot(3, 10);
	arrangement_1.to_Screen();

	printf("  cleaning vertex 4 ...\n");
	arrangement_1.clean_Vertex(4);
	arrangement_1.to_Screen();

	printf("  removing robot 3 ...\n");
	arrangement_1.remove_Robot(3);
	arrangement_1.to_Screen();

	printf("  moving robot 1 to vertex 6 ...\n");
	arrangement_1.move_Robot(1, 6);
	arrangement_1.to_Screen();

	printf("Robot arrangement test 1 ... finished\n");
    }

    void test_robot_goal_1(void)
    {
	printf("Robot goal test 1 ...\n");

	sRobotGoal robot_goal_1(10, 8);
	robot_goal_1.to_Screen();

	printf("  charging goals 1 ...\n");
	robot_goal_1.charge_Robot(2, 2);
	robot_goal_1.charge_Robot(2, 4);
	robot_goal_1.charge_Robot(2, 5);
	robot_goal_1.to_Screen();

	printf("  charging goals 2 ...\n");
	sRobotGoal::Vertices_set goal_IDs_1;
	goal_IDs_1.insert(6);
	goal_IDs_1.insert(7);
	robot_goal_1.charge_Robot(3, goal_IDs_1);
	robot_goal_1.to_Screen();

	printf("  charging goals 3 ...\n");
	sRobotGoal::Vertices_set goal_IDs_2;
	goal_IDs_2.insert(1);
	goal_IDs_2.insert(7);
	goal_IDs_2.insert(8);
	robot_goal_1.charge_Robot(5, goal_IDs_2);
	robot_goal_1.to_Screen();

	sRobotGoal robot_goal_2(10, 8, 2);
	robot_goal_2.to_Screen();

	sRobotGoal robot_goal_3(10, 8, 3);
	robot_goal_3.to_Screen();

	printf("Robot goal test 1 ... finished\n");
    }

    void test_multirobot_instance_1(void)
    {
	printf("Multirobot instance test 1 ...\n");

	sUndirectedGraph graph_1;
	graph_1.add_Vertices(16);

	for (int i = 0; i < 15; ++i)
	{
	    for (int j = i + 1; j < 16; ++j)
	    {
		double rnd = rand() / (double)RAND_MAX;
		if (rnd < 0.1)
		{
		    graph_1.add_Edge(i, j);
		}
	    }
	}

	sRobotArrangement arrangement_1(16, 4);
	arrangement_1.place_Robot(1, 3);
	arrangement_1.place_Robot(2, 5);
	arrangement_1.place_Robot(3, 7);
	arrangement_1.place_Robot(4, 13);

	sRobotArrangement arrangement_2(arrangement_1);
	arrangement_2.move_Robot(3, 8);
	arrangement_2.move_Robot(4, 2);

	sMultirobotInstance multirobot_instance_1(graph_1, arrangement_1, arrangement_2);
	multirobot_instance_1.to_Screen();

	printf("Multirobot instance test 1 ... finished\n");
    }


    void test_multirobot_solution_1(void)
    {
	printf("Multirobot solution test 1 ...\n");

	sMultirobotSolution solution_1;

	solution_1.to_Screen();

	solution_1.add_Move(0, sMultirobotSolution::Move(2, 1, 2));
	solution_1.add_Move(3, sMultirobotSolution::Move(3, 7, 1));
	solution_1.to_Screen();

	solution_1.add_Move(1, sMultirobotSolution::Move(5, 3, 6));
	solution_1.add_Move(0, sMultirobotSolution::Move(7, 8, 1));
	solution_1.add_Move(1, sMultirobotSolution::Move(4, 3, 7));
	solution_1.add_Move(2, sMultirobotSolution::Move(9, 7, 9));
	solution_1.to_Screen();

	printf("Multirobot solution test 1 ... finished\n");
    }


    void test_multirobot_solver_ID_1(void)
    {
	printf("Multirobot solver ID test 1 ...\n");

	sUndirectedGraph graph_1;
	graph_1.add_Vertices(4);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 0);

	sRobotArrangement arrangement_1(4, 3);
	arrangement_1.place_Robot(1, 0);
	arrangement_1.place_Robot(2, 1);
	arrangement_1.place_Robot(3, 2);

	sRobotArrangement arrangement_2(4, 3);
	arrangement_2.place_Robot(1, 1);
	arrangement_2.place_Robot(2, 2);
	arrangement_2.place_Robot(3, 3);

	sMultirobotSolver_ID solver_ID;

	solver_ID.setup_Solver(10);
	solver_ID.setup_Instance(sMultirobotInstance(graph_1, arrangement_1, arrangement_2));

	sMultirobotSolution solution;

	if (solver_ID.solve_Instance(solution))
	{
	    printf("Solution found !\n");
	    solution.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}
	
	printf("Multirobot solver ID test 1 ... finished\n");
    }


    void test_multirobot_solver_ID_Astar_1(int x, int y, int N_Robots)
    {
	printf("Multirobot solver ID/Astar test 1 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	graph_2.to_Screen();
	arrangement_1.to_Screen();
	arrangement_2.to_Screen();

	printf("\n*** Iterative deepening solver ...\n");

	sMultirobotSolver_ID solver_ID;

	solver_ID.setup_Solver(20);
	solver_ID.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution;

#ifdef sVERBOSE
	clock_t begin_1 = clock();
#endif

	if (solver_ID.solve_Instance(solution))
	{
	    printf("Solution found !\n");
	    solution.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

	printf("\n");

#ifdef sVERBOSE
	clock_t end_1 = clock();
	printf("\nID Solver time: %.6f\n", (end_1 - begin_1) / (double)CLOCKS_PER_SEC);
#endif

	printf("\n*** A* (Astar) solver ...\n");

	sMultirobotSolver_Astar solver_Astar;
	sAstarHeuristic_Trivial trivial_heuristic;

	solver_Astar.setup_Solver(&trivial_heuristic);
	solver_Astar.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_2;

#ifdef sVERBOSE
	clock_t begin_2 = clock();
#endif

	if (solver_Astar.solve_Instance(solution_2))
	{
	    printf("Solution found !\n");
	    solution_2.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_2 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_2 - begin_2) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver ID/Astar test 1 ... finished\n");
    }


    void test_multirobot_solver_Astar_1(int x, int y, int N_Robots)
    {
	printf("Multirobot solver Astar test 1 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** A* (Astar) solver (trivial)...\n");

	sMultirobotSolver_Astar solver_Astar;
	sAstarHeuristic_Trivial trivial_heuristic;

	solver_Astar.setup_Solver(&trivial_heuristic);
	solver_Astar.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_2;

#ifdef sVERBOSE
	clock_t begin_2 = clock();
#endif

	if (solver_Astar.solve_Instance(solution_2))
	{
	    printf("Solution found !\n");
	    solution_2.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_2 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_2 - begin_2) / (double)CLOCKS_PER_SEC);
#endif
    }


    void test_multirobot_solver_Astar_2(int x, int y, int N_Robots)
    {
	printf("Multirobot solver Astar test 2 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** A* (Astar) solver (placement)...\n");

	sMultirobotSolver_Astar solver_Astar_2;
	sAstarHeuristic_Placement placement_heuristic;

	solver_Astar_2.setup_Solver(&placement_heuristic);
	solver_Astar_2.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_3;

#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_Astar_2.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_3 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver Astar test 2 ... finished\n");

    }

    void test_multirobot_solver_Astar_3(int x, int y, int N_Robots)
    {
	printf("Multirobot solver Astar test 3 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** A* (Astar) solver (distance)...\n");

	sMultirobotSolver_Astar solver_Astar_3;
	sAstarHeuristic_Distance distance_heuristic;

	solver_Astar_3.setup_Solver(&distance_heuristic);
	solver_Astar_3.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));
	distance_heuristic.to_String_matrix();

	sMultirobotSolution solution_3;

#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_Astar_3.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_3 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver Astar test 3 ... finished\n");
    }


    void test_multirobot_solver_Astar_4(int x, int y, int N_Robots)
    {
	printf("Multirobot solver Astar test 4 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** A* (Astar) solver (distance)...\n");

	sMultirobotSolver_DecomposedAstar solver_Astar_3;
	sAstarHeuristic_Distance distance_heuristic;

	solver_Astar_3.setup_Solver(&distance_heuristic);
	solver_Astar_3.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));
	distance_heuristic.to_String_matrix();

	sMultirobotSolution solution_3;


#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_Astar_3.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_3 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver Astar test 4 ... finished\n");
    }


    void test_multirobot_solver_IDAstar_1(int x, int y, int N_Robots)
    {
	printf("Multirobot solver IDAstar test 1 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** IDA* (IDAstar) solver (distance)...\n");

	sMultirobotSolver_IDAstar solver_IDAstar_1;
	sAstarHeuristic_Distance distance_heuristic;

	solver_IDAstar_1.setup_Solver(200, &distance_heuristic);
	solver_IDAstar_1.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_3;

#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_IDAstar_1.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_3 = clock();
	printf("\nAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver IDAstar test 1 ... finished\n");
    }


    void test_multirobot_PDDL_1(void)
    {
	printf("Multirobot PDDL test 1 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(6);
	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 0);
	environment.add_Edge(1, 4);
	environment.add_Edge(4, 5);
	environment.add_Edge(5, 2);

	sRobotArrangement initial(6, 4);
	initial.place_Robot(1, 3);
	initial.place_Robot(2, 1);
	initial.place_Robot(3, 0);
	initial.place_Robot(4, 5);

	sRobotArrangement goal(6, 4);
	goal.place_Robot(1, 1);
	goal.place_Robot(2, 3);
	goal.place_Robot(3, 5);
	goal.place_Robot(4, 2);

	sMultirobotInstance multirobot_1(environment, initial, goal);

	multirobot_1.to_Screen();
	multirobot_1.to_Screen_domainPDDL();
	multirobot_1.to_Screen_problemPDDL();

	multirobot_1.to_File_domainPDDL("multirobot.pddl");
	multirobot_1.to_File_problemPDDL("multirobot_01.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Multirobot PDDL test 1 ... finished\n");
    }

    void test_multirobot_PDDL_2(void)
    {
	printf("Multirobot PDDL test 2 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(9);

	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);

	environment.add_Edge(3, 4);
	environment.add_Edge(4, 5);

	environment.add_Edge(6, 7);
	environment.add_Edge(7, 8);

	environment.add_Edge(0, 3);
	environment.add_Edge(3, 6);

	environment.add_Edge(1, 4);
	environment.add_Edge(4, 7);

	environment.add_Edge(2, 5);
	environment.add_Edge(5, 8);

	sRobotArrangement initial(9, 4);
	initial.place_Robot(1, 3);
	initial.place_Robot(2, 1);
	initial.place_Robot(3, 0);
	initial.place_Robot(4, 5);

	sRobotArrangement goal(9, 4);
	goal.place_Robot(1, 1);
	goal.place_Robot(2, 3);
	goal.place_Robot(3, 5);
	goal.place_Robot(4, 2);

	sMultirobotInstance multirobot_2(environment, initial, goal);

	multirobot_2.to_Screen();
	multirobot_2.to_Screen_domainPDDL();
	multirobot_2.to_Screen_problemPDDL();

	multirobot_2.to_File_domainPDDL("multirobot.pddl");
	multirobot_2.to_File_problemPDDL("multirobot_02.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Multirobot PDDL test 2 ... finished\n");
    }


    void test_multirobot_PDDL_3(void)
    {
	printf("Multirobot PDDL test 3 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(9);

	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);

	environment.add_Edge(3, 4);
	environment.add_Edge(4, 5);

	environment.add_Edge(6, 7);
	environment.add_Edge(7, 8);

	environment.add_Edge(0, 3);
	environment.add_Edge(3, 6);

	environment.add_Edge(1, 4);
	environment.add_Edge(4, 7);

	environment.add_Edge(2, 5);
	environment.add_Edge(5, 8);

	sRobotArrangement initial(9, 7);
	initial.place_Robot(1, 1);
	initial.place_Robot(2, 2);
	initial.place_Robot(3, 3);
	initial.place_Robot(4, 4);
	initial.place_Robot(5, 5);
	initial.place_Robot(6, 7);
	initial.place_Robot(7, 8);

	sRobotArrangement goal(9, 7);
	goal.place_Robot(1, 8);
	goal.place_Robot(2, 6);
	goal.place_Robot(3, 2);
	goal.place_Robot(4, 0);
	goal.place_Robot(5, 3);
	goal.place_Robot(6, 1);
	goal.place_Robot(7, 4);

	sMultirobotInstance multirobot_3(environment, initial, goal);

	multirobot_3.to_Screen();
	multirobot_3.to_Screen_domainPDDL();
	multirobot_3.to_Screen_problemPDDL();

	multirobot_3.to_File_domainPDDL("multirobot.pddl");
	multirobot_3.to_File_problemPDDL("multirobot_03.pddl");
	
#ifdef sVERBOSE
#endif
	printf("Multirobot PDDL test 3 ... finished\n");
    }


    void test_multirobot_CNF_1(void)
    {
	printf("Multirobot CNF test 1 ...\n");

	for (int i = 1; i < 10; ++i)
	{
	    printf("Log2(%d) = %d\n", i, sIndexableStateIdentifier::calc_Log2(i));
	}

	sUndirectedGraph environment;
	environment.add_Vertices(4);

	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(1, 3);

	sRobotArrangement initial(4, 2);
	initial.place_Robot(1, 0);
	initial.place_Robot(2, 2);

	sRobotArrangement goal(4, 2);
	goal.place_Robot(1, 2);
	goal.place_Robot(2, 0);

	sMultirobotInstance multirobot_1(environment, initial, goal);

	sMultirobotEncodingContext_CNFsat encoding_context_1i(10);
	multirobot_1.to_File_InverseCNFsat("multirobot_01i.cnf", encoding_context_1i, "", false);
	sMultirobotEncodingContext_CNFsat encoding_context_1d(10);
	multirobot_1.to_File_DifferentialCNFsat("multirobot_01d.cnf", encoding_context_1d, "", false);
	
	printf("Multirobot CNF test 1 ... finished\n");
    }


    void test_multirobot_CNF_2(int N_Layers)
    {
	printf("Multirobot CNF test 2 ...\n");

	sUndirectedGraph environment;
	environment.add_Vertices(9);

	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);

	environment.add_Edge(3, 4);
	environment.add_Edge(4, 5);

	environment.add_Edge(6, 7);
	environment.add_Edge(7, 8);

	environment.add_Edge(0, 3);
	environment.add_Edge(3, 6);

	environment.add_Edge(1, 4);
	environment.add_Edge(4, 7);

	environment.add_Edge(2, 5);
	environment.add_Edge(5, 8);

	sRobotArrangement initial(9, 7);
	initial.place_Robot(1, 0);
	initial.place_Robot(2, 3);
	initial.place_Robot(3, 6);

	initial.place_Robot(4, 2);
	initial.place_Robot(5, 5);
	initial.place_Robot(6, 8);

	initial.place_Robot(7, 1);

	sRobotArrangement goal(9, 7);
	goal.place_Robot(1, 8);
	goal.place_Robot(2, 5);
	goal.place_Robot(3, 2);

	goal.place_Robot(4, 6);
	goal.place_Robot(5, 3);
	goal.place_Robot(6, 0);

	goal.place_Robot(7, 7);

	sMultirobotInstance multirobot_2(environment, initial, goal);

	sMultirobotEncodingContext_CNFsat encoding_context_2i(N_Layers);
	multirobot_2.to_File_InverseCNFsat("multirobot_02i.cnf", encoding_context_2i, "", false);
	sMultirobotEncodingContext_CNFsat encoding_context_2d(N_Layers);
	multirobot_2.to_File_DifferentialCNFsat("multirobot_02d.cnf", encoding_context_2d, "", false);
	
	printf("Multirobot CNF test 2 ... finished\n");
    }


    void test_multirobot_CNF_3(int x, int y, int N_Robots, int N_Layers)
    {
	printf("Multirobot CNF test 3 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}
	sMultirobotInstance multirobot_3(graph_2, arrangement_1, arrangement_2);

	sMultirobotEncodingContext_CNFsat encoding_context_3i(N_Layers);
	multirobot_3.to_File_InverseCNFsat("multirobot_03i.cnf", encoding_context_3i, "", false);
	sMultirobotEncodingContext_CNFsat encoding_context_3d(N_Layers);
	multirobot_3.to_File_DifferentialCNFsat("multirobot_03d.cnf", encoding_context_3d, "", false);
	
	printf("Multirobot CNF test 3 ... finished\n");
    }


    void test_multirobot_solver_HCAstar_1(int x, int y, int N_Robots)
    {
	printf("Multirobot solver HCAstar test 1 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** HCA* (HCAstar) solver (distance)...\n");

	sMultirobotSolver_HCAstar solver_CAstar_2;
	sAstarHeuristic_Distance distance_heuristic;

	printf("Initial arrangement\n");
	arrangement_1.to_Screen();

	printf("Goal arrangement\n");
	arrangement_2.to_Screen();

	solver_CAstar_2.setup_Solver(&distance_heuristic);
	solver_CAstar_2.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_3;

#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_CAstar_2.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	clock_t end_3 = clock();
	printf("\nHCAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver HCAstar test 1 ... finished\n");
    }


    void test_multirobot_solver_HCAstar_2(int x, int y, int N_Robots)
    {
	printf("Multirobot solver HCAstar test 2 ...\n");

	sUndirectedGraph graph_2;
	
	graph_2.add_Vertices(x * y);

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	}

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_1(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = graph_2.get_VertexCount();

	sRobotArrangement arrangement_2(N_Vertices, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	printf("\n*** WHCA* (WHCAstar) solver (distance)...\n");

	sMultirobotSolver_WHCAstar solver_WHCAstar_2;
	sAstarHeuristic_Distance distance_heuristic;

	printf("Initial arrangement\n");
	arrangement_1.to_Screen();

	printf("Goal arrangement\n");
	arrangement_2.to_Screen();

	solver_WHCAstar_2.setup_Solver(&distance_heuristic, 16);
	solver_WHCAstar_2.setup_Instance(sMultirobotInstance(graph_2, arrangement_1, arrangement_2));

	sMultirobotSolution solution_3;

#ifdef sVERBOSE
	clock_t begin_3 = clock();
#endif

	if (solver_WHCAstar_2.solve_Instance(solution_3))
	{
	    printf("Solution found !\n");
	    solution_3.to_Screen();
	}
	else
	{
	    printf("Unable to find solution.\n");
	}

#ifdef sVERBOSE
	s_GlobalPhaseStatistics.to_Screen();

	clock_t end_3 = clock();
	printf("\nWHCAstar Solver time: %.6f\n", (end_3 - begin_3) / (double)CLOCKS_PER_SEC);
#endif
	
	printf("Multirobot solver HCAstar test 2 ... finished\n");

    }


    void test_graph_1(void)
    {
	printf("Graph test 1\n");
	sDigraph digraph;

	digraph.add_Vertex("v1");
	digraph.add_Vertex("v2");
	digraph.add_Vertex("v3");

	digraph.add_Arc("v1", "v2", 3, 0);
	digraph.add_Arc("v2", "v3", 2, 0);
	digraph.add_Arc("v3", "v1", 1, 0);

	digraph.to_Screen();
    }


    void test_graph_2(void)
    {
	printf("Graph test 2\n");
	sDigraph digraph;

	sDigraph::Vertices_map::iterator source = digraph.add_Vertex("s1");
	digraph.add_Vertex("v1");
	digraph.add_Vertex("v2");
	digraph.add_Vertex("v3");
	digraph.add_Vertex("v4");
	sDigraph::Vertices_map::iterator sink = digraph.add_Vertex("s2");

	digraph.add_Arc("s1", "v1", 2, 0);
	digraph.add_Arc("v1", "v2", 1, 0);
	digraph.add_Arc("v2", "v3", 1, 0);
	digraph.add_Arc("v1", "v4", 1, 0);
	digraph.add_Arc("v4", "v3", 1, 0);
	digraph.add_Arc("v3", "s2", 3, 0);

	digraph.to_Screen();

	sGoldberg goldberg;
	goldberg.compute_Flow(&digraph, &source->second, &sink->second);

	printf("\nComputed flow\n");
	digraph.to_Screen();
    }


    void test_graph_3(void)
    {
	printf("Graph test 3\n");
	sUndirectedGraph environment;
	environment.add_Vertices(6);

	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 4);
	environment.add_Edge(1, 5);

	sRobotArrangement initial(6, 3);
	initial.place_Robot(1, 0);
	initial.place_Robot(2, 1);
	initial.place_Robot(3, 5);

	sRobotArrangement goal(6, 3);
	goal.place_Robot(1, 5);
	goal.place_Robot(2, 4);
	goal.place_Robot(3, 3);

	sMultirobotInstance multirobot(environment, initial, goal);
	multirobot.to_Screen();

	sMultirobotFlowModel flow_model(multirobot);
	flow_model.build_Network(4);

	int relocation = flow_model.compute_Relocation();
	printf("Relocation approximation: %d\n", relocation);
    }


    void test_graph_4(void)
    {
	printf("Graph test 4\n");
	sUndirectedGraph environment;
	environment.add_Vertices(15);

	environment.add_Edge(0, 1);
	environment.add_Edge(0, 2);
	environment.add_Edge(1, 3);
	environment.add_Edge(2, 3);

	environment.add_Edge(5, 7);
	environment.add_Edge(5, 6);
	environment.add_Edge(7, 8);
	environment.add_Edge(6, 8);

	environment.add_Edge(2, 9);
	environment.add_Edge(3, 10);
	environment.add_Edge(9, 10);

	environment.add_Edge(6, 11);
	environment.add_Edge(11, 12);
	environment.add_Edge(8, 12);

	environment.add_Edge(3, 13);
	environment.add_Edge(13, 4);

	environment.add_Edge(14, 6);
	environment.add_Edge(4, 14);

	sRobotArrangement initial(15, 6);
	initial.place_Robot(1, 0);
	initial.place_Robot(2, 1);
	initial.place_Robot(3, 2);
	initial.place_Robot(4, 3);
	initial.place_Robot(5, 9);
	initial.place_Robot(6, 10);

	sRobotArrangement goal(15, 6);
	goal.place_Robot(1, 5);
	goal.place_Robot(2, 6);
	goal.place_Robot(3, 7);
	goal.place_Robot(4, 8);
	goal.place_Robot(5, 11);
	goal.place_Robot(6, 12);

	sMultirobotInstance multirobot(environment, initial, goal);
	multirobot.to_Screen();

	sMultirobotFlowModel flow_model(multirobot);

	flow_model.build_Network(9);
	int relocation = flow_model.compute_Relocation();
	printf("Relocation approximation: %d\n", relocation);

	int distance = flow_model.compute_Distance();
	printf("Relocation distance      : %d\n", distance);

	sMultirobotFlowModel::Robots_vector robots_1;
	robots_1.push_back(1);
	robots_1.push_back(2);
	robots_1.push_back(3);

	int sub_distance = flow_model.compute_Distance(robots_1);
	printf("Relocation sub-distance 1: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_2;
	robots_2.push_back(3);
	robots_2.push_back(4);
	robots_2.push_back(5);

	sub_distance = flow_model.compute_Distance(robots_2);
	printf("Relocation sub-distance 2: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_3;
	robots_3.push_back(1);
	robots_3.push_back(3);
	robots_3.push_back(5);

	sub_distance = flow_model.compute_Distance(robots_3);
	printf("Relocation sub-distance 3: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_4;
	robots_4.push_back(2);
	robots_4.push_back(4);
	robots_4.push_back(6);

	sub_distance = flow_model.compute_Distance(robots_4);
	printf("Relocation sub-distance 4: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_5;
	robots_5.push_back(1);
	robots_5.push_back(6);

	sub_distance = flow_model.compute_Distance(robots_5);
	printf("Relocation sub-distance 5: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_6;
	robots_6.push_back(2);
	robots_6.push_back(5);

	sub_distance = flow_model.compute_Distance(robots_6);
	printf("Relocation sub-distance 6: %d\n", sub_distance);

	sMultirobotFlowModel::Robots_vector robots_7;
	robots_7.push_back(3);
	robots_7.push_back(4);

	sub_distance = flow_model.compute_Distance(robots_7);
	printf("Relocation sub-distance 7: %d\n", sub_distance);

	int N_tries[] = { 3, 4, 5, 4, 3, 1 };
	int multi_distance = flow_model.compute_Distance(N_tries);
	printf("Relocation multi-distance: %d\n", multi_distance);
    }


    void test_graph_5(void)
    {
	printf("Graph test 5\n");

	sUndirectedGraph environment;
	environment.add_Vertices(6);
	environment.add_Edge(0, 1);
	environment.add_Edge(1, 2);
	environment.add_Edge(2, 3);
	environment.add_Edge(3, 0);
	environment.add_Edge(1, 4);
	environment.add_Edge(4, 5);
	environment.add_Edge(5, 2);

	sRobotArrangement initial(6, 4);
	initial.place_Robot(1, 3);
	initial.place_Robot(2, 1);
	initial.place_Robot(3, 0);
	initial.place_Robot(4, 5);

	sRobotArrangement goal(6, 4);
	goal.place_Robot(1, 1);
	goal.place_Robot(2, 3);
	goal.place_Robot(3, 5);
	goal.place_Robot(4, 2);

	sMultirobotInstance multirobot(environment, initial, goal);
	sMultirobotFlowModel flow_model(multirobot);

	int N_tries[] = { 3, 4, 5, 4, 3, 1 };
	int multi_distance = flow_model.compute_Distance(N_tries);
	printf("Relocation multi-distance: %d\n", multi_distance);
    }


    void test_graph_6(int x, int y, int N_Robots)
    {
	printf("Graph test 6\n");

	sUndirectedGraph graph_2;
	graph_2.add_Vertices(2 * x * y);

	int half = x * y;
	int full = 2 * half;

	for (int i = 0; i < y-1; ++i)
	{
	    for (int j = 0; j < x-1; ++j)
	    {
		graph_2.add_Edge(i * x + j, (i + 1) * x + j);
		graph_2.add_Edge(i * x + j, i * x + (j + 1));

		graph_2.add_Edge(half + i * x + j, half + (i + 1) * x + j);
		graph_2.add_Edge(half + i * x + j, half + i * x + (j + 1));
	    }
	}
	
	for (int i = 0; i < y-1; ++i)
	{
	    graph_2.add_Edge(i * x + (x - 1), (i + 1) * x + (x - 1));
	    graph_2.add_Edge(half + i * x + (x - 1), half + (i + 1) * x + (x - 1));
	}

	for (int j = 0; j < x-1; ++j)
	{
	    graph_2.add_Edge((y - 1) * x + j, (y - 1) * x + j + 1);
	    graph_2.add_Edge(half + (y - 1) * x + j, half + (y - 1) * x + j + 1);
	}
	graph_2.add_Edge(0, half);

	int Remaining_cnt, N_Vertices;
	N_Vertices = Remaining_cnt = half;

	sRobotArrangement arrangement_1(full, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_1.get_VertexOccupancy(v_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_1.place_Robot(r_id, v_id);
			break;
		    }
		}
	    }
	}

	N_Vertices = Remaining_cnt = half;

	sRobotArrangement arrangement_2(full, N_Robots);

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int position = random() % Remaining_cnt--;
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		if (arrangement_2.get_VertexOccupancy(v_id + half) == sRobotArrangement::VACANT_VERTEX)
		{
		    if (position-- <= 0)
		    {
			arrangement_2.place_Robot(r_id, v_id + half);
			break;
		    }
		}
	    }
	}

	sMultirobotInstance multirobot(graph_2, arrangement_1, arrangement_2);
	sMultirobotFlowModel flow_model(multirobot);

	int N_tries[] = { 16, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
	int multi_distance = flow_model.compute_Distance(N_tries);
	printf("Relocation multi-distance: %d\n", multi_distance);
    }


    int test_multirobot_instance_2(const sString &filename)
    {
	FILE *fr;
	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return -1;
	}
	
	sUndirectedGraph graph;
	sResult result;
	
	if (sFAILED(result = graph.from_File_multirobot(filename)))
	{
	    return result;
	}
	
	graph.to_Screen();              
	fclose(fr);
	
	return sRESULT_SUCCESS;
    }


    int test_multirobot_dibox_20(void)
    {
	sResult result;

	FILE *fw_names;
	fw_names = fopen("exampleSmall20_names.txt", "w");

	if (fw_names == NULL)
	{
	    return -1;
	}
			
	sUndirectedGraph biconnected_digraph(true, 5, 2, 5, 20);
	
	for (int i = 0; i < 10; ++i)
	{
	    for (int a = 1; a < biconnected_digraph.get_VertexCount() - 1; ++a)
	    {
		sRobotArrangement initial_arrangement(biconnected_digraph.get_VertexCount(), a, true);
		sRobotArrangement goal_arrangement(biconnected_digraph.get_VertexCount(), a, true);
		
//		sRobotArrangement goal_arrangement = initial_arrangement;
//		goal_arrangement.generate_NovelWalk(initial_arrangement, biconnected_digraph);
		
		sMultirobotInstance instance(biconnected_digraph, initial_arrangement, goal_arrangement);

		char filename[256];
		sprintf(filename, "exampleSmall20_a%d_%d.dat", a, i);
		fprintf(fw_names, "%s\n", filename);
		
		if (sFAILED(result = instance.to_File_dibox(filename)))
		{
		    return result;
		}
	    }
	}
	fclose(fw_names);
	
	return sRESULT_SUCCESS;
    }

    
    int test_multirobot_dibox_40(void)
    {
	sResult result;

	FILE *fw_names;
	fw_names = fopen("exampleSmall40_names.txt", "w");

	if (fw_names == NULL)
	{
	    return -1;
	}	

	sUndirectedGraph biconnected_digraph(true, 5, 2, 8, 40);
	    
	for (int i = 0; i < 10; ++i)
	{
	    for (int a = 1; a < biconnected_digraph.get_VertexCount() / 2; ++a)
	    {
		sRobotArrangement initial_arrangement(biconnected_digraph.get_VertexCount(), a, true);
		sRobotArrangement goal_arrangement(biconnected_digraph.get_VertexCount(), a, true);
		
//		sRobotArrangement goal_arrangement = initial_arrangement;
//		goal_arrangement.generate_NovelWalk(initial_arrangement, biconnected_digraph);
		
		sMultirobotInstance instance(biconnected_digraph, initial_arrangement, goal_arrangement);
		char filename[256];

		sprintf(filename, "exampleSmall40_a%d_%d.dat", a, i);
		fprintf(fw_names, "%s\n", filename);
		
		if (sFAILED(result = instance.to_File_dibox(filename)))
		{
		    return result;
		}
	    }
	}
	fclose(fw_names);
		
	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    /*
    test_robot_arrangement_1();
    test_multirobot_instance_1();

    test_multirobot_solution_1();
    */
    /*
    test_multirobot_solver_ID_1();

    test_multirobot_solver_ID_Astar_1(3, 3, 5);
        
    test_multirobot_solver_Astar_1(3, 3, 3);
    test_multirobot_solver_Astar_2(3, 3, 3);
    test_multirobot_solver_Astar_3(3, 3, 3);
    */
    //test_multirobot_solver_IDAstar_1(4, 4, 8);
    
    /*
    test_multirobot_PDDL_1();
    test_multirobot_PDDL_2();
    test_multirobot_PDDL_3();
    */

    /*
    test_multirobot_CNF_1();
    test_multirobot_CNF_2(15);
    test_multirobot_CNF_3(4, 4, 4, 8);
    
    test_multirobot_CNF_3(16, 16, 4, 20);
    test_multirobot_CNF_3(32, 32, 4, 40);

    test_multirobot_solver_HCAstar_1(8, 8, 20);
    test_multirobot_solver_HCAstar_2(8, 8, 32);

    test_robot_goal_1();
    */
    //test_graph_1();
    //test_graph_2();
    //test_graph_3();
    //test_graph_4();
    //test_graph_5();
    //test_graph_6(4, 4, 6);
    //test_graph_6(6, 6, 16);

    //test_multirobot_solver_Astar_4(5, 5, 8);
    /*
    if (test_multirobot_instance_2("test_grid.cpf") < 0)
    {
	printf("Error: testing instance\n");
	return -1;
    }
    */

    if (test_multirobot_dibox_20())
    {
	printf("Error: testing dibox\n");
	return -1;
    }
    if (test_multirobot_dibox_40())
    {
	printf("Error: testing dibox\n");
	return -1;
    } 
/*
    if (test_multirobot_dibox_ext_20())
    {
	printf("Error: testing dibox ext\n");
	return -1;
    }
    if (test_multirobot_dibox_ext_40())
    {
	printf("Error: testing dibox ext\n");
	return -1;
    }     
*/
    return 0;
}

