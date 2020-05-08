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
/* complete_test.cpp / 0.21-robik_041                                         */
/*----------------------------------------------------------------------------*/
//
// Complete CPF solver - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "cnf.h"
#include "compress.h"
#include "complete.h"
#include "statistics.h"

#include "complete_test.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{

/*----------------------------------------------------------------------------*/

    struct ResultItem
    {
	ResultItem()
	    : m_standard_moves(0)
	    , m_twin_moves(0)
	{	    
	}

	ResultItem(double standard_moves, double twin_moves)
	    : m_standard_moves(standard_moves)
	    , m_twin_moves(twin_moves)
	{	    
	}

	double m_standard_moves;
	double m_twin_moves;
    };

    typedef std::vector<ResultItem> ResultItems_vector;

/*----------------------------------------------------------------------------*/

    void test_complete_1(void)
    {
	ResultItems_vector result_Items;

	printf("Complete test 0 ...\n");
	sUndirectedGraph biconnected_graph(5, 5, 8, 12);
	printf("Constructed\n");
	biconnected_graph.to_Screen();
	//	exit(0);

	printf("Complete test 1 ...\n");
/*
	srand(1235);
//	srand(1231);
//	srand(3234);
//	srand(211);
//	srand(371);
//	srand(58);
//	srand(647);
//	srand(731);
//	srand(22);
//	srand(94);

	sUndirectedGraph environment(4, 4, 0.1);
	sRobotArrangement initial_arrangement(environment.get_VertexCount(), 8, true);
	sRobotArrangement goal_arrangement(environment.get_VertexCount(), 8);
	goal_arrangement.generate_Walk(initial_arrangement, environment);
*/
	int total_standard = 0;
	int total_twin = 0;
	
	int seeds[] = {	1235, 1231, 3234, 211, 371,   58, 647,  731,   22,  94,
			 428,  263,   33, 728,  74, 1223, 278, 4353, 9090,  37,
	                 381,  762,   24, 717, 346,  238,  87,   50,  181, 436,
			 128,  981,  567, 159, 472,  991,  18,  845,  886, 648 };
	
	//	int seeds[] = {	1235 };

	sUndirectedGraph environment;

	for (int c = 0; c < 8; ++c)
	{
	printf("Working on case: %d\n", c);

	switch (c)
	{
	case 0:
	{
	    environment = sUndirectedGraph(5, 2, 3, 256);
	    break;
	}
	case 1:
	{
	    environment = sUndirectedGraph(7, 3, 5, 256);
	    break;
	}
	case 3:
	{
	    environment = sUndirectedGraph(7, 7, 9, 256);
	    break;
	}
	case 4:
	{
	    environment = sUndirectedGraph(7, 11, 13, 256);
	    break;
	}
	case 5:
	{
	    environment = sUndirectedGraph(7, 14, 18, 256);
	    break;
	}
	case 6:
	{
	    environment = sUndirectedGraph(7, 20, 28, 256);
	    break;
	}
	case 7:
	{
	    environment = sUndirectedGraph(7, 24, 40, 256);
	    break;
	}
	default:
	{
	    break;
	}
	}

	clock_t start_std, end_std, start_twin, end_twin, total_time_std, total_time_twin;
	for (int i = 1; i <= environment.get_VertexCount() - 2; ++i)
	{
	    printf("Robots: %d (case:%d)\n", i, c);

	    int sum_standard = 0;
	    int sum_twin = 0;

	    total_time_std = 0;
	    total_time_twin = 0;

	    for (int j = 0; j < sizeof(seeds) / sizeof(int); ++j)
	    {
		srand(seeds[j]);

		//	    sUndirectedGraph environment(32, 32, 0.2);
		sRobotArrangement initial_arrangement(environment.get_VertexCount(), i, true);
		sRobotArrangement goal_arrangement(environment.get_VertexCount(), i);
		goal_arrangement.generate_Walk(initial_arrangement, environment);
	    
		sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);
	    /*
	    environment.to_Screen();
	    initial_arrangement.to_Screen();
	    goal_arrangement.to_Screen();	    
	    instance.to_Screen();
	    */
		sCompleteSolver complete_solver;
		sCompleteSolver complete_solver_twin;

		complete_solver.setup_Solver();
		complete_solver.setup_Instance(instance);
	    
		complete_solver_twin.setup_Solver();
		complete_solver_twin.setup_Instance(instance);

	    /*
	      sCompleteSolver::VertexIDs_set locked_Vertices;
	      complete_solver.search_SingleSourcePaths(2, locked_Vertices);
	      
	      complete_solver.m_multirobot_instance.to_Screen();
	      complete_solver.push(2, 6, locked_Vertices);
	    */
	    /*
	      int pretarget_id;
	      complete_solver.multipush(9, 13, 3, pretarget_id);
	      printf("Pretarget: %d\n", pretarget_id);
	    */
	    //	complete_solver.swap(8, 9);
	    //	complete_solver.solve();
	    
		sCompleteSolver::VertexIDs_set locked_Vertices;
	    //	complete_solver.push_Twin(9, 13, 6, locked_Vertices);
	    //	complete_solver.push_Twin(6, 9, 5, locked_Vertices);
	    /*
	      int pretarget_id, prepretarget_id;
	      complete_solver.multipush_Twin(13, 9, 8, 5, pretarget_id, prepretarget_id);
	      printf("pretargets:%d,%d\n", pretarget_id, prepretarget_id);
	      
	      int vacant_1, vacant_2, vacant_3;
	      complete_solver.clear_Twin(8, 9, 5, vacant_1, vacant_2, vacant_3);
	      printf("Vacants:%d,%d,%d\n", vacant_1, vacant_2, vacant_3);
	    */
	    //	complete_solver.relocate_TwinRobots(8, 9, 7, locked_Vertices);
	    //	complete_solver.swap_Twin(10, 7, 11);

		start_std = clock();
		complete_solver.solve();
		end_std = clock();
		total_time_std += end_std - start_std;

		start_twin = clock();
		complete_solver_twin.solve_Twin();
		end_twin = clock();
		total_time_twin += end_twin - start_twin;
	    
	    //	complete_solver.m_multirobot_instance.to_Screen();
	    //	    complete_solver.m_multirobot_solution.to_Screen();
	    //	    complete_solver_twin.m_multirobot_solution.to_Screen();
	    
		sum_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
		sum_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;

		printf("Comparison: %d x %d (twin)\n", complete_solver.m_multirobot_solution.m_Moves_cnt, complete_solver_twin.m_multirobot_solution.m_Moves_cnt);
		printf("Compression ratio: %.4f\n", (double)complete_solver_twin.m_multirobot_solution.m_Moves_cnt / complete_solver.m_multirobot_solution.m_Moves_cnt);
		
		total_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
		total_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;
	    }
	    result_Items.push_back(ResultItem((double)sum_standard / sizeof(seeds), (double)sum_twin / sizeof(seeds)));
	    printf("----> Absolute: %.4f x %.4f\n", result_Items.back().m_standard_moves, result_Items.back().m_twin_moves);
	    printf("----> Ratio: %.4f \n", result_Items.back().m_twin_moves / result_Items.back().m_standard_moves);
	    printf("----> Time std : %.4f\n", (double)total_time_std / CLOCKS_PER_SEC);
	    printf("----> Time twin: %.4f\n", (double)total_time_twin / CLOCKS_PER_SEC);
	    printf("\n");
	}
	printf("\nOverall statistics for case: %d\n", c);
	for (ResultItems_vector::const_iterator item = result_Items.begin(); item != result_Items.end(); ++item)
	{
	    printf("Comparison: %.4f x %.4f (twin) ... %.4f\n",
		   item->m_standard_moves, item->m_twin_moves,
		   item->m_twin_moves / item->m_standard_moves);
	    
	}
	printf("\n");
	result_Items.clear();
	}
	printf("Total: %d x %d (twin) ... %.4f\n", total_standard, total_twin, (double)total_twin / total_standard);
    }


    void test_complete_puzzle(int size_x, int size_y)
    {
	ResultItems_vector result_Items;

	printf("Complete test 2 ...\n");

	int total_standard = 0;
	int total_twin = 0;
	
	int seeds[] = {	1235, 1231, 3234, 211, 371,   58, 647,  731,   22,  94,
			 428,  263,   33, 728,  74, 1223, 278, 4353, 9090,  37,
	                 381,  762,   24, 717, 346,  238,  87,   50,  181, 436,
			 128,  981,  567, 159, 472,  991,  18,  845,  886, 648 };
	
	//	int seeds[] = {	1235 };

	sUndirectedGraph environment;
	environment = sUndirectedGraph(size_x, size_y);

	clock_t start_std, end_std, start_twin, end_twin, total_time_std, total_time_twin;

	printf("CASE: %d x %d, Robots: %d\n", size_x, size_y, environment.get_VertexCount() - 1);

	int sum_standard = 0;
	int sum_twin = 0;

	total_time_std = 0;
	total_time_twin = 0;

	for (int j = 0; j < sizeof(seeds) / sizeof(int); ++j)
	{
	    srand(seeds[j]);
	    
	    sRobotArrangement initial_arrangement(environment.get_VertexCount(), environment.get_VertexCount() - 2, true);
	    sRobotArrangement goal_arrangement(environment.get_VertexCount(), environment.get_VertexCount() - 2);
	    goal_arrangement.generate_Walk(initial_arrangement, environment);
	    
	    sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);

	    sCompleteSolver complete_solver;
	    sCompleteSolver complete_solver_twin;
	    
	    complete_solver.setup_Solver();
	    complete_solver.setup_Instance(instance);
	    
	    complete_solver_twin.setup_Solver();
	    complete_solver_twin.setup_Instance(instance);
	    
	    sCompleteSolver::VertexIDs_set locked_Vertices;

	    start_std = clock();
	    if (!complete_solver.solve())
	    {
		printf("************************************************************\n");
	    }
	    end_std = clock();
	    total_time_std += end_std - start_std;
	    
	    start_twin = clock();
	    if (!complete_solver_twin.solve_Twin())
	    {
		printf("************************************************************\n");
	    }
	    end_twin = clock();
	    total_time_twin += end_twin - start_twin;
	    
	    
	    sum_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
	    sum_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;

	    printf("Comparison: %d x: %d (twin)\n", complete_solver.m_multirobot_solution.m_Moves_cnt, complete_solver_twin.m_multirobot_solution.m_Moves_cnt);
	    printf("Compression ratio: %.4f\n", (double)complete_solver_twin.m_multirobot_solution.m_Moves_cnt / complete_solver.m_multirobot_solution.m_Moves_cnt);
		
	    total_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
	    total_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;

	    result_Items.push_back(ResultItem(complete_solver.m_multirobot_solution.m_Moves_cnt, complete_solver_twin.m_multirobot_solution.m_Moves_cnt));
	    printf("----> Absolute: %.4f x: %.4f\n", result_Items.back().m_standard_moves, result_Items.back().m_twin_moves);
	    printf("----> Ratio: %.4f \n", result_Items.back().m_twin_moves / result_Items.back().m_standard_moves);
	    printf("\n");
	}
	printf("Time std : %.4f\n", (double)total_time_std / CLOCKS_PER_SEC);
	printf("Time twin: %.4f\n", (double)total_time_twin / CLOCKS_PER_SEC);

	printf("\nOverall statistics for case\n");
	for (ResultItems_vector::const_iterator item = result_Items.begin(); item != result_Items.end(); ++item)
	{
	    printf("Comparison: %.4f x: %.4f (twin) ...: %.4f\n",
		   item->m_standard_moves, item->m_twin_moves,
		   item->m_twin_moves / item->m_standard_moves);
	    
	}
	printf("\n");
	result_Items.clear();
	printf("Total: %d x: %d (twin) ...: %.4f\n", total_standard, total_twin, (double)total_twin / total_standard);
    }


    void test_complete_biconnected(void)
    {
	ResultItems_vector result_Items;

	printf("Complete test 0 ...\n");
	sUndirectedGraph biconnected_graph(5, 5, 8, 12);
	printf("Constructed\n");
	biconnected_graph.to_Screen();
	//	exit(0);

	printf("Complete test 1 ...\n");
/*
	srand(1235);
//	srand(1231);
//	srand(3234);
//	srand(211);
//	srand(371);
//	srand(58);
//	srand(647);
//	srand(731);
//	srand(22);
//	srand(94);

	sUndirectedGraph environment(4, 4, 0.1);
	sRobotArrangement initial_arrangement(environment.get_VertexCount(), 8, true);
	sRobotArrangement goal_arrangement(environment.get_VertexCount(), 8);
	goal_arrangement.generate_Walk(initial_arrangement, environment);
*/
	int total_standard = 0;
	int total_twin = 0;
	
	int seeds[] = {	1235, 1231, 3234, 211, 371,   58, 647,  731,   22,  94,
			 428,  263,   33, 728,  74, 1223, 278, 4353, 9090,  37,
	                 381,  762,   24, 717, 346,  238,  87,   50,  181, 436,
			 128,  981,  567, 159, 472,  991,  18,  845,  886, 648 };
	
	//	int seeds[] = {	1235 };

	sUndirectedGraph environment;

	for (int c = 0; c < 6; ++c)
	{
	printf("Working on case: %d\n", c);

	switch (c)
	{
	case 0:
	{
	    environment = sUndirectedGraph(5, 1, 3, 90);
	    break;
	}
	case 1:
	{
	    environment = sUndirectedGraph(7, 3, 5, 90);
	    break;
	}
	case 2:
	{
	    environment = sUndirectedGraph(7, 5, 7, 90);
	    break;
	}
	case 3:
	{
	    environment = sUndirectedGraph(7, 7, 9, 90);
	    break;
	}
	case 4:
	{
	    environment = sUndirectedGraph(7, 8, 12, 90);
	    break;
	}
	case 5:
	{
	    environment = sUndirectedGraph(7, 10, 14, 90);
	    break;
	}
	default:
	{
	    break;
	}
	}

	clock_t start_std, end_std, start_twin, end_twin, total_time_std, total_time_twin;
	for (int i = 1; i <= environment.get_VertexCount() - 2; ++i)
	{
	    printf("Robots: %d (case:%d)\n", i, c);

	    int sum_standard = 0;
	    int sum_twin = 0;

	    total_time_std = 0;
	    total_time_twin = 0;

	    for (int j = 0; j < sizeof(seeds) / sizeof(int); ++j)
	    {
		srand(seeds[j]);

		//	    sUndirectedGraph environment(32, 32, 0.2);
		sRobotArrangement initial_arrangement(environment.get_VertexCount(), i, true);
		sRobotArrangement goal_arrangement(environment.get_VertexCount(), i);
		goal_arrangement.generate_Walk(initial_arrangement, environment);
	    
		sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);
	    /*
	    environment.to_Screen();
	    initial_arrangement.to_Screen();
	    goal_arrangement.to_Screen();	    
	    instance.to_Screen();
	    */
		sCompleteSolver complete_solver;
		sCompleteSolver complete_solver_twin;

		complete_solver.setup_Solver();
		complete_solver.setup_Instance(instance);
	    
		complete_solver_twin.setup_Solver();
		complete_solver_twin.setup_Instance(instance);

	    /*
	      sCompleteSolver::VertexIDs_set locked_Vertices;
	      complete_solver.search_SingleSourcePaths(2, locked_Vertices);
	      
	      complete_solver.m_multirobot_instance.to_Screen();
	      complete_solver.push(2, 6, locked_Vertices);
	    */
	    /*
	      int pretarget_id;
	      complete_solver.multipush(9, 13, 3, pretarget_id);
	      printf("Pretarget: %d\n", pretarget_id);
	    */
	    //	complete_solver.swap(8, 9);
	    //	complete_solver.solve();
	    
		sCompleteSolver::VertexIDs_set locked_Vertices;
	    //	complete_solver.push_Twin(9, 13, 6, locked_Vertices);
	    //	complete_solver.push_Twin(6, 9, 5, locked_Vertices);
	    /*
	      int pretarget_id, prepretarget_id;
	      complete_solver.multipush_Twin(13, 9, 8, 5, pretarget_id, prepretarget_id);
	      printf("pretargets:%d,%d\n", pretarget_id, prepretarget_id);
	      
	      int vacant_1, vacant_2, vacant_3;
	      complete_solver.clear_Twin(8, 9, 5, vacant_1, vacant_2, vacant_3);
	      printf("Vacants:%d,%d,%d\n", vacant_1, vacant_2, vacant_3);
	    */
	    //	complete_solver.relocate_TwinRobots(8, 9, 7, locked_Vertices);
	    //	complete_solver.swap_Twin(10, 7, 11);

		start_std = clock();
		complete_solver.solve();
		end_std = clock();
		total_time_std += end_std - start_std;

		start_twin = clock();
		complete_solver_twin.solve_Twin();
		end_twin = clock();
		total_time_twin += end_twin - start_twin;
	    
	    //	complete_solver.m_multirobot_instance.to_Screen();
	    //	    complete_solver.m_multirobot_solution.to_Screen();
	    //	    complete_solver_twin.m_multirobot_solution.to_Screen();
	    
		sum_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
		sum_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;

		printf("Comparison: %d x %d (twin)\n", complete_solver.m_multirobot_solution.m_Moves_cnt, complete_solver_twin.m_multirobot_solution.m_Moves_cnt);
		printf("Compression ratio: %.4f\n", (double)complete_solver_twin.m_multirobot_solution.m_Moves_cnt / complete_solver.m_multirobot_solution.m_Moves_cnt);
		
		total_standard += complete_solver.m_multirobot_solution.m_Moves_cnt;
		total_twin += complete_solver_twin.m_multirobot_solution.m_Moves_cnt;
	    }
	    result_Items.push_back(ResultItem((double)sum_standard / sizeof(seeds), (double)sum_twin / sizeof(seeds)));
	    printf("----> Absolute: %.4f x %.4f\n", result_Items.back().m_standard_moves, result_Items.back().m_twin_moves);
	    printf("----> Ratio: %.4f \n", result_Items.back().m_twin_moves / result_Items.back().m_standard_moves);
	    printf("----> Time std : %.4f\n", (double)total_time_std / CLOCKS_PER_SEC);
	    printf("----> Time twin: %.4f\n", (double)total_time_twin / CLOCKS_PER_SEC);
	    printf("\n");
	}
	printf("\nOverall statistics for case: %d\n", c);
	for (ResultItems_vector::const_iterator item = result_Items.begin(); item != result_Items.end(); ++item)
	{
	    printf("Comparison: %.4f x %.4f (twin) ... %.4f\n",
		   item->m_standard_moves, item->m_twin_moves,
		   item->m_twin_moves / item->m_standard_moves);
	    
	}
	printf("\n");
	result_Items.clear();
	}
	printf("Total: %d x %d (twin) ... %.4f\n", total_standard, total_twin, (double)total_twin / total_standard);
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
//    test_complete_biconnected();

    test_complete_puzzle(4, 4);
    test_complete_puzzle(16, 16);
/*
    test_complete_puzzle(30, 30);
    test_complete_puzzle(50, 50);
*/
//    test_complete_puzzle(45, 45);
/*
    for (int s = 4; s <= 50; ++s)
    {
	test_complete_puzzle(s, s);
    }
*/
}

