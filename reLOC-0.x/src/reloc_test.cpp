/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.22-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* reloc_test.cpp / 0.22-robik_094                                            */
/*----------------------------------------------------------------------------*/
//
// Relocation problem solving package - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_undirected_graph_1(void)
    {
	printf("Undirecteg graph test 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.to_Screen();
	graph_1.add_Vertex();
	graph_1.to_Screen();

	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();
	graph_1.add_Vertex();

	graph_1.add_Vertices(10);
	graph_1.to_Screen();

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(0, 2);
	graph_1.to_Screen();

	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(2, 4);
	graph_1.add_Edge(7, 4);
	graph_1.add_Edge(8, 4);
	graph_1.add_Edge(9, 4);
	graph_1.add_Edge(10, 4);
	graph_1.add_Edge(11, 4);
	graph_1.to_Screen();

	printf("Undirecteg graph test 1 ... finished\n");
    }


    void test_undirected_graph_2(int N_Vertices, double edge_prob)
    {
	printf("Undirecteg graph test 2 ...\n");

	sUndirectedGraph graph_2;
	graph_2.to_Screen();
	graph_2.add_Vertices(N_Vertices);
	graph_2.to_Screen();

	for (int i = 0; i < N_Vertices - 1; ++i)
	{
	    for (int j = i + 1; j < N_Vertices; ++j)
	    {
		double rnd = rand() / (double)RAND_MAX;
		if (rnd < edge_prob)
		{
		    graph_2.add_Edge(i, j);
		}
	    }
	}

	graph_2.to_Screen_vertices();

	printf("Undirecteg graph test 2 ... finished\n");
    }


    void test_undirected_graph_3(const sString &filename)
    {
	printf("Undirecteg graph test 3 ...\n");
	sUndirectedGraph graph_3;

	sResult result = graph_3.from_File_multirobot(filename);
	if (sFAILED(result))
	{
	    printf("Reading graph from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading graph from file %s succeeded.\n", filename.c_str());
	}
	graph_3.to_Screen_vertices();

	sRobotArrangement arrangement;
	result = arrangement.from_File_multirobot(filename);
	if (sFAILED(result))
	{
	    printf("Reading arrangement from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading arrangement from file %s succeeded.\n", filename.c_str());
	}
	arrangement.to_Screen();

	sMultirobotSolution solution;
	result = solution.from_File_multirobot(filename);
	if (sFAILED(result))
	{
	    printf("Reading solution from file %s failed.\n", filename.c_str());
	    return;
	}
	else
	{
	    printf("Reading solution from file %s succeeded.\n", filename.c_str());
	}
	solution.to_Screen();

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 4);
	sMultirobotSolutionCompressor::Arrangements_vector unfolded_Arrangements;

	if (compressor.verify_Unfolding(arrangement, solution, graph_3))
	{
	    printf("Unfolding verification succeeded.\n");
	}
	else
	{
	    printf("Unfolding verification failed.\n");
	    return;
	}
	compressor.unfold_Solution(arrangement, solution, unfolded_Arrangements);	

	printf("Undirecteg graph test 3 ... finished\n");
    }


    void test_undirected_graph_4(void)
    {
	printf("Undirecteg graph test 4 ...\n");
	sUndirectedGraph graph_1(2, 2);
	graph_1.to_Screen();

	sUndirectedGraph graph_2(2, 3);
	graph_2.to_Screen(); 

	sUndirectedGraph graph_3(3, 3);
	graph_3.to_Screen();

	sUndirectedGraph graph_4(5, 5, 0.2);
	graph_4.to_Screen();

	sRobotArrangement arrangement_1(10, 5);
	arrangement_1.to_Screen();

	sRobotArrangement arrangement_2(10, 5, true);
	arrangement_2.to_Screen();

	printf("Undirecteg graph test 4 ... finished\n");
    }


    void test_undirected_graph_5(void)
    {
	printf("Undirecteg graph test 5 ...\n");
	sUndirectedGraph graph_1;
	graph_1.add_Vertices(8);
	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(4, 5);
	graph_1.add_Edge(3, 6);
	//	graph_1.add_Edge(4, 7);
	graph_1.to_Screen();

	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 1, 2, 3));
	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 1, 1, 2));
	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 1, 3, 4));
	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 1, 4, 5));
	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 1, 1, 0));
	printf("Coop path:%d\n", graph_1.calc_ShortestCoopPath(0, 5, 5, 0));

	sUndirectedGraph graph_2;
	if (sFAILED(graph_2.from_File_map("../maps/small001.map")))
	{
	    printf("Cannot read map from file.\n");
	    return;
	}

	sVectorGraph vector_graph_1(graph_1);
	vector_graph_1.to_Screen();

	sVectorGraph vector_graph_2(graph_2);
	vector_graph_2.to_Screen();

	printf("Undirecteg graph test 5 ... finished\n");
    }


    void test_statistics_1(void)
    {
	double wc_start = sPhaseStatistics::get_WC_Seconds();
	double cpu_start = sPhaseStatistics::get_CPU_Seconds();

	int j = 1;
	for (int i  = 0; i < 100; ++i)
	{
	    system("ls");
	    for (int ii  = 0; ii < 100000; ++ii)
	    {
		j *= i * ii;
	    }
	}

	double wc_finish = sPhaseStatistics::get_WC_Seconds();
	double cpu_finish = sPhaseStatistics::get_CPU_Seconds();
	printf("Wall clock time: %.3f\n", wc_finish - wc_start);
	printf("CPU time       : %.3f\n", cpu_finish - cpu_start);

	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;

	s_GlobalPhaseStatistics.to_Screen();
	s_GlobalPhaseStatistics.enter_Phase("extreme");

	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;

	s_GlobalPhaseStatistics.to_Screen();
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    test_undirected_graph_1();

    test_undirected_graph_2(16, 0.1);
    test_undirected_graph_2(16, 0.5);
    test_undirected_graph_2(32, 0.6);
    test_undirected_graph_2(64, 0.9);

    test_undirected_graph_3("graph_01.txt");
    //test_statistics_1();

    test_undirected_graph_4();
    test_undirected_graph_5();
}


