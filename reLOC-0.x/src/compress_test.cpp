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
/* compress_test.cpp / 0.20-kruh_054                                          */
/*----------------------------------------------------------------------------*/
//
// Compression tools for relocation problem solutions - testing program.
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
#include "statistics.h"

#include "compress_test.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_solution_compression_1(void)
    {
	printf("Solution compression test 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertices(24);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(4, 5);

	graph_1.add_Edge(6, 7);
	graph_1.add_Edge(7, 8);
	graph_1.add_Edge(8, 9);
	graph_1.add_Edge(9, 10);
	graph_1.add_Edge(10, 11);

	graph_1.add_Edge(12, 13);
	graph_1.add_Edge(13, 14);
	graph_1.add_Edge(14, 15);
	graph_1.add_Edge(15, 16);
	graph_1.add_Edge(16, 17);

	graph_1.add_Edge(18, 19);
	graph_1.add_Edge(19, 20);
	graph_1.add_Edge(20, 21);
	graph_1.add_Edge(21, 22);
	graph_1.add_Edge(22, 23);

	graph_1.add_Edge(0, 6);
	graph_1.add_Edge(6, 12);
	graph_1.add_Edge(12, 18);

	graph_1.add_Edge(1, 7);
	graph_1.add_Edge(7, 13);
	graph_1.add_Edge(13, 19);

	graph_1.add_Edge(2, 8);
	graph_1.add_Edge(8, 14);
	graph_1.add_Edge(14, 20);

	graph_1.add_Edge(3, 9);
	graph_1.add_Edge(9, 15);
	graph_1.add_Edge(15, 21);

	graph_1.add_Edge(4, 10);
	graph_1.add_Edge(10, 16);
	graph_1.add_Edge(16, 22);

	graph_1.add_Edge(5, 11);
	graph_1.add_Edge(11, 17);
	graph_1.add_Edge(17, 23);

	sRobotArrangement arrangement_1(24, 16);
	arrangement_1.place_Robot(1, 0);
	arrangement_1.place_Robot(2, 1);
	arrangement_1.place_Robot(3, 6);
	arrangement_1.place_Robot(4, 7);
	arrangement_1.place_Robot(5, 12);
	arrangement_1.place_Robot(6, 13);
	arrangement_1.place_Robot(7, 18);
	arrangement_1.place_Robot(8, 19);

	arrangement_1.place_Robot(9, 4);
	arrangement_1.place_Robot(10, 5);
	arrangement_1.place_Robot(11, 10);
	arrangement_1.place_Robot(12, 11);
	arrangement_1.place_Robot(13, 16);
	arrangement_1.place_Robot(14, 17);
	arrangement_1.place_Robot(15, 22);
	arrangement_1.place_Robot(16, 23);

	sMultirobotSolution solution_1;

	solution_1.add_Move(0, sMultirobotSolution::Move(2, 1, 2));
	solution_1.add_Move(0, sMultirobotSolution::Move(4, 7, 8));
	solution_1.add_Move(0, sMultirobotSolution::Move(13, 16, 15));
	solution_1.add_Move(0, sMultirobotSolution::Move(15, 22, 21));
	solution_1.add_Move(1, sMultirobotSolution::Move(2, 2, 3));
	solution_1.add_Move(1, sMultirobotSolution::Move(4, 8, 9));
	solution_1.add_Move(1, sMultirobotSolution::Move(13, 15, 14));
	solution_1.add_Move(1, sMultirobotSolution::Move(15, 21, 20));
	solution_1.add_Move(2, sMultirobotSolution::Move(1, 0, 1));
	solution_1.add_Move(2, sMultirobotSolution::Move(3, 6, 7));
	solution_1.add_Move(3, sMultirobotSolution::Move(4, 9, 15));
	solution_1.add_Move(3, sMultirobotSolution::Move(13, 14, 8));
	solution_1.add_Move(3, sMultirobotSolution::Move(11, 10, 9));
	solution_1.add_Move(4, sMultirobotSolution::Move(1, 1, 2));
	solution_1.add_Move(4, sMultirobotSolution::Move(15, 20, 14));
	solution_1.add_Move(5, sMultirobotSolution::Move(8, 19, 20));
	solution_1.add_Move(5, sMultirobotSolution::Move(16, 23, 22));
	solution_1.add_Move(5, sMultirobotSolution::Move(14, 17, 16));
	solution_1.add_Move(6, sMultirobotSolution::Move(3, 7, 1));
	solution_1.add_Move(6, sMultirobotSolution::Move(12, 11, 10));
	solution_1.add_Move(7, sMultirobotSolution::Move(13, 8, 7));
	solution_1.add_Move(8, sMultirobotSolution::Move(7, 18, 19));
	solution_1.add_Move(9, sMultirobotSolution::Move(13, 7, 6));
	solution_1.add_Move(10, sMultirobotSolution::Move(16, 22, 21));
	solution_1.add_Move(10, sMultirobotSolution::Move(11, 9, 8));
	solution_1.add_Move(11, sMultirobotSolution::Move(12, 10, 9));
	solution_1.add_Move(12, sMultirobotSolution::Move(14, 16, 10));
	solution_1.add_Move(13, sMultirobotSolution::Move(4, 15, 16));
	solution_1.add_Move(14, sMultirobotSolution::Move(13, 6, 0));
	solution_1.add_Move(15, sMultirobotSolution::Move(11, 8, 7));
	solution_1.add_Move(16, sMultirobotSolution::Move(4, 16, 17));
	solution_1.add_Move(17, sMultirobotSolution::Move(15, 14, 8));
	solution_1.add_Move(18, sMultirobotSolution::Move(6, 13, 14));
	solution_1.add_Move(18, sMultirobotSolution::Move(11, 7, 6));
	solution_1.add_Move(18, sMultirobotSolution::Move(4, 17, 23));
	solution_1.add_Move(19, sMultirobotSolution::Move(6, 14, 15));
	solution_1.add_Move(19, sMultirobotSolution::Move(15, 8, 7));
	solution_1.add_Move(19, sMultirobotSolution::Move(5, 12, 13));
	solution_1.add_Move(20, sMultirobotSolution::Move(11, 6, 12));
	solution_1.add_Move(20, sMultirobotSolution::Move(5, 13, 14));
	solution_1.add_Move(20, sMultirobotSolution::Move(1, 2, 8));
	solution_1.add_Move(21, sMultirobotSolution::Move(6, 15, 16));
	solution_1.add_Move(21, sMultirobotSolution::Move(15, 7, 6));
	solution_1.add_Move(22, sMultirobotSolution::Move(6, 16, 17));
	solution_1.add_Move(22, sMultirobotSolution::Move(5, 14, 15));
	solution_1.add_Move(22, sMultirobotSolution::Move(3, 1, 2));
	solution_1.add_Move(23, sMultirobotSolution::Move(1, 8, 14));
	solution_1.add_Move(23, sMultirobotSolution::Move(5, 15, 16));
	solution_1.add_Move(24, sMultirobotSolution::Move(12, 9, 8));
	solution_1.add_Move(24, sMultirobotSolution::Move(1, 14, 15));
	solution_1.add_Move(25, sMultirobotSolution::Move(12, 8, 7));
	solution_1.add_Move(25, sMultirobotSolution::Move(6, 17, 11));
	solution_1.add_Move(25, sMultirobotSolution::Move(14, 10, 9));
	solution_1.add_Move(25, sMultirobotSolution::Move(8, 20, 14));
	solution_1.add_Move(26, sMultirobotSolution::Move(5, 16, 17));
	solution_1.add_Move(26, sMultirobotSolution::Move(11, 12, 18));
	solution_1.add_Move(26, sMultirobotSolution::Move(7, 19, 20));
	solution_1.add_Move(27, sMultirobotSolution::Move(14, 9, 8));
	solution_1.add_Move(27, sMultirobotSolution::Move(1, 15, 16));
	solution_1.add_Move(27, sMultirobotSolution::Move(15, 6, 12));
	solution_1.add_Move(28, sMultirobotSolution::Move(12, 7, 6));
	solution_1.add_Move(28, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(28, sMultirobotSolution::Move(2, 3, 9));
	solution_1.add_Move(29, sMultirobotSolution::Move(9, 4, 3));
	solution_1.add_Move(29, sMultirobotSolution::Move(2, 9, 10));
	solution_1.add_Move(29, sMultirobotSolution::Move(14, 8, 7));
	solution_1.add_Move(30, sMultirobotSolution::Move(9, 3, 9));
	solution_1.add_Move(30, sMultirobotSolution::Move(10, 5, 4));
	solution_1.add_Move(30, sMultirobotSolution::Move(8, 15, 14));
	solution_1.add_Move(31, sMultirobotSolution::Move(6, 11, 5));
	solution_1.add_Move(31, sMultirobotSolution::Move(9, 9, 8));
	solution_1.add_Move(31, sMultirobotSolution::Move(3, 2, 3));
	solution_1.add_Move(31, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(32, sMultirobotSolution::Move(2, 10, 11));
	solution_1.add_Move(32, sMultirobotSolution::Move(14, 7, 1));
	solution_1.add_Move(32, sMultirobotSolution::Move(1, 16, 22));
	solution_1.add_Move(33, sMultirobotSolution::Move(8, 15, 16));
	solution_1.add_Move(33, sMultirobotSolution::Move(7, 20, 14));
	solution_1.add_Move(33, sMultirobotSolution::Move(9, 8, 7));
	solution_1.add_Move(34, sMultirobotSolution::Move(10, 4, 10));
	solution_1.add_Move(34, sMultirobotSolution::Move(7, 14, 15));
	solution_1.add_Move(34, sMultirobotSolution::Move(16, 21, 20));
	solution_1.add_Move(35, sMultirobotSolution::Move(10, 10, 9));
	solution_1.add_Move(35, sMultirobotSolution::Move(9, 7, 13));
	solution_1.add_Move(36, sMultirobotSolution::Move(16, 20, 19));
	solution_1.add_Move(37, sMultirobotSolution::Move(3, 3, 4));
	solution_1.add_Move(37, sMultirobotSolution::Move(8, 16, 10));
	solution_1.add_Move(38, sMultirobotSolution::Move(10, 9, 8));
	solution_1.add_Move(38, sMultirobotSolution::Move(7, 15, 16));
	solution_1.add_Move(39, sMultirobotSolution::Move(10, 8, 7));

	solution_1.to_Screen();

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 8);
	sMultirobotSolutionCompressor::Arrangements_vector unfolded_Arrangements;

	if (compressor.verify_Unfolding(arrangement_1, solution_1, graph_1))
	{
	    printf("Unfolding verification succeeded.\n");
	}
	else
	{
	    printf("Unfolding verification failed.\n");
	    return;
	}
	compressor.unfold_Solution(arrangement_1, solution_1, unfolded_Arrangements);

	sMultirobotSolutionAnalyzer solution_analyzer;

	solution_analyzer.analyze_Solution(solution_1, arrangement_1, graph_1);
	solution_analyzer.to_Screen();

	int optimal_makespan;
	sMultirobotSolution optimal_solution;

	sUndirectedGraph sparse_graph_1;
	graph_1.build_SpanningTree(0, sparse_graph_1);

	if (sFAILED(compressor.compute_OptimalSolution(unfolded_Arrangements[0], unfolded_Arrangements[8], graph_1, sparse_graph_1, 8, optimal_makespan, optimal_solution)))
	{
	    printf("Optimal solution computation failed.\n");
	    return;
	}
	else
	{
	    printf("Optimal solution computation succeeded.\n");
	}
	printf("Optimal solution makespan = %d\n", optimal_makespan);

	optimal_solution.to_Screen();

	clock_t begin = clock();

	sMultirobotSolution compressed_solution;

	if (sFAILED(compressor.compress_Solution(arrangement_1, solution_1, graph_1, sparse_graph_1, compressed_solution)))
	{
	    printf("Solution compression failed.\n");
	    return;
	}
	else
	{
	    printf("Solution compression succeeded.\n");
	}
	compressed_solution.to_Screen();

	clock_t end = clock();
	printf("Compression time: %.3f\n", (end - begin) / (double)CLOCKS_PER_SEC);

	solution_analyzer.analyze_Solution(compressed_solution, arrangement_1, graph_1);
	solution_analyzer.to_Screen();
	
	printf("Solution compression test 1 ... finished\n");
    }


    void test_solution_compression_2(void)
    {
	printf("Solution compression test 2 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertices(24);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(4, 5);

	graph_1.add_Edge(6, 7);
	graph_1.add_Edge(7, 8);
	graph_1.add_Edge(8, 9);
	graph_1.add_Edge(9, 10);
	graph_1.add_Edge(10, 11);

	graph_1.add_Edge(12, 13);
	graph_1.add_Edge(13, 14);
	graph_1.add_Edge(14, 15);
	graph_1.add_Edge(15, 16);
	graph_1.add_Edge(16, 17);

	graph_1.add_Edge(18, 19);
	graph_1.add_Edge(19, 20);
	graph_1.add_Edge(20, 21);
	graph_1.add_Edge(21, 22);
	graph_1.add_Edge(22, 23);

	graph_1.add_Edge(0, 6);
	graph_1.add_Edge(6, 12);
	graph_1.add_Edge(12, 18);

	graph_1.add_Edge(1, 7);
	graph_1.add_Edge(7, 13);
	graph_1.add_Edge(13, 19);

	graph_1.add_Edge(2, 8);
	graph_1.add_Edge(8, 14);
	graph_1.add_Edge(14, 20);

	graph_1.add_Edge(3, 9);
	graph_1.add_Edge(9, 15);
	graph_1.add_Edge(15, 21);

	graph_1.add_Edge(4, 10);
	graph_1.add_Edge(10, 16);
	graph_1.add_Edge(16, 22);

	graph_1.add_Edge(5, 11);
	graph_1.add_Edge(11, 17);
	graph_1.add_Edge(17, 23);

	sRobotArrangement arrangement_1(24, 16);
	arrangement_1.place_Robot(1, 0);
	arrangement_1.place_Robot(2, 1);
	arrangement_1.place_Robot(3, 6);
	arrangement_1.place_Robot(4, 7);
	arrangement_1.place_Robot(5, 12);
	arrangement_1.place_Robot(6, 13);
	arrangement_1.place_Robot(7, 18);
	arrangement_1.place_Robot(8, 19);

	arrangement_1.place_Robot(9, 4);
	arrangement_1.place_Robot(10, 5);
	arrangement_1.place_Robot(11, 10);
	arrangement_1.place_Robot(12, 11);
	arrangement_1.place_Robot(13, 16);
	arrangement_1.place_Robot(14, 17);
	arrangement_1.place_Robot(15, 22);
	arrangement_1.place_Robot(16, 23);

	sMultirobotSolution solution_1;

	solution_1.add_Move(0, sMultirobotSolution::Move(2, 1, 2));
	solution_1.add_Move(0, sMultirobotSolution::Move(4, 7, 8));
	solution_1.add_Move(0, sMultirobotSolution::Move(13, 16, 15));
	solution_1.add_Move(0, sMultirobotSolution::Move(15, 22, 21));
	solution_1.add_Move(1, sMultirobotSolution::Move(2, 2, 3));
	solution_1.add_Move(1, sMultirobotSolution::Move(4, 8, 9));
	solution_1.add_Move(1, sMultirobotSolution::Move(13, 15, 14));
	solution_1.add_Move(1, sMultirobotSolution::Move(15, 21, 20));
	solution_1.add_Move(2, sMultirobotSolution::Move(1, 0, 1));
	solution_1.add_Move(2, sMultirobotSolution::Move(3, 6, 7));
	solution_1.add_Move(3, sMultirobotSolution::Move(4, 9, 15));
	solution_1.add_Move(3, sMultirobotSolution::Move(13, 14, 8));
	solution_1.add_Move(3, sMultirobotSolution::Move(11, 10, 9));
	solution_1.add_Move(4, sMultirobotSolution::Move(1, 1, 2));
	solution_1.add_Move(4, sMultirobotSolution::Move(15, 20, 14));
	solution_1.add_Move(5, sMultirobotSolution::Move(8, 19, 20));
	solution_1.add_Move(5, sMultirobotSolution::Move(16, 23, 22));
	solution_1.add_Move(5, sMultirobotSolution::Move(14, 17, 16));
	solution_1.add_Move(6, sMultirobotSolution::Move(3, 7, 1));
	solution_1.add_Move(6, sMultirobotSolution::Move(12, 11, 10));
	solution_1.add_Move(7, sMultirobotSolution::Move(13, 8, 7));
	solution_1.add_Move(8, sMultirobotSolution::Move(7, 18, 19));
	solution_1.add_Move(9, sMultirobotSolution::Move(13, 7, 6));
	solution_1.add_Move(10, sMultirobotSolution::Move(16, 22, 21));
	solution_1.add_Move(10, sMultirobotSolution::Move(11, 9, 8));
	solution_1.add_Move(11, sMultirobotSolution::Move(12, 10, 9));
	solution_1.add_Move(12, sMultirobotSolution::Move(14, 16, 10));
	solution_1.add_Move(13, sMultirobotSolution::Move(4, 15, 16));
	solution_1.add_Move(14, sMultirobotSolution::Move(13, 6, 0));
	solution_1.add_Move(15, sMultirobotSolution::Move(11, 8, 7));
	solution_1.add_Move(16, sMultirobotSolution::Move(4, 16, 17));
	solution_1.add_Move(17, sMultirobotSolution::Move(15, 14, 8));
	solution_1.add_Move(18, sMultirobotSolution::Move(6, 13, 14));
	solution_1.add_Move(18, sMultirobotSolution::Move(11, 7, 6));
	solution_1.add_Move(18, sMultirobotSolution::Move(4, 17, 23));
	solution_1.add_Move(19, sMultirobotSolution::Move(6, 14, 15));
	solution_1.add_Move(19, sMultirobotSolution::Move(15, 8, 7));
	solution_1.add_Move(19, sMultirobotSolution::Move(5, 12, 13));
	solution_1.add_Move(20, sMultirobotSolution::Move(11, 6, 12));
	solution_1.add_Move(20, sMultirobotSolution::Move(5, 13, 14));
	solution_1.add_Move(20, sMultirobotSolution::Move(1, 2, 8));
	solution_1.add_Move(21, sMultirobotSolution::Move(6, 15, 16));
	solution_1.add_Move(21, sMultirobotSolution::Move(15, 7, 6));
	solution_1.add_Move(22, sMultirobotSolution::Move(6, 16, 17));
	solution_1.add_Move(22, sMultirobotSolution::Move(5, 14, 15));
	solution_1.add_Move(22, sMultirobotSolution::Move(3, 1, 2));
	solution_1.add_Move(23, sMultirobotSolution::Move(1, 8, 14));
	solution_1.add_Move(23, sMultirobotSolution::Move(5, 15, 16));
	solution_1.add_Move(24, sMultirobotSolution::Move(12, 9, 8));
	solution_1.add_Move(24, sMultirobotSolution::Move(1, 14, 15));
	solution_1.add_Move(25, sMultirobotSolution::Move(12, 8, 7));
	solution_1.add_Move(25, sMultirobotSolution::Move(6, 17, 11));
	solution_1.add_Move(25, sMultirobotSolution::Move(14, 10, 9));
	solution_1.add_Move(25, sMultirobotSolution::Move(8, 20, 14));
	solution_1.add_Move(26, sMultirobotSolution::Move(5, 16, 17));
	solution_1.add_Move(26, sMultirobotSolution::Move(11, 12, 18));
	solution_1.add_Move(26, sMultirobotSolution::Move(7, 19, 20));
	solution_1.add_Move(27, sMultirobotSolution::Move(14, 9, 8));
	solution_1.add_Move(27, sMultirobotSolution::Move(1, 15, 16));
	solution_1.add_Move(27, sMultirobotSolution::Move(15, 6, 12));
	solution_1.add_Move(28, sMultirobotSolution::Move(12, 7, 6));
	solution_1.add_Move(28, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(28, sMultirobotSolution::Move(2, 3, 9));
	solution_1.add_Move(29, sMultirobotSolution::Move(9, 4, 3));
	solution_1.add_Move(29, sMultirobotSolution::Move(2, 9, 10));
	solution_1.add_Move(29, sMultirobotSolution::Move(14, 8, 7));
	solution_1.add_Move(30, sMultirobotSolution::Move(9, 3, 9));
	solution_1.add_Move(30, sMultirobotSolution::Move(10, 5, 4));
	solution_1.add_Move(30, sMultirobotSolution::Move(8, 15, 14));
	solution_1.add_Move(31, sMultirobotSolution::Move(6, 11, 5));
	solution_1.add_Move(31, sMultirobotSolution::Move(9, 9, 8));
	solution_1.add_Move(31, sMultirobotSolution::Move(3, 2, 3));
	solution_1.add_Move(31, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(32, sMultirobotSolution::Move(2, 10, 11));
	solution_1.add_Move(32, sMultirobotSolution::Move(14, 7, 1));
	solution_1.add_Move(32, sMultirobotSolution::Move(1, 16, 22));
	solution_1.add_Move(33, sMultirobotSolution::Move(8, 15, 16));
	solution_1.add_Move(33, sMultirobotSolution::Move(7, 20, 14));
	solution_1.add_Move(33, sMultirobotSolution::Move(9, 8, 7));
	solution_1.add_Move(34, sMultirobotSolution::Move(10, 4, 10));
	solution_1.add_Move(34, sMultirobotSolution::Move(7, 14, 15));
	solution_1.add_Move(34, sMultirobotSolution::Move(16, 21, 20));
	solution_1.add_Move(35, sMultirobotSolution::Move(10, 10, 9));
	solution_1.add_Move(35, sMultirobotSolution::Move(9, 7, 13));
	solution_1.add_Move(36, sMultirobotSolution::Move(16, 20, 19));
	solution_1.add_Move(37, sMultirobotSolution::Move(3, 3, 4));
	solution_1.add_Move(37, sMultirobotSolution::Move(8, 16, 10));
	solution_1.add_Move(38, sMultirobotSolution::Move(10, 9, 8));
	solution_1.add_Move(38, sMultirobotSolution::Move(7, 15, 16));
	solution_1.add_Move(39, sMultirobotSolution::Move(10, 8, 7));

	solution_1.to_Screen();

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 6);
	sMultirobotSolutionCompressor::Arrangements_vector unfolded_Arrangements;

	clock_t begin = clock();

	sMultirobotSolution compressed_solution_stage_1;

	printf("*** Stage 1 ***\n");

	sUndirectedGraph sparse_graph_1;
	graph_1.build_SpanningTree(0, sparse_graph_1);

	if (sFAILED(compressor.compress_Solution(arrangement_1, solution_1, graph_1, sparse_graph_1, compressed_solution_stage_1)))
	{
	    printf("Solution compression failed.\n");
	    return;
	}
	else
	{
	    printf("Solution compression succeeded.\n");
	}

	sMultirobotSolution compressed_solution_stage_2;

	printf("*** Stage 2 ***\n");

	if (sFAILED(compressor.compress_Solution(arrangement_1, compressed_solution_stage_1, graph_1, sparse_graph_1, compressed_solution_stage_2)))
	{
	    printf("Solution compression failed.\n");
	    return;
	}
	else
	{
	    printf("Solution compression succeeded.\n");
	}

	sMultirobotSolution compressed_solution_stage_3;

	printf("*** Stage 3 ***\n");

	if (sFAILED(compressor.compress_Solution(arrangement_1, compressed_solution_stage_2, graph_1, sparse_graph_1, compressed_solution_stage_3)))
	{
	    printf("Solution compression failed.\n");
	    return;
	}
	else
	{
	    printf("Solution compression succeeded.\n");
	}

	sMultirobotSolution compressed_solution_stage_4;

	printf("*** Stage 4 ***\n");

	if (sFAILED(compressor.compress_Solution(arrangement_1, compressed_solution_stage_3, graph_1, sparse_graph_1, compressed_solution_stage_4)))
	{
	    printf("Solution compression failed.\n");
	    return;
	}
	else
	{
	    printf("Solution compression succeeded.\n");
	}


	printf("Stage 1 solution:\n");
	compressed_solution_stage_1.to_Screen();
	printf("Stage 2 solution:\n");
	compressed_solution_stage_2.to_Screen();
	printf("Stage 3 solution:\n");
	compressed_solution_stage_3.to_Screen();
	printf("Stage 4 solution:\n");
	compressed_solution_stage_4.to_Screen();

	clock_t end = clock();
	printf("Compression time: %.3f\n", (end - begin) / (double)CLOCKS_PER_SEC);
	
	printf("Solution compression test 2 ... finished\n");
    }


    void test_solution_deflation_1(void)
    {
	printf("Solution deflation test 1 ...\n");

	sUndirectedGraph graph_1;

	graph_1.add_Vertices(24);

	graph_1.add_Edge(0, 1);
	graph_1.add_Edge(1, 2);
	graph_1.add_Edge(2, 3);
	graph_1.add_Edge(3, 4);
	graph_1.add_Edge(4, 5);

	graph_1.add_Edge(6, 7);
	graph_1.add_Edge(7, 8);
	graph_1.add_Edge(8, 9);
	graph_1.add_Edge(9, 10);
	graph_1.add_Edge(10, 11);

	graph_1.add_Edge(12, 13);
	graph_1.add_Edge(13, 14);
	graph_1.add_Edge(14, 15);
	graph_1.add_Edge(15, 16);
	graph_1.add_Edge(16, 17);

	graph_1.add_Edge(18, 19);
	graph_1.add_Edge(19, 20);
	graph_1.add_Edge(20, 21);
	graph_1.add_Edge(21, 22);
	graph_1.add_Edge(22, 23);

	graph_1.add_Edge(0, 6);
	graph_1.add_Edge(6, 12);
	graph_1.add_Edge(12, 18);

	graph_1.add_Edge(1, 7);
	graph_1.add_Edge(7, 13);
	graph_1.add_Edge(13, 19);

	graph_1.add_Edge(2, 8);
	graph_1.add_Edge(8, 14);
	graph_1.add_Edge(14, 20);

	graph_1.add_Edge(3, 9);
	graph_1.add_Edge(9, 15);
	graph_1.add_Edge(15, 21);

	graph_1.add_Edge(4, 10);
	graph_1.add_Edge(10, 16);
	graph_1.add_Edge(16, 22);

	graph_1.add_Edge(5, 11);
	graph_1.add_Edge(11, 17);
	graph_1.add_Edge(17, 23);

	sRobotArrangement arrangement_1(24, 16);
	arrangement_1.place_Robot(1, 0);
	arrangement_1.place_Robot(2, 1);
	arrangement_1.place_Robot(3, 6);
	arrangement_1.place_Robot(4, 7);
	arrangement_1.place_Robot(5, 12);
	arrangement_1.place_Robot(6, 13);
	arrangement_1.place_Robot(7, 18);
	arrangement_1.place_Robot(8, 19);

	arrangement_1.place_Robot(9, 4);
	arrangement_1.place_Robot(10, 5);
	arrangement_1.place_Robot(11, 10);
	arrangement_1.place_Robot(12, 11);
	arrangement_1.place_Robot(13, 16);
	arrangement_1.place_Robot(14, 17);
	arrangement_1.place_Robot(15, 22);
	arrangement_1.place_Robot(16, 23);

	sMultirobotSolution solution_1;

	solution_1.add_Move(0, sMultirobotSolution::Move(2, 1, 2));
	solution_1.add_Move(0, sMultirobotSolution::Move(4, 7, 8));
	solution_1.add_Move(0, sMultirobotSolution::Move(13, 16, 15));
	solution_1.add_Move(0, sMultirobotSolution::Move(15, 22, 21));
	solution_1.add_Move(1, sMultirobotSolution::Move(2, 2, 3));
	solution_1.add_Move(1, sMultirobotSolution::Move(4, 8, 9));
	solution_1.add_Move(1, sMultirobotSolution::Move(13, 15, 14));
	solution_1.add_Move(1, sMultirobotSolution::Move(15, 21, 20));
	solution_1.add_Move(2, sMultirobotSolution::Move(1, 0, 1));
	solution_1.add_Move(2, sMultirobotSolution::Move(3, 6, 7));
	solution_1.add_Move(3, sMultirobotSolution::Move(4, 9, 15));
	solution_1.add_Move(3, sMultirobotSolution::Move(13, 14, 8));
	solution_1.add_Move(3, sMultirobotSolution::Move(11, 10, 9));
	solution_1.add_Move(4, sMultirobotSolution::Move(1, 1, 2));
	solution_1.add_Move(4, sMultirobotSolution::Move(15, 20, 14));
	solution_1.add_Move(5, sMultirobotSolution::Move(8, 19, 20));
	solution_1.add_Move(5, sMultirobotSolution::Move(16, 23, 22));
	solution_1.add_Move(5, sMultirobotSolution::Move(14, 17, 16));
	solution_1.add_Move(6, sMultirobotSolution::Move(3, 7, 1));
	solution_1.add_Move(6, sMultirobotSolution::Move(12, 11, 10));
	solution_1.add_Move(7, sMultirobotSolution::Move(13, 8, 7));
	solution_1.add_Move(8, sMultirobotSolution::Move(7, 18, 19));
	solution_1.add_Move(9, sMultirobotSolution::Move(13, 7, 6));
	solution_1.add_Move(10, sMultirobotSolution::Move(16, 22, 21));
	solution_1.add_Move(10, sMultirobotSolution::Move(11, 9, 8));
	solution_1.add_Move(11, sMultirobotSolution::Move(12, 10, 9));
	solution_1.add_Move(12, sMultirobotSolution::Move(14, 16, 10));
	solution_1.add_Move(13, sMultirobotSolution::Move(4, 15, 16));
	solution_1.add_Move(14, sMultirobotSolution::Move(13, 6, 0));
	solution_1.add_Move(15, sMultirobotSolution::Move(11, 8, 7));
	solution_1.add_Move(16, sMultirobotSolution::Move(4, 16, 17));
	solution_1.add_Move(17, sMultirobotSolution::Move(15, 14, 8));
	solution_1.add_Move(18, sMultirobotSolution::Move(6, 13, 14));
	solution_1.add_Move(18, sMultirobotSolution::Move(11, 7, 6));
	solution_1.add_Move(18, sMultirobotSolution::Move(4, 17, 23));
	solution_1.add_Move(19, sMultirobotSolution::Move(6, 14, 15));
	solution_1.add_Move(19, sMultirobotSolution::Move(15, 8, 7));
	solution_1.add_Move(19, sMultirobotSolution::Move(5, 12, 13));
	solution_1.add_Move(20, sMultirobotSolution::Move(11, 6, 12));
	solution_1.add_Move(20, sMultirobotSolution::Move(5, 13, 14));
	solution_1.add_Move(20, sMultirobotSolution::Move(1, 2, 8));
	solution_1.add_Move(21, sMultirobotSolution::Move(6, 15, 16));
	solution_1.add_Move(21, sMultirobotSolution::Move(15, 7, 6));
	solution_1.add_Move(22, sMultirobotSolution::Move(6, 16, 17));
	solution_1.add_Move(22, sMultirobotSolution::Move(5, 14, 15));
	solution_1.add_Move(22, sMultirobotSolution::Move(3, 1, 2));
	solution_1.add_Move(23, sMultirobotSolution::Move(1, 8, 14));
	solution_1.add_Move(23, sMultirobotSolution::Move(5, 15, 16));
	solution_1.add_Move(24, sMultirobotSolution::Move(12, 9, 8));
	solution_1.add_Move(24, sMultirobotSolution::Move(1, 14, 15));
	solution_1.add_Move(25, sMultirobotSolution::Move(12, 8, 7));
	solution_1.add_Move(25, sMultirobotSolution::Move(6, 17, 11));
	solution_1.add_Move(25, sMultirobotSolution::Move(14, 10, 9));
	solution_1.add_Move(25, sMultirobotSolution::Move(8, 20, 14));
	solution_1.add_Move(26, sMultirobotSolution::Move(5, 16, 17));
	solution_1.add_Move(26, sMultirobotSolution::Move(11, 12, 18));
	solution_1.add_Move(26, sMultirobotSolution::Move(7, 19, 20));
	solution_1.add_Move(27, sMultirobotSolution::Move(14, 9, 8));
	solution_1.add_Move(27, sMultirobotSolution::Move(1, 15, 16));
	solution_1.add_Move(27, sMultirobotSolution::Move(15, 6, 12));
	solution_1.add_Move(28, sMultirobotSolution::Move(12, 7, 6));
	solution_1.add_Move(28, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(28, sMultirobotSolution::Move(2, 3, 9));
	solution_1.add_Move(29, sMultirobotSolution::Move(9, 4, 3));
	solution_1.add_Move(29, sMultirobotSolution::Move(2, 9, 10));
	solution_1.add_Move(29, sMultirobotSolution::Move(14, 8, 7));
	solution_1.add_Move(30, sMultirobotSolution::Move(9, 3, 9));
	solution_1.add_Move(30, sMultirobotSolution::Move(10, 5, 4));
	solution_1.add_Move(30, sMultirobotSolution::Move(8, 15, 14));
	solution_1.add_Move(31, sMultirobotSolution::Move(6, 11, 5));
	solution_1.add_Move(31, sMultirobotSolution::Move(9, 9, 8));
	solution_1.add_Move(31, sMultirobotSolution::Move(3, 2, 3));
	solution_1.add_Move(31, sMultirobotSolution::Move(8, 14, 15));
	solution_1.add_Move(32, sMultirobotSolution::Move(2, 10, 11));
	solution_1.add_Move(32, sMultirobotSolution::Move(14, 7, 1));
	solution_1.add_Move(32, sMultirobotSolution::Move(1, 16, 22));
	solution_1.add_Move(33, sMultirobotSolution::Move(8, 15, 16));
	solution_1.add_Move(33, sMultirobotSolution::Move(7, 20, 14));
	solution_1.add_Move(33, sMultirobotSolution::Move(9, 8, 7));
	solution_1.add_Move(34, sMultirobotSolution::Move(10, 4, 10));
	solution_1.add_Move(34, sMultirobotSolution::Move(7, 14, 15));
	solution_1.add_Move(34, sMultirobotSolution::Move(16, 21, 20));
	solution_1.add_Move(35, sMultirobotSolution::Move(10, 10, 9));
	solution_1.add_Move(35, sMultirobotSolution::Move(9, 7, 13));
	solution_1.add_Move(36, sMultirobotSolution::Move(16, 20, 19));
	solution_1.add_Move(37, sMultirobotSolution::Move(3, 3, 4));
	solution_1.add_Move(37, sMultirobotSolution::Move(8, 16, 10));
	solution_1.add_Move(38, sMultirobotSolution::Move(10, 9, 8));
	solution_1.add_Move(38, sMultirobotSolution::Move(7, 15, 16));
	solution_1.add_Move(39, sMultirobotSolution::Move(10, 8, 7));

	solution_1.to_Screen();

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 8);
	sMultirobotSolutionCompressor::Arrangements_vector unfolded_Arrangements;

	struct tms begin_tms, end_tms;
	times(&begin_tms);

	sMultirobotSolution deflated_solution;

	sUndirectedGraph sparse_graph_1;
	graph_1.build_SpanningTree(0, sparse_graph_1);

	if (sFAILED(compressor.deflate_Solution(arrangement_1, solution_1, graph_1, sparse_graph_1, deflated_solution)))
	{
	    printf("Solution deflation failed.\n");
	    return;
	}
	else
	{
	    printf("Solution deflation succeeded.\n");
	}

	deflated_solution.to_Screen();

	times(&end_tms);

	printf("Deflation time:\n\t%.3f\n\t%.3f\n\t%.3f\n\t%.3f\n",
	       (end_tms.tms_utime - begin_tms.tms_utime) / (double)sysconf(_SC_CLK_TCK),
	       (end_tms.tms_stime - begin_tms.tms_stime) / (double)sysconf(_SC_CLK_TCK),
	       (end_tms.tms_cutime - begin_tms.tms_cutime) / (double)sysconf(_SC_CLK_TCK),
	       (end_tms.tms_cstime - begin_tms.tms_cstime) / (double)sysconf(_SC_CLK_TCK));
	
	printf("Solution deflation test 1 ... finished\n");
    }


    void test_bibox_compression_1(const sString &filename)
    {
	printf("BIBOX compression test 1 ...\n");
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

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 4, 8);
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

	sMultirobotSolution deflated_solution;

	sUndirectedGraph sparse_graph_3;
	graph_3.build_SpanningTree(0, sparse_graph_3);

	if (sFAILED(compressor.deflate_Solution(arrangement, solution, graph_3, sparse_graph_3, deflated_solution)))
	{
	    printf("Solution deflation failed.\n");
	    return;
	}
	else
	{
	    printf("Solution deflation succeeded.\n");
	}

	solution.to_Screen();
	deflated_solution.to_Screen();

	s_GlobalPhaseStatistics.to_Screen();

	printf("BIBOX compression test 1 ... finished\n");
    }


    void test_bibox_compression_2(const sString &filename)
    {
	printf("BIBOX compression test 2 ...\n");
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

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static", 8/*sMultirobotSolutionCompressor::MINISAT_TIMEOUT_UNDEFINED*/, 6);
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

	sMultirobotSolution optimized_solution;

	sUndirectedGraph sparse_graph_3;
	graph_3.build_SpanningTree(0, sparse_graph_3);

	if (sFAILED(compressor.optimize_Solution(arrangement, solution, graph_3, sparse_graph_3, optimized_solution)))
	{
	    printf("Solution optimization failed.\n");
	    return;
	}
	else
	{
	    printf("Solution optimization succeeded.\n");
	}

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

	sMultirobotSolution optimized_solution_2;
	if (sFAILED(compressor.optimize_Solution(arrangement, optimized_solution, graph_3, sparse_graph_3, optimized_solution_2)))
	{
	    printf("Solution optimization failed.\n");
	    return;
	}
	else
	{
	    printf("Solution optimization succeeded.\n");
	}

	sMultirobotSolution optimized_solution_3;
	if (sFAILED(compressor.optimize_Solution(arrangement, optimized_solution_2, graph_3, sparse_graph_3, optimized_solution_3)))
	{
	    printf("Solution optimization failed.\n");
	    return;
	}
	else
	{
	    printf("Solution optimization succeeded.\n");
	}

	solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;

	solution_analyzer.analyze_Solution(solution, arrangement, graph_3);
	solution_analyzer.to_Screen();

	optimized_solution.to_Screen();

	solution_analyzer.analyze_Solution(optimized_solution, arrangement, graph_3);
	solution_analyzer.to_Screen();

	optimized_solution_2.to_Screen();

	solution_analyzer.analyze_Solution(optimized_solution_2, arrangement, graph_3);
	solution_analyzer.to_Screen();

	optimized_solution_3.to_Screen();

	solution_analyzer.analyze_Solution(optimized_solution_3, arrangement, graph_3);
	solution_analyzer.to_Screen();

	s_GlobalPhaseStatistics.to_Screen();

	printf("BIBOX compression test 2 ... finished\n");
    }


    void test_parallel_1(const sString &filename)
    {
	printf("Parallel test 1\n");

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

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 8/*sMultirobotSolutionCompressor::MINISAT_TIMEOUT_UNDEFINED*/,
						 6,
						 3);

	sMultirobotSolution shortened_solution;

	sUndirectedGraph sparse_graph_3;
	graph_3.build_SpanningTree(0, sparse_graph_3);

	if (sFAILED(compressor.shorten_Solution_mt(arrangement, solution, graph_3, sparse_graph_3, shortened_solution)))
	{
	    printf("Solution shortening failed.\n");
	    return;
	}
	else
	{
	    printf("Solution shortening succeeded.\n");
	}

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(shortened_solution, arrangement, graph_3);
	solution_analyzer.to_Screen();
	s_GlobalPhaseStatistics.to_Screen();

	printf("Parallel test 1 ... finished\n");
    }


    void test_parallel_2(const sString &filename)
    {
	printf("Parallel test 1\n");

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

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 8/*sMultirobotSolutionCompressor::MINISAT_TIMEOUT_UNDEFINED*/,
						 6,
						 4);

	sMultirobotSolution optimized_solution;

	sUndirectedGraph sparse_graph_3;
	graph_3.build_SpanningTree(0, sparse_graph_3);

	if (sFAILED(compressor.optimize_Solution_mt(arrangement, solution, graph_3, sparse_graph_3, optimized_solution)))
	{
	    printf("Solution optimization failed.\n");
	    return;
	}
	else
	{
	    printf("Solution optimization succeeded.\n");
	}
	optimized_solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(optimized_solution, arrangement, graph_3);
	solution_analyzer.to_Screen();
	s_GlobalPhaseStatistics.to_Screen();

	printf("Parallel test 2 ... finished\n");
    }


    void test_parallel_3(const sString &filename)
    {
	printf("Parallel test 3\n");

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

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 sMultirobotSolutionCompressor::MINISAT_TIMEOUT_UNDEFINED,
						 10,
						 1);

	sMultirobotSolution optimized_solution;

	sUndirectedGraph sparse_graph_3;
	graph_3.build_SpanningTree(0, sparse_graph_3);

	if (sFAILED(compressor.primeOptimize_Solution_mt(arrangement, solution, graph_3, sparse_graph_3, optimized_solution)))
	{
	    printf("Solution optimization failed.\n");
	    return;
	}
	else
	{
	    printf("Solution optimization succeeded.\n");
	}
	optimized_solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(optimized_solution, arrangement, graph_3);
	solution_analyzer.to_Screen();
	s_GlobalPhaseStatistics.to_Screen();

	printf("Parallel test 3 ... finished\n");
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    //test_solution_compression_1();
    //test_solution_compression_2();
    //    test_solution_deflation_1();
    //    test_bibox_compression_1("../../bibox/grid_04.txt");
    //    test_bibox_compression_2("../../bibox/grid_02.txt");
    //    test_parallel_1("../../bibox/grid_02.txt");
    //    test_parallel_2("../../bibox/grid_02.txt");
    test_parallel_3("../../bibox/16x16/random/bibox_grid_16x16#16_redundant.txt");
}

