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
/* optimizer_main.cpp / 0.20-kruh_051                                         */
/*----------------------------------------------------------------------------*/
//
// Solution optimizer - main program.
//
// This optimizes a given solution of multirobot path planning instance.
// SAT-based techniques are used for the solution optimization.
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
#include "compress.h"
#include "statistics.h"
#include "version.h"

#include "optimizer_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("================================================================\n");
	printf("%s : Multirobot Solution Optimizer\n", sPRODUCT);
	printf("%s\n", sCOPYRIGHT);
	printf("----------------------------------------------------------------\n");
    }


    void print_ConcludingMessage(void)
    {
	printf("----------------------------------------------------------------\n");
    }


    void print_Help(void)
    {
	printf("Usage:\n");
	printf("optimizer_reLOC --minisat-timeout=<int>\n");
	printf("                --makespan-bound=<int>\n");
	printf("                --number-of-threads=<int>\n");
	printf("                --base-method=[optimization|prime-optimization|deflation]\n");
	printf("                --encoding=[inverse|differential|flow|matching|direct]\n");
	printf("                --solution-file=<string>\n");
	printf("\n");
	printf("Example:\n");
	printf("optimizer_reLOC --minisat-timeout=4\n");
	printf("                --makespan-bound=6\n");
	printf("                --number-of-threads=4\n");
	printf("                --solution-file=grid_02.txt\n");
	printf("\n");
	printf("Defaults:\n");
	printf("  --base-method=prime-optimization\n");
	printf("  --encoding=inverse\n");
	
    }


    sResult optimize_MultirobotSolution(const sCommandParameters &command_parameters)
    {
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, sparse_environment);

	sRobotArrangement initial_arrangement;
	result = initial_arrangement.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolution original_solution;
	result = original_solution.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading solution from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 command_parameters.m_minisat_timeout,
						 command_parameters.m_makespan_bound,
						 command_parameters.m_N_Threads,
						 command_parameters.m_cnf_encoding);

	sMultirobotSolution optimized_solution;

	result = compressor.optimize_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	if (sFAILED(result))
	{
	    printf("Error: Solution optimization failed (code = %d).\n", result);
	    return result;
	}

	printf("Original solution:\n");
	original_solution.to_Screen();

	printf("Optimized solution:\n");
	optimized_solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();
	solution_analyzer.analyze_Solution(optimized_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();

	s_GlobalPhaseStatistics.to_Screen();

	return sRESULT_SUCCESS;
    }


    sResult prime_optimize_MultirobotSolution(const sCommandParameters &command_parameters)
    {
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, sparse_environment);

	sRobotArrangement initial_arrangement;
	result = initial_arrangement.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolution original_solution;
	result = original_solution.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading solution from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 command_parameters.m_minisat_timeout,
						 command_parameters.m_makespan_bound,
						 command_parameters.m_N_Threads,
						 command_parameters.m_cnf_encoding);

	sMultirobotSolution optimized_solution;

	result = compressor.primeOptimize_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	if (sFAILED(result))
	{
	    printf("Error: Solution optimization failed (code = %d).\n", result);
	    return result;
	}

	printf("Original solution:\n");
	original_solution.to_Screen();

	printf("Optimized solution:\n");
	optimized_solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();
	solution_analyzer.analyze_Solution(optimized_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();

	s_GlobalPhaseStatistics.to_Screen();

	return sRESULT_SUCCESS;
    }


    sResult deflate_MultirobotSolution(const sCommandParameters &command_parameters)
    {
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, sparse_environment);

	sRobotArrangement initial_arrangement;
	result = initial_arrangement.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolution original_solution;
	result = original_solution.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading solution from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}

	sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						 command_parameters.m_minisat_timeout,
						 command_parameters.m_makespan_bound,
						 command_parameters.m_N_Threads,
						 command_parameters.m_cnf_encoding);

	sMultirobotSolution optimized_solution;

	result = compressor.deflate_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	if (sFAILED(result))
	{
	    printf("Error: Solution optimization failed (code = %d).\n", result);
	    return result;
	}

	printf("Original solution:\n");
	original_solution.to_Screen();

	printf("Optimized solution:\n");
	optimized_solution.to_Screen();

	sMultirobotSolutionAnalyzer solution_analyzer;
	solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();
	solution_analyzer.analyze_Solution(optimized_solution, initial_arrangement, environment);
	solution_analyzer.to_Screen();

	s_GlobalPhaseStatistics.to_Screen();

	return sRESULT_SUCCESS;
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	command_parameters.m_base_method = sCommandParameters::METHOD_PRIME_OPTIMIZATION;
	command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
	command_parameters.m_minisat_timeout = -1;

	if (parameter.find("--solution-file=") == 0)
	{
	    command_parameters.m_solution_filename = parameter.substr(16, parameter.size());
	}
	else if (parameter.find("--base-method=") == 0)
	{
	    sString method_str = parameter.substr(14, parameter.size());
	    if (method_str == "optimization")
	    {
		command_parameters.m_base_method = sCommandParameters::METHOD_OPTIMIZATION;
	    }
	    else if (method_str == "prime-optimization")
	    {
		command_parameters.m_base_method = sCommandParameters::METHOD_PRIME_OPTIMIZATION;
	    }
	    else if (method_str == "deflation")
	    {
		command_parameters.m_base_method = sCommandParameters::METHOD_DEFLATION;
	    }
	    else
	    {
		return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--encoding=") == 0)
	{
	    sString encoding_str = parameter.substr(11, parameter.size());
	    if (encoding_str == "inverse")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
	    }
	    else if (encoding_str == "differential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
	    }
	    else if (encoding_str == "advanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
	    }
	    else if (encoding_str == "flow")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_FLOW;
	    }
	    else if (encoding_str == "matching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
	    }
	    else if (encoding_str == "direct")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIRECT;
	    }
	    else
	    {
		return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--minisat-timeout=") == 0)
	{
	    command_parameters.m_minisat_timeout = sInt_32_from_String(parameter.substr(18, parameter.size()));
	}
	else if (parameter.find("--makespan-bound=") == 0)
	{
	    command_parameters.m_makespan_bound = sInt_32_from_String(parameter.substr(17, parameter.size()));
	}
	else if (parameter.find("--number-of-threads=") == 0)
	{
	    command_parameters.m_N_Threads = sInt_32_from_String(parameter.substr(20, parameter.size()));
	}
	else
	{
	    return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	}

	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int argc, char **argv)
{
    sResult result;
    sCommandParameters command_parameters;

    print_IntroductoryMessage();

    if (argc >= 5)
    {
	for (int i = 1; i < argc; ++i)
	{
	    result = parse_CommandLineParameter(argv[i], command_parameters);
	    if (sFAILED(result))
	    {
		printf("Error: Cannot parse command line parameters (code = %d).\n", result);
		print_Help();
		return result;
	    }
	}
	if (command_parameters.m_base_method == sCommandParameters::METHOD_OPTIMIZATION)
	{
	    result = optimize_MultirobotSolution(command_parameters);
	    if (sFAILED(result))
	    {
		return result;
	    }
	}
	else if (command_parameters.m_base_method == sCommandParameters::METHOD_PRIME_OPTIMIZATION)
	{
	    result = prime_optimize_MultirobotSolution(command_parameters);
	    if (sFAILED(result))
	    {
		return result;
	    }
	}
	else if (command_parameters.m_base_method == sCommandParameters::METHOD_DEFLATION)
	{
	    result = deflate_MultirobotSolution(command_parameters);
	    if (sFAILED(result))
	    {
		return result;
	    }
	}
    }
    else
    {
	print_Help();
    }
    print_ConcludingMessage();

    return sRESULT_SUCCESS;
}

