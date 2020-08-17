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
/* autooptim_main.cpp / 0.21-robik_041                                        */
/*----------------------------------------------------------------------------*/
//
// Automatic solution optimizer - main program.
//
// The program automatically optimizes given solution to multi-robot
// path planning instance. Maximum makespan bound that fits into the
// given time limit is used.
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

#include "autooptim_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("================================================================\n");
	printf("%s : Automatic Multirobot Solution Optimizer\n", sPRODUCT);
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
	printf("autooptim_reLOC  --total-timeout=<int>\n");
	printf("                 --number-of-threads=<int>\n");
	printf("                 --solution-file=<string>\n");
	printf("                [--minisat-timeout=<int>]\n");
	printf("                [--base-method={optimization|prime-optimization|deflation}]\n");
	printf("                [--encoding={inverse|advanced|differential|bijection|Hdifferential|Hbijection|flow}]\n");
	printf("\n");
	printf("Example:\n");
	printf("autooptim_reLOC --total-timeout=600\n");
	printf("                --number-of-threads=4\n");
	printf("                --solution-file=grid_02.txt\n");	
	printf("\n");
	printf("Defaults: --minisat-timeout=-1 (unlimited)\n");
	printf("          --base-method=prime-optimization\n");
	printf("          --encoding=inverse\n");	
	printf("          --total-timeout=600\n");
    }


    sResult autoOptimize_MultirobotSolution(const sCommandParameters &command_parameters)
    {	
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, environment);

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

	double start_seconds = sGet_WC_Seconds();
	double finish_seconds = sGet_WC_Seconds();

	int makespan_bound = 3;

	sMultirobotSolution optimized_solution;

	while (true)
	{
	    sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						     command_parameters.m_minisat_timeout,
						     command_parameters.m_total_timeout,
						     makespan_bound,
						     command_parameters.m_N_Threads,
						     command_parameters.m_cnf_encoding);

	    result = compressor.optimize_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	    if (sFAILED(result))
	    {
		printf("Error: Solution optimization failed (code = %d).\n", result);
		return result;
	    }

	    finish_seconds = sGet_WC_Seconds();

	    if (finish_seconds - start_seconds > command_parameters.m_total_timeout || optimized_solution.get_StepCount() < makespan_bound)
	    {
		break;
	    }
	    ++makespan_bound;
	    original_solution = optimized_solution;
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


    sResult autoPrimeOptimize_MultirobotSolution(const sCommandParameters &command_parameters)
    {
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, environment);

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

	double start_seconds = sGet_WC_Seconds();
	double finish_seconds = sGet_WC_Seconds();

	int makespan_bound = 3;

	sMultirobotSolution optimized_solution;

	while (true)
	{
	    sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						     command_parameters.m_minisat_timeout,
						     command_parameters.m_total_timeout,
						     makespan_bound,
						     command_parameters.m_N_Threads,
						     command_parameters.m_cnf_encoding);
	    
	    
	    result = compressor.primeOptimize_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	    if (sFAILED(result))
	    {
		printf("Error: Solution optimization failed (code = %d).\n", result);
		return result;
	    }

	    finish_seconds = sGet_WC_Seconds();

	    if (finish_seconds - start_seconds > command_parameters.m_total_timeout || optimized_solution.get_StepCount() < makespan_bound)
	    {
		break;
	    }
	    ++makespan_bound;
	    original_solution = optimized_solution;
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


    sResult autoDeflate_MultirobotSolution(const sCommandParameters &command_parameters)
    {
	sUndirectedGraph environment;

	sResult result = environment.from_File_multirobot(command_parameters.m_solution_filename);
	if (sFAILED(result))
	{
	    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_solution_filename.c_str(), result);
	    return result;
	}
	sUndirectedGraph sparse_environment;
	environment.build_SpanningTree(0, environment);

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

	double start_seconds = sGet_WC_Seconds();
	double finish_seconds = sGet_WC_Seconds();

	int makespan_bound = 3;

	sMultirobotSolution optimized_solution;

	while (true)
	{
	    sMultirobotSolutionCompressor compressor("../../sat/minisat_static",
						     command_parameters.m_minisat_timeout,
						     command_parameters.m_total_timeout,
						     makespan_bound,
						     command_parameters.m_N_Threads,
						     command_parameters.m_cnf_encoding);
	    
	    result = compressor.deflate_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, optimized_solution);
	    if (sFAILED(result))
	    {
		printf("Error: Solution optimization failed (code = %d).\n", result);
		return result;
	    }

	    finish_seconds = sGet_WC_Seconds();

	    if (finish_seconds - start_seconds > command_parameters.m_total_timeout || optimized_solution.get_StepCount() < makespan_bound)
	    {
		break;
	    }
	    ++makespan_bound;
	    original_solution = optimized_solution;
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
	    else if (encoding_str == "advanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
	    }
	    else if (encoding_str == "differential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
	    }
	    else if (encoding_str == "bijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BIJECTION;
	    }
	    else if (encoding_str == "Hdifferential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL;
	    }
	    else if (encoding_str == "Hbijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION;
	    }
	    else if (encoding_str == "flow")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_FLOW;
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
	else if (parameter.find("--total-timeout=") == 0)
	{
	    command_parameters.m_total_timeout = sInt_32_from_String(parameter.substr(16, parameter.size()));
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

    if (argc >= 4)
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
	    result = autoOptimize_MultirobotSolution(command_parameters);
	    if (sFAILED(result))
	    {
		return result;
	    }
	}
	else if (command_parameters.m_base_method == sCommandParameters::METHOD_PRIME_OPTIMIZATION)
	{
	    result = autoPrimeOptimize_MultirobotSolution(command_parameters);
	    if (sFAILED(result))
	    {
		return result;
	    }
	}
	else if (command_parameters.m_base_method == sCommandParameters::METHOD_DEFLATION)
	{
	    result = autoDeflate_MultirobotSolution(command_parameters);
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

