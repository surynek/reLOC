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
/* resolver_main.cpp / 0.20-kruh_055                                          */
/*----------------------------------------------------------------------------*/
//
// Solution resolver - main program.
//
// Resolves a given solution of multi-robot path planning problem by a
// specified method.
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
#include "search.h"

#include "resolver_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("================================================================\n");
	printf("%s : Multirobot SAT Solver\n", sPRODUCT);
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
	printf("resolver_reLOC {--solution-file=<string> | --input-file=<string>}\n");
	printf("               [--output-file=<string> --window-size=<int>]\n");
	printf("               [--pddl-domain-file=<string>]\n");
	printf("               [--pddl-problem-file=<string>]\n");
	printf("               [--encoding={inverse|advanced|differential|bijection|Hdifferential|Hbijection|\n");
	printf("                            flow|matching|Hmatching|direct}]\n");
	printf("               [--cnf-file=<string>]\n");
	printf("               [--cnf-level=<int>]\n");
	printf("\n");
	printf("Examples:\n");
	printf("resolver_reLOC --solution-file=grid_02.txt\n");
	printf("               --window-size=16\n");
	printf("               --output-file=grid_02.out\n");
	printf("\n");
	printf("resolver_reLOC --solution-file=grid_02.txt\n");
	printf("               --window-size=16\n");
	printf("               --pddl-domain-file=multirobot.pddl\n");
	printf("               --output-file=grid_02.out\n");
	printf("\n");
	printf("resolver_reLOC --solution-file=grid_02.txt\n");
	printf("               --window-size=16\n");
	printf("               --pddl-problem-file=multirobot_grid_02.pddl\n");
	printf("               --output-file=grid_02.out\n");
	printf("\n");
	printf("resolver_reLOC --solution-file=grid_02.txt\n");
	printf("               --cnf-level=16\n");
	printf("               --cnf-file=multirobot_grid_02.cnf\n");
    }


    sResult resolve_MultirobotSolution_BIBOX(const sCommandParameters &parameters)
    {
	sResult result;
	sUndirectedGraph environment;

	if (!parameters.m_solution_filename.empty())
	{
	    printf("Reading graph from solution ...\n");
	    result = environment.from_File_multirobot(parameters.m_solution_filename);
	    if (sFAILED(result))
	    {
		printf("Error: Reading graph from file %s failed (code = %d).\n", parameters.m_solution_filename.c_str(), result);
		return result;
	    }
	}
	else
	{
	    printf("Reading graph from input ...\n");
	    result = environment.from_File_multirobot(parameters.m_input_filename);
	    if (sFAILED(result))
	    {
		printf("Error: Reading graph from file %s failed (code = %d).\n", parameters.m_input_filename.c_str(), result);
		return result;
	    }
	}

	sRobotArrangement initial_arrangement;
	sRobotGoal robot_goal;

	if (!parameters.m_solution_filename.empty())
	{
	    printf("Reading initial arrangement from solution...\n");
	    result = initial_arrangement.from_File_multirobot(parameters.m_solution_filename);
	    if (sFAILED(result))
	    {
		printf("Error: Reading arrangement from file %s failed (code = %d).\n", parameters.m_solution_filename.c_str(), result);
		return result;
	    }
	}
	else
	{
	    if (!parameters.m_input_filename.empty())
	    {
		printf("Reading initial arrangement from input...\n");	
		result = initial_arrangement.from_File_multirobot(parameters.m_input_filename);
		if (sFAILED(result))
		{
		    printf("Error: Reading arrangement from file %s failed (code = %d).\n", parameters.m_input_filename.c_str(), result);
		    return result;
		}
		printf("Reading goal from input...\n");	
		result = robot_goal.from_File_multirobot(parameters.m_input_filename, 2);
		if (sFAILED(result))
		{
		    printf("Error: Reading goal from file %s failed (code = %d).\n", parameters.m_input_filename.c_str(), result);
		    return result;
		}
	    }	    
	}

	sMultirobotSolution original_solution;

	if (!parameters.m_solution_filename.empty())
	{
	    printf("Reading original solution...\n");
	    result = original_solution.from_File_multirobot(parameters.m_solution_filename);
	    if (sFAILED(result))
	    {
		printf("Error: Reading solution from file %s failed (code = %d).\n", parameters.m_solution_filename.c_str(), result);
		return result;
	    }
	}	
	sRobotArrangement goal_arrangement;

	if (!parameters.m_solution_filename.empty())
	{
	    printf("Reading goal arrangement from solution...\n");

	    result = goal_arrangement.from_File_multirobot(parameters.m_solution_filename, 2);
	    if (sFAILED(result))
	    {
		printf("Error: Reading arrangement from file %s failed (code = %d).\n", parameters.m_solution_filename.c_str(), result);
		return result;
	    }
	}
	/*
	else
	{
	    printf("Reading goal arrangement from input...\n");

	    result = goal_arrangement.from_File_multirobot(parameters.m_input_filename, 2);
	    if (sFAILED(result))
	    {
		printf("Error: Reading arrangement from file %s failed (code = %d).\n", parameters.m_input_filename.c_str(), result);
		return result;
	    }
	}
	*/
	/*
	original_solution.execute_Solution(initial_arrangement, goal_arrangement);
	*/
	// sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);
	sMultirobotInstance instance(environment, initial_arrangement, robot_goal);


	if (!parameters.m_pddl_problem_filename.empty())
	{
	    result = instance.to_File_problemPDDL(parameters.m_pddl_problem_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL problem file %s (code = %d).\n", parameters.m_pddl_problem_filename.c_str(), result);
		return result;
	    }
	}
	if (!parameters.m_pddl_domain_filename.empty())
	{
	    result = instance.to_File_domainPDDL(parameters.m_pddl_domain_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL domain file %s (code = %d).\n", parameters.m_pddl_domain_filename.c_str(), result);
		return result;
	    }
	}

	if (!parameters.m_cnf_filename.empty())
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(parameters.m_cnf_level);
	    switch(parameters.m_cnf_encoding)
	    {
	    case sMultirobotSolutionCompressor::ENCODING_INVERSE:
	    {
		result = instance.to_File_InverseCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_ADVANCED:
	    {
		result = instance.to_File_AdvancedCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL:
	    {
		result = instance.to_File_DifferentialCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_BIJECTION:
	    {
		result = instance.to_File_BijectionCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = instance.to_File_HeuristicDifferentialCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION:
	    {
		result = instance.to_File_HeuristicBijectionCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_PUZZLE:
	    {
		result = instance.to_File_PuzzleCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_FLOW:
	    {
		result = instance.to_File_FlowCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_MATCHING:
	    {
		result = instance.to_File_MatchingCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING:
	    {
		result = instance.to_File_HeuristicMatchingCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    case sMultirobotSolutionCompressor::ENCODING_DIRECT:
	    {
		result = instance.to_File_DirectCNFsat(parameters.m_cnf_filename, encoding_context, "");

		if (sFAILED(result))
		{
		    printf("Error: Failed to write CNF file %s (code = %d).\n", parameters.m_cnf_filename.c_str(), result);
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}

	#ifdef sVERBOSE
	{
	    environment.to_Screen();
	    initial_arrangement.to_Screen();
	    goal_arrangement.to_Screen();
	    robot_goal.to_Screen();
	}
	#endif

	if (!parameters.m_output_filename.empty())
	{
	    sMultirobotSolver_WHCAstar solver_WHCAstar;
	    sAstarHeuristic_Distance distance_heuristic;
	    
	    solver_WHCAstar.setup_Solver(&distance_heuristic, parameters.m_whca_window_size);
	    solver_WHCAstar.setup_Instance(instance);
	    
	    sMultirobotSolution resolved_solution;
	    bool answer = solver_WHCAstar.solve_Instance(resolved_solution);

	    if (!answer)
	    {
		printf("Unable to provide a new solution.\n");
	    }
	    else
	    {
		printf("Original solution:\n");
		original_solution.to_Screen();
		
		printf("reSolved solution:\n");
		resolved_solution.to_Screen();
		
		sMultirobotSolutionAnalyzer solution_analyzer;
		/*
		solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		*/
		solution_analyzer.analyze_Solution(resolved_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		
		FILE *fw;
		
		if ((fw = fopen(parameters.m_output_filename.c_str(), "w")) == NULL)
		{
		    printf("Error: Cannot open output file %s (code = %d).\n", parameters.m_output_filename.c_str(), sRESOLVER_PROGRAM_OUTPUT_OPEN_ERROR);
		    return sRESOLVER_PROGRAM_OUTPUT_OPEN_ERROR;
		}
		initial_arrangement.to_Stream_multirobot(fw);
		environment.to_Stream_multirobot(fw);
		resolved_solution.to_Stream_multirobot(fw);
		fclose(fw);
	    }
	}
	s_GlobalPhaseStatistics.to_Screen();

	return sRESULT_SUCCESS;
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--solution-file=") == 0)
	{
	    command_parameters.m_solution_filename = parameter.substr(16, parameter.size());
	}
	else if (parameter.find("--output-file=") == 0)
	{
	    command_parameters.m_output_filename = parameter.substr(14, parameter.size());
	}
	else if (parameter.find("--input-file=") == 0)
	{
	    command_parameters.m_input_filename = parameter.substr(13, parameter.size());
	}
	else if (parameter.find("--pddl-domain-file=") == 0)
	{
	    command_parameters.m_pddl_domain_filename = parameter.substr(19, parameter.size());
	}
	else if (parameter.find("--pddl-problem-file=") == 0)
	{
	    command_parameters.m_pddl_problem_filename = parameter.substr(20, parameter.size());
	}
	else if (parameter.find("--window-size=") == 0)
	{
	    command_parameters.m_whca_window_size = sInt_32_from_String(parameter.substr(14, parameter.size()));
	    printf("window size:%d\n", command_parameters.m_whca_window_size);
	}
	else if (parameter.find("--cnf-file=") == 0)
	{
	    command_parameters.m_cnf_filename = parameter.substr(11, parameter.size());
	}
	else if (parameter.find("--cnf-level=") == 0)
	{
	    command_parameters.m_cnf_level = sInt_32_from_String(parameter.substr(12, parameter.size()));
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
	    else if (encoding_str == "matching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
	    }
	    else if (encoding_str == "Hmatching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING;
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
	else
	{
	    return sRESOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	}

	command_parameters.m_base_method = sCommandParameters::METHOD_WHCA;

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

    if (argc >= 2 && argc <= 8)
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
	if (command_parameters.m_base_method == sCommandParameters::METHOD_WHCA)
	{
	    result = resolve_MultirobotSolution_BIBOX(command_parameters);
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

