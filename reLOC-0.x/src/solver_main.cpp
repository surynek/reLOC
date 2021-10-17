/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* solver_main.cpp / 0.21-robik_056                                           */
/*----------------------------------------------------------------------------*/
//
// Solution generator - main program.
//
// Solves a given multirobot instance by the SAT solving technique from scratch.
// SATPlan iterative solution length increasing is adopted.
//
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/times.h>
#include <unistd.h>

#include "config.h"
#include "compile.h"
#include "complete.h"
#include "defs.h"
#include "reloc.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"
#include "version.h"
#include "search.h"

#include "solver_main.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

  sCommandParameters::sCommandParameters()
      : m_base_strategy(sCommandParameters::STRATEGY_LINEAR_UP)
      , m_completion(sCommandParameters::COMPLETION_SIMULTANEOUS)
      , m_cnf_encoding(sMultirobotSolutionCompressor::ENCODING_UNDEFINED)
      , m_makespan_upper_bound(65536)
      , m_makespan_specified(-1)	
      , m_layer_upper_bound(65536)
      , m_total_cost_bound(65536)
      , m_total_fuel_bound(65536)	
      , m_minisat_timeout(-1)
      , m_total_timeout(600)
      , m_independence_detection(false)
      , m_avoidance_detection(false)
      , m_suboptimal_ratio(-1.0)
      , m_robustness(1)
      , m_directed(false)
  {
      // nothing
  }


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void)
    {
	printf("================================================================\n");
	printf("%s : Multirobot Solver\n", sPRODUCT);
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
	printf("solver_reLOC  --input-file=<string>\n");
	printf("             [--lusc-map-input=<string>]\n");
	printf("             [--lusc-robot-input=<string>]\n");	
	printf("             [--bgu-input=<string>]\n");
	printf("             [--dibox-input=<string>]\n");	
	printf("             [--output-file=<string>]\n");
	printf("             [--lusc-output-file=<string>]\n");
	printf("             [--graphrec-file=<string>]\n");	
	printf("             [--total-timeout=<int>]\n");
	printf("             [--minisat-timeout=<int>]\n");
	printf("             [--pddl-domain-file=<string>]\n");
	printf("             [--pddl-problem-file=<string>]\n");
	printf("             [--encoding={inverse|advanced|differential|bijection|Hadvanced|Hdifferential|Hbijection|\n");
	printf("                          bitwise|flow|matching|Hmatching|direct|Hdirect|simplicial|Hsimplicial|\n");
	printf("                          singular|plural|plural2|heighted|mddnomdd|decomposed|independent|\n");
	printf("                          mdd|mdd+|mdd++|mmdd|mmdd+|mmdd++|rmdd|rmmdd|lmdd++|mddf++|mddx++|mddu|mddx\n");
	printf("	                  tmdd|tmmdd|tmdd|temmdd|pmdd|pmmdd\n");	
	printf("                          idmdd|idmdd+|idmdd++|idmmdd|idmmdd+|idmmdd++|\n");
	printf("                          admdd|admdd+|admdd++|admmdd|admmdd+|admmdd++|\n");
	printf("                          mdd*|idmdd*|admdd*|gmdd|ano|gano|\n");
	printf("                          pcmdd}\n");	
	printf("             [--strategy={linear-down|linear-up|binary}]\n");
	printf("             [--completion={simultaneous|unirobot|whca|complete}]\n");       
	printf("             [--makespan-limit=<int>]\n");
	printf("             [--makespan-specified=<int>]\n");	
	printf("             [--layer-limit=<int>]\n");
	printf("             [--cost-limit=<int>]\n");
	printf("             [--fuel-limit=<int>]\n");	
	printf("             [--suboptimal-ratio=<double>]\n");
	printf("             [--directed]\n");
	printf("\n");
	printf("Examples:\n");
	printf("solver_reLOC --input-file=grid_02.txt\n");
	printf("             --output-file=grid_02.out\n");
	printf("\n");
	printf("Defaults: --minisat-timeout=-1 (unlimited)\n");
	printf("          --strategy=linear-up\n");
	printf("          --encoding=inverse\n");	
	printf("          --total-timeout=600\n");
	printf("          --makespan-limit=65536\n");
	printf("          --layer-limit=65536\n");
	printf("          --cost-limit=65536\n");
	printf("          --fuel-limit=65536\n");	
	printf("          --completion=simultaneous\n");
    }


    sResult solve_MultirobotInstance_SAT(const sCommandParameters &command_parameters)
    {
	sResult result;
	sUndirectedGraph environment(command_parameters.m_directed);

	sRobotArrangement initial_arrangement;
	sRobotArrangement goal_arrangement;
	sRobotGoal robot_goal;

	if (!command_parameters.m_input_filename.empty())
	{
	    printf("Reading graph...\n");

	    if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PERMUTATION_CMDD)
	    {
		result  = environment.from_File_capacitated_multirobot(command_parameters.m_input_filename);
		if (sFAILED(result))
		{
		    printf("Error: Reading capacitated graph from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}		
	    }
	    else
	    {
		result  = environment.from_File_multirobot(command_parameters.m_input_filename);
		if (sFAILED(result))
		{
		    printf("Error: Reading graph from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}
	    }

	    printf("Reading graph... FINISHED\n");
	}

	if (!command_parameters.m_input_filename.empty())
	{
	    printf("Reading initial arrangement...\n");

	    if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PERMUTATION_CMDD)
	    {
		result = initial_arrangement.from_File_capacitated_multirobot(command_parameters.m_input_filename, environment);
		if (sFAILED(result))
		{
		    printf("Error: Reading capacitated arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}
	    }
	    else
	    {
		result = initial_arrangement.from_File_multirobot(command_parameters.m_input_filename);
		if (sFAILED(result))
		{
		    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}
	    }
	    
	    printf("Reading initial arrangement... FINISHED.\n");
	}

	if (!command_parameters.m_input_filename.empty())
	{
	    printf("Reading goal arrangement...\n");

	    if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PERMUTATION_CMDD)
	    {
		result = goal_arrangement.from_File_capacitated_multirobot(command_parameters.m_input_filename, environment, 2);
		if (sFAILED(result))
		{
		    printf("Error: Reading capacitated arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
	    }

		result = robot_goal.from_File_capacitated_multirobot(command_parameters.m_input_filename, environment, 2);
		if (sFAILED(result))
		{
		    printf("Error: Reading capacitated arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}		
	    }
	    else
	    {
		result = goal_arrangement.from_File_multirobot(command_parameters.m_input_filename, 2);
		if (sFAILED(result))
		{
		    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
	    }

		result = robot_goal.from_File_multirobot(command_parameters.m_input_filename, 2);
		if (sFAILED(result))
		{
		    printf("Error: Reading arrangement from file %s failed (code = %d).\n", command_parameters.m_input_filename.c_str(), result);
		    return result;
		}		
	    }
	    printf("Reading goal arrangement... FINISHED.\n");		
	}
	/*
	original_solution.execute_Solution(initial_arrangement, goal_arrangement);
	*/

	//	sMultirobotInstance instance(environment, initial_arrangement, goal_arrangement);

//	printf("Goal\n");
//	robot_goal.to_Screen();
//	printf("Arrangement\n");
//	goal_arrangement.to_Screen();

	sMultirobotInstance instance_tmp;

	if (!command_parameters.m_lusc_map_input_filename.empty() && !command_parameters.m_lusc_robot_input_filename.empty())
	{
	    sResult result = instance_tmp.from_File_lusc(command_parameters.m_lusc_map_input_filename, command_parameters.m_lusc_robot_input_filename);
	    if (sFAILED(result))
	    {
		printf("Error: Reading instance from USC(large robots) file %s (%s) failed (code = %d).\n",
		       command_parameters.m_lusc_map_input_filename.c_str(),
		       command_parameters.m_lusc_robot_input_filename.c_str(),
		       result);
		
		return result;
	    }
	    environment = instance_tmp.m_environment;
	    initial_arrangement = instance_tmp.m_initial_arrangement;
	    goal_arrangement = instance_tmp.m_goal_arrangement;
	    robot_goal = instance_tmp.m_goal_specification;
	}	

	if (!command_parameters.m_bgu_input.empty())
	{
	    sResult result = instance_tmp.from_File_bgu(command_parameters.m_bgu_input);
	    if (sFAILED(result))
	    {
		printf("Error: Reading instance from BGU file %s failed (code = %d).\n", command_parameters.m_bgu_input.c_str(), result);
		return result;
	    }
	    environment = instance_tmp.m_environment;
	    initial_arrangement = instance_tmp.m_initial_arrangement;
	    goal_arrangement = instance_tmp.m_goal_arrangement;
	    robot_goal = instance_tmp.m_goal_specification;
	}

	if (!command_parameters.m_dibox_input.empty())
	{
	    instance_tmp.m_environment.m_directed = true;
	    sResult result = instance_tmp.from_File_dibox(command_parameters.m_dibox_input);
	    if (sFAILED(result))
	    {
		printf("Error: Reading instance from diBOX file %s failed (code = %d).\n", command_parameters.m_dibox_input.c_str(), result);
		return result;
	    }
	    environment = instance_tmp.m_environment;
	    initial_arrangement = instance_tmp.m_initial_arrangement;
	    goal_arrangement = instance_tmp.m_goal_arrangement;
	    robot_goal = instance_tmp.m_goal_specification;
	}	
	printf("Building instance...\n");
	sMultirobotInstance instance(environment, initial_arrangement, robot_goal);
	sMultirobotInstance instance_whca(environment, initial_arrangement, goal_arrangement);
	printf("Building instance... FINISHED.\n");

	instance.to_Screen();

	if (!command_parameters.m_pddl_problem_filename.empty())
	{
	    result = instance.to_File_problemPDDL(command_parameters.m_pddl_problem_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL problem file %s (code = %d).\n", command_parameters.m_pddl_problem_filename.c_str(), result);
		return result;
	    }
	}
	if (!command_parameters.m_pddl_domain_filename.empty())
	{
	    result = instance.to_File_domainPDDL(command_parameters.m_pddl_domain_filename);

	    if (sFAILED(result))
	    {
		printf("Error: Failed to write PDDL domain file %s (code = %d).\n", command_parameters.m_pddl_domain_filename.c_str(), result);
		return result;
	    }
	}	

	#ifdef sVERBOSE
	{
/*
	    environment.to_Screen();
	    initial_arrangement.to_Screen();
	    goal_arrangement.to_Screen();
	    robot_goal.to_Screen();
*/
	}
	#endif


	if (!command_parameters.m_output_filename.empty() || !command_parameters.m_lusc_output_filename.empty())
	{
	    int optimal_makespan, optimal_cost, optimal_fuel;
	    sMultirobotSolution optimal_solution;
	    
	    if (command_parameters.m_completion == sCommandParameters::COMPLETION_UNIROBOT)
	    {
		sASSERT(command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL2 || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL)

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);
		
		result = compressor.compute_UnirobotsSolution(initial_arrangement,
							      robot_goal,
							      environment,
							      instance.m_sparse_environment,
							      command_parameters.m_layer_upper_bound,
							      optimal_makespan,
							      optimal_solution);

		printf("Computed UNIROBOT makespan:%d\n", optimal_makespan);
	    }
	    else if (command_parameters.m_completion == sCommandParameters::COMPLETION_WHCA)
	    {
		sMultirobotSolver_WHCAstar solver_WHCAstar;
		sAstarHeuristic_Distance distance_heuristic;

		solver_WHCAstar.setup_Solver(&distance_heuristic, 16);
		solver_WHCAstar.setup_Instance(instance_whca);

		bool answer = solver_WHCAstar.solve_Instance(optimal_solution);

		if (!answer)
		{
		    printf("WHCA solver UNABLE to provide a solution.\n");
		}
	    }
	    else if (command_parameters.m_completion == sCommandParameters::COMPLETION_COMPLETE)
	    {
		sCompleteSolver complete_solver;
		complete_solver.setup_Solver();

		complete_solver.setup_Instance(sMultirobotInstance(environment, initial_arrangement, goal_arrangement));
		if (!complete_solver.solve())
		{
		    printf("Complete solver UNABLE to provide a solution.\n");
		}
		optimal_solution = complete_solver.m_multirobot_solution;
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_SINGULAR)
	    {
		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								command_parameters.m_layer_upper_bound,
								optimal_makespan,
								optimal_solution);

		printf("Computed makespan:%d\n", optimal_makespan);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL)
	    {
		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								command_parameters.m_layer_upper_bound,
								optimal_makespan,
								optimal_solution);

		printf("Computed makespan:%d\n", optimal_makespan);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PLURAL2)
	    {
		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);
		
		result = compressor.compute_OrtoOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								command_parameters.m_layer_upper_bound,
								optimal_makespan,
								optimal_solution);
		printf("Computed makespan:%d\n", optimal_makespan);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_HEIGHTED)
	    {
		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								instance.m_heighted_Environments,
								command_parameters.m_total_cost_bound,
								optimal_cost,
								optimal_solution);

		printf("Computed makespan:%d\n", optimal_makespan);
	    }
	    else if (   command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_UMTEX
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_MUTEX
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_GMDD			
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_WATER_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_RELAXED_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_TOKEN_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_TOKEN_EMPTY_MDD			
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PERMUTATION_MDD			
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus_mutex			
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_LMDD_plus_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_star
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_PERMUTATION_CMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								MDD,
								command_parameters.m_total_cost_bound,
								optimal_cost,
								optimal_solution);

		printf("Computed sum of costs:%d\n", optimal_cost);
		if (optimal_solution.m_optimality_ratio >= 0.0)
		{
		    printf("Cost <= %.3f * optimum.\n", optimal_solution.m_optimality_ratio);
		    printf("Optimality ratio = %.3f\n", optimal_solution.m_optimality_ratio);
		}
		else
		{
		    printf("Cost <= 1.000 * optimum.\n");
		    printf("Optimality ratio = 1.000\n");
		}
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus_fuel)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);

		int fuel_makespan;
		
		result = compressor.compute_FuelOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								MDD,
								command_parameters.m_total_fuel_bound,
								optimal_fuel,
								fuel_makespan,
								optimal_solution);

		printf("Computed total fuel:%d (makespan = %d)\n", optimal_fuel, fuel_makespan);
	    }
	    else if (   command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ID_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ID_WATER_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ID_MDD_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ID_MDD_plus_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_ID_MDD_star)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolutionID(initial_arrangement,
								  robot_goal,
								  environment,
								  instance.m_sparse_environment,
								  MDD,
								  command_parameters.m_total_cost_bound,
								  optimal_cost,
								  optimal_solution);

		printf("Computed sum of costs:%d\n", optimal_cost);
		if (optimal_solution.m_optimality_ratio >= 0.0)
		{
		    printf("Cost <= %.3f * optimum.\n", optimal_solution.m_optimality_ratio);
		    printf("Optimality ratio = %.3f\n", optimal_solution.m_optimality_ratio);
		}
		else
		{
		    printf("Cost <= 1.000 * optimum.\n");
		    printf("Optimality ratio = 1.000\n");
		}
	    }
	    else if (   command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_AD_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_AD_WATER_MDD
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_AD_MDD_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_AD_MDD_plus_plus
		     || command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_AD_MDD_star)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolutionAD(initial_arrangement,
								  robot_goal,
								  environment,
								  instance.m_sparse_environment,
								  MDD,
								  command_parameters.m_total_cost_bound,
								  optimal_cost,
								  optimal_solution);

		printf("Computed sum of costs:%d\n", optimal_cost);
		if (optimal_solution.m_optimality_ratio >= 0.0)
		{
		    printf("Cost <= %.3f * optimum.\n", optimal_solution.m_optimality_ratio);
		    printf("Optimality ratio = %.3f\n", optimal_solution.m_optimality_ratio);
		}
		else
		{
		    printf("Cost <= 1.000 * optimum.\n");
		    printf("Optimality ratio = 1.000\n");
		}
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_RXMDD_BINARY)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolution_binary(initial_arrangement,
								       robot_goal,
								       environment,
								       instance.m_sparse_environment,
								       MDD,
								       command_parameters.m_total_cost_bound,
								       optimal_cost,
								       optimal_solution);
		
		printf("Computed sum of costs:%d\n", optimal_cost);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_BestCostSolution(initial_arrangement,
							     robot_goal,
							     environment,
							     instance.m_sparse_environment,
							     MDD,
							     command_parameters.m_total_cost_bound,
							     optimal_cost,
							     optimal_solution);
		
		printf("Computed sum of costs:%d\n", optimal_cost);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BCMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_BestCostSolution(initial_arrangement,
							     robot_goal,
							     environment,
							     instance.m_sparse_environment,
							     MDD,
							     command_parameters.m_total_cost_bound,
							     optimal_cost,
							     optimal_solution);
		
		printf("Computed sum of costs:%d\n", optimal_cost);
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BNOMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_BestCostSolution(initial_arrangement,
							     robot_goal,
							     environment,
							     instance.m_sparse_environment,
							     MDD,
							     command_parameters.m_total_cost_bound,
							     optimal_cost,
							     optimal_solution);
		
		printf("Computed sum of costs:%d\n", optimal_cost);
		#ifndef sEXPERIMENT
		{
		    printf("Computed solution:\n");
				
		    optimal_solution.to_Screen();
		    optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
		}
		#endif
		
		if (!command_parameters.m_graphrec_filename.empty())
		{
		    optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
		}		
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_BCNOMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_BestCostSolution(initial_arrangement,
							     robot_goal,
							     environment,
							     instance.m_sparse_environment,
							     MDD,
							     command_parameters.m_total_cost_bound,
							     optimal_cost,
							     optimal_solution);
		
		printf("Computed sum of costs:%d\n", optimal_cost);

		#ifndef sEXPERIMENT
		{
		    printf("Computed solution:\n");
				
		    optimal_solution.to_Screen();
		    optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
		}
		#endif

		if (!command_parameters.m_graphrec_filename.empty())
		{
		    optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
		}		
	    }
	    else if (command_parameters.m_cnf_encoding == sMultirobotSolutionCompressor::ENCODING_NOMDD)
	    {
		sMultirobotInstance::MDD_vector MDD;

		sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
							 command_parameters.m_minisat_timeout,
							 command_parameters.m_total_timeout,
							 command_parameters.m_makespan_upper_bound,
							 sDEFAULT_N_PARALLEL_THREADS,
							 command_parameters.m_cnf_encoding);
		compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
		compressor.set_Robustness(command_parameters.m_robustness);		
		
		result = compressor.compute_CostOptimalSolution(initial_arrangement,
								robot_goal,
								environment,
								instance.m_sparse_environment,
								MDD,
								command_parameters.m_total_cost_bound,
								optimal_cost,
								optimal_solution);

		printf("Computed sum of costs:%d\n", optimal_cost);

                #ifndef sEXPERIMENT
		{
		    printf("Computed solution:\n");
				
		    optimal_solution.to_Screen();
		    optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
		}
		#endif

		if (!command_parameters.m_graphrec_filename.empty())
		{
		    optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
		}		
	    }
	    else
	    {
		if (command_parameters.m_cnf_encoding != sMultirobotSolutionCompressor::ENCODING_UNDEFINED)
		{
		    if (command_parameters.m_cnf_encoding != sMultirobotSolutionCompressor::ENCODING_UNUSED)
		    {
			if (command_parameters.m_makespan_specified > 0)
			{
			    sMultirobotSolution specified_solution;
			    int specified_makespan = command_parameters.m_makespan_specified;

			    sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
								     command_parameters.m_minisat_timeout,
								     command_parameters.m_total_timeout,
								     command_parameters.m_makespan_upper_bound,
								     sDEFAULT_N_PARALLEL_THREADS,
								     command_parameters.m_cnf_encoding);

			    result = compressor.compute_SpecifiedSolution(initial_arrangement,
									  robot_goal,
									  environment,
									  instance.m_sparse_environment,
									  specified_makespan,
									  specified_solution);			    
			    
			    if (sFAILED(result))
			    {
				printf("Error: Solution existence checking failed (code = %d).\n", result);
				return result;
			    }

			    if (specified_makespan != sMultirobotSolutionCompressor::MAKESPAN_UNDEFINED)
			    {
				printf("Computed solution for specified makespan:%d\n", specified_makespan);

                                #ifndef sEXPERIMENT
				{
				    printf("Makespan specified solution:\n");
			    
				    specified_solution.to_Screen();
				    specified_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
				}
			        #endif
				if (!command_parameters.m_graphrec_filename.empty())
				{
				    specified_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
				}

				sMultirobotSolutionAnalyzer solution_analyzer;
				solution_analyzer.analyze_Solution(specified_solution, initial_arrangement, environment);
				
				solution_analyzer.to_Screen();				
			    }
			    else
			    {
				printf("Not solvable under specified makespan.\n");
			    }
			}
			else
			{
			    sMultirobotSolutionCompressor compressor(sRELOC_SAT_SOLVER_PATH,
								     command_parameters.m_minisat_timeout,
								     command_parameters.m_total_timeout,
								     command_parameters.m_makespan_upper_bound,
								     sDEFAULT_N_PARALLEL_THREADS,
								     command_parameters.m_cnf_encoding);
			    compressor.set_Ratio(command_parameters.m_suboptimal_ratio);
			    compressor.set_Robustness(command_parameters.m_robustness);			
			    
			    switch (command_parameters.m_base_strategy)
			    {
			    case sCommandParameters::STRATEGY_LINEAR_DOWN:
			    {
				result = compressor.compute_OptimalSolution(initial_arrangement,
									    robot_goal,
									    environment,
									    instance.m_sparse_environment,
									    command_parameters.m_makespan_upper_bound,
									    optimal_makespan,
									    optimal_solution);
				break;
			    }
			    case sCommandParameters::STRATEGY_LINEAR_UP:
			    {
				if (command_parameters.m_independence_detection)
				{
				    result = compressor.compute_MakespanOptimalSolutionID(initial_arrangement,
											  robot_goal,
											  environment,
											  instance.m_sparse_environment,
											  command_parameters.m_makespan_upper_bound,
											  optimal_makespan,
											  optimal_solution);
				}
				else if (command_parameters.m_avoidance_detection)
				{
				    result = compressor.compute_MakespanOptimalSolutionAD(initial_arrangement,
											  robot_goal,
											  environment,
											  instance.m_sparse_environment,
											  command_parameters.m_makespan_upper_bound,
											  optimal_makespan,
											  optimal_solution);
				}
				else
				{
				    result = compressor.compute_OptimalSolution_(initial_arrangement,
										 robot_goal,
										 environment,
										 instance.m_sparse_environment,
										 command_parameters.m_makespan_upper_bound,
										 optimal_makespan,
										 optimal_solution);
				}
				break;
			    }
			    case sCommandParameters::STRATEGY_BINARY:
			    {
				result = compressor.compute_OptimalSolution(initial_arrangement,
									    robot_goal,
									    environment,
									    instance.m_sparse_environment,
									    2,
									    command_parameters.m_makespan_upper_bound,
									    optimal_makespan,
									    optimal_solution);
				break;
			    }
			    default:
			    {
				sASSERT(false);
			    }
			    }
			    
			    if (sFAILED(result))
			    {
				printf("Error: Solution optimization failed (code = %d).\n", result);
				return result;
			    }
			    printf("Computed optimal makespan:%d\n", optimal_makespan);

                            #ifndef sEXPERIMENT
			    {
				printf("Makespan optimal solution:\n");
			    
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
			    }
			    #endif
			    if (!command_parameters.m_graphrec_filename.empty())
			    {
				optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
			    }
			}
		    }
		    else
		    {
			sMultirobotSolver_DecomposedAstar solver_Astar(command_parameters.m_total_timeout);
			sAstarHeuristic_Distance distance_heuristic;
			
			solver_Astar.setup_Solver(&distance_heuristic);
			solver_Astar.setup_Instance(sMultirobotInstance(environment, initial_arrangement, robot_goal));
			
			switch (solver_Astar.solve_Instance(optimal_solution))
			{
			case sMultirobotSolver_DecomposedAstar::RESULT_SOLVABLE:
			{
			    optimal_makespan = optimal_solution.get_StepCount();
			    
			    printf("Computed optimal makespan:%d\n", optimal_makespan);
			    
                            #ifndef sEXPERIMENT
			    {
				printf("Makespan optimal solution:\n");
				optimal_solution.to_Screen();
				optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
			    }
			    #endif

			    if (!command_parameters.m_graphrec_filename.empty())
			    {
				optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
			    }			    
			    break;
			}
			case sMultirobotSolver_DecomposedAstar::RESULT_UNSOLVABLE:
			{
			    printf("Unable to find solution.\n");
			    break;
			}
			case sMultirobotSolver_DecomposedAstar::RESULT_INDETERMINATE:
			{
			    printf("Cannot decide existence of solution..\n");
			    break;
			}
			default:
			{
			    sASSERT(false);
			}
			}
		    }
		}
		else
		{
		    sMultirobotSolver_DecomposedAstar solver_Astar(command_parameters.m_total_timeout);
		    sAstarHeuristic_Distance distance_heuristic;
		    
		    solver_Astar.setup_Solver(&distance_heuristic);
		    solver_Astar.setup_Instance(sMultirobotInstance(environment, initial_arrangement, robot_goal));
		    
		    switch (solver_Astar.solve_DecomposedInstance(optimal_solution))
		    {
		    case sMultirobotSolver_DecomposedAstar::RESULT_SOLVABLE:
		    {
			optimal_makespan = optimal_solution.get_StepCount();
			
			printf("Computed optimal makespan:%d\n", optimal_makespan);

                        #ifndef sEXPERIMENT
			{
			    printf("Makespan optimal solution:\n");
			    optimal_solution.to_Screen();
			    optimal_solution.to_File_multirobot(command_parameters.m_output_filename.c_str());
			}
			#endif

			if (!command_parameters.m_graphrec_filename.empty())
			{
			    optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
			}			
			break;
		    }
		    case sMultirobotSolver_DecomposedAstar::RESULT_UNSOLVABLE:
		    {
			printf("Unable to find solution.\n");
			break;
		    }
		    case sMultirobotSolver_DecomposedAstar::RESULT_INDETERMINATE:
		    {
			printf("Cannot decide existence of solution..\n");
			break;
		    }
		    default:
		    {
			sASSERT(false);
		    }
		    }
		}
	    }
	    if (command_parameters.m_makespan_specified < 0)
	    {
		printf("Optimal solution:\n");
		if (environment.m_Matrix != NULL)
		{
		    optimal_solution.to_Screen(environment);
		}
		else
		{
		    optimal_solution.to_Screen();
		}
		if (!command_parameters.m_lusc_output_filename.empty() && environment.m_Matrix != NULL)
		{
		    optimal_solution.to_File(environment, command_parameters.m_lusc_output_filename);
		}
	    
		sMultirobotSolutionAnalyzer solution_analyzer;
		solution_analyzer.analyze_Solution(optimal_solution, initial_arrangement, environment);
	    
		solution_analyzer.to_Screen();
		
		if (!command_parameters.m_graphrec_filename.empty())
		{
		    optimal_solution.to_File_graphrec(command_parameters.m_graphrec_filename.c_str(), instance);
		}			    
	    }
	}	    
	s_GlobalPhaseStatistics.to_Screen();

	return sRESULT_SUCCESS;
    }


    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters)
    {
	if (parameter.find("--input-file=") == 0)
	{
	    command_parameters.m_input_filename = parameter.substr(13, parameter.size());
	}
	else if (parameter.find("--bgu-input=") == 0)
	{
	    command_parameters.m_bgu_input = parameter.substr(12, parameter.size());
	}
	else if (parameter.find("--lusc-map-input=") == 0)
	{
	    command_parameters.m_lusc_map_input_filename = parameter.substr(17, parameter.size());
	}
	else if (parameter.find("--lusc-robot-input=") == 0)
	{
	    command_parameters.m_lusc_robot_input_filename = parameter.substr(19, parameter.size());
	}	
	else if (parameter.find("--dibox-input=") == 0)
	{
	    command_parameters.m_dibox_input = parameter.substr(14, parameter.size());
	}	
	else if (parameter.find("--output-file=") == 0)
	{
	    command_parameters.m_output_filename = parameter.substr(14, parameter.size());
	}
	else if (parameter.find("--lusc-output-file=") == 0)
	{
	    command_parameters.m_lusc_output_filename = parameter.substr(19, parameter.size());
	}	
	else if (parameter.find("--graphrec-file=") == 0)
	{
	    command_parameters.m_graphrec_filename = parameter.substr(16, parameter.size());
	}	
	else if (parameter.find("--pddl-domain-file=") == 0)
	{
	    command_parameters.m_pddl_domain_filename = parameter.substr(19, parameter.size());
	}
	else if (parameter.find("--pddl-problem-file=") == 0)
	{
	    command_parameters.m_pddl_problem_filename = parameter.substr(20, parameter.size());
	}
	else if (parameter.find("--makespan-limit=") == 0)
	{
	    command_parameters.m_makespan_upper_bound = sInt_32_from_String(parameter.substr(17, parameter.size()));
	}
	else if (parameter.find("--makespan-specified=") == 0)
	{
	    command_parameters.m_makespan_specified = sInt_32_from_String(parameter.substr(21, parameter.size()));
	}	
	else if (parameter.find("--layer-limit=") == 0)
	{
	    command_parameters.m_layer_upper_bound = sInt_32_from_String(parameter.substr(14, parameter.size()));
	}
	else if (parameter.find("--suboptimal-ratio=") == 0)
	{
	    command_parameters.m_suboptimal_ratio = sDouble_from_String(parameter.substr(19, parameter.size()));
	}
	else if (parameter.find("--robustness=") == 0)
	{
	    command_parameters.m_robustness = sInt_32_from_String(parameter.substr(13, parameter.size()));
	}
	else if (parameter.find("--directed") == 0)
	{
	    command_parameters.m_directed = true;
	}			
	else if (parameter.find("--cost-limit=") == 0)
	{
	    command_parameters.m_total_cost_bound = sInt_32_from_String(parameter.substr(13, parameter.size()));
	}
	else if (parameter.find("--fuel-limit=") == 0)
	{
	    command_parameters.m_total_fuel_bound = sInt_32_from_String(parameter.substr(13, parameter.size()));
	}	
	else if (parameter.find("--encoding=") == 0)
	{
	    sString encoding_str = parameter.substr(11, parameter.size());

	    if (encoding_str == "inverse")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
	    }
	    else if (encoding_str == "idinverse")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_INVERSE;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "advanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
	    }
	    else if (encoding_str == "idadvanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ADVANCED;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "differential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
	    }
	    else if (encoding_str == "iddifferential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIFFERENTIAL;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "bijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BIJECTION;
	    }
	    else if (encoding_str == "idbijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BIJECTION;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hadvanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_ADVANCED;
	    }
	    else if (encoding_str == "idHadvanced")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_ADVANCED;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hdifferential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL;
	    }
	    else if (encoding_str == "idHdifferential")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIFFERENTIAL;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hbijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION;
	    }
	    else if (encoding_str == "idHbijection")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_BIJECTION;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "bitwise")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BITWISE;
	    }
	    else if (encoding_str == "flow")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_FLOW;
	    }
	    else if (encoding_str == "matching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
	    }
	    else if (encoding_str == "idmatching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MATCHING;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hmatching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING;
	    }
	    else if (encoding_str == "idHmatching")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_MATCHING;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "direct")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIRECT;
	    }
	    else if (encoding_str == "iddirect")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_DIRECT;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hdirect")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIRECT;
	    }
	    else if (encoding_str == "idHdirect")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_DIRECT;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "simplicial")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SIMPLICIAL;
	    }
	    else if (encoding_str == "idsimplicial")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SIMPLICIAL;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "Hsimplicial")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_SIMPLICIAL;
	    }
	    else if (encoding_str == "idHsimplicial")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEURISTIC_SIMPLICIAL;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "mmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
	    }
	    else if (encoding_str == "rmmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RELAXED_MMDD;
	    }
	    else if (encoding_str == "tmmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_TOKEN_MMDD;
	    }
	    else if (encoding_str == "temmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_TOKEN_EMPTY_MMDD;
	    }	    
	    else if (encoding_str == "pmmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PERMUTATION_MMDD;
	    }	    	    	    
	    else if (encoding_str == "mmdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
	    }
	    else if (encoding_str == "mmdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
	    }
	    else if (encoding_str == "idmmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "idmmdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "idmmdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
		command_parameters.m_independence_detection = true;
	    }	    else if (encoding_str == "admmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "admmdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "admmdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MMDD_plus_plus;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "singular")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_SINGULAR;
	    }
	    else if (encoding_str == "plural")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PLURAL;
	    }
	    else if (encoding_str == "plural2")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PLURAL2;
	    }
	    else if (encoding_str == "heighted")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_HEIGHTED;
	    }
	    else if (encoding_str == "mdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD;
	    }
	    else if (encoding_str == "mddu")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_UMTEX;
	    }
	    else if (encoding_str == "mddx")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_MUTEX;
	    }	    
	    else if (encoding_str == "gmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_GMDD;
	    }
	    else if (encoding_str == "ano")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ANO;
	    }
	    else if (encoding_str == "gano")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_GANO;
	    }	    	    
	    else if (encoding_str == "wmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_WATER_MDD;
	    }	    
	    else if (encoding_str == "rmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RELAXED_MDD;
	    }
	    else if (encoding_str == "tmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_TOKEN_MDD;
	    }
	    else if (encoding_str == "temdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_TOKEN_EMPTY_MDD;
	    }	    
	    else if (encoding_str == "pmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PERMUTATION_MDD;
	    }	    	    
	    else if (encoding_str == "idmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ID_MDD;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "admdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_AD_MDD;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "idwmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ID_WATER_MDD;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "adwmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_AD_WATER_MDD;
		command_parameters.m_avoidance_detection = true;
	    }	    	    
	    else if (encoding_str == "mdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus;
	    }
	    else if (encoding_str == "mdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus;
	    }
	    else if (encoding_str == "mddx++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus_mutex;
	    }	    
	    else if (encoding_str == "mddf++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_plus_plus_fuel;
	    }	    
	    else if (encoding_str == "lmdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_LMDD_plus_plus;
	    }	    
	    else if (encoding_str == "mdd*")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_MDD_star;
	    }	    	    
	    else if (encoding_str == "idmdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ID_MDD_plus;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "idmdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ID_MDD_plus_plus;
		command_parameters.m_independence_detection = true;
	    }
	    else if (encoding_str == "idmdd*")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_ID_MDD_star;
		command_parameters.m_independence_detection = true;
	    }	    
	    else if (encoding_str == "admdd+")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_AD_MDD_plus;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "admdd++")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_AD_MDD_plus_plus;
		command_parameters.m_avoidance_detection = true;
	    }
	    else if (encoding_str == "admdd*")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_AD_MDD_star;
		command_parameters.m_avoidance_detection = true;
	    }	    	    	    	    	    
	    else if (encoding_str == "rxmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RXMDD;
	    }
	    else if (encoding_str == "bmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BMDD;
	    }
	    else if (encoding_str == "bcmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BCMDD;
	    }
	    else if (encoding_str == "bnomdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BNOMDD;
	    }
	    else if (encoding_str == "bcnomdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_BCNOMDD;
	    }
	    else if (encoding_str == "rxbmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_RXMDD_BINARY;
	    }
	    else if (encoding_str == "nomdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_NOMDD;
	    }
	    else if (encoding_str == "decomposed")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_UNUSED;
	    }
	    else if (encoding_str == "independent")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_UNDEFINED;
	    }
	    else if (encoding_str == "pcmdd")
	    {
		command_parameters.m_cnf_encoding = sMultirobotSolutionCompressor::ENCODING_PERMUTATION_CMDD;
	    }	    
	    else
	    {
		return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--strategy=") == 0)
	{
	    sString encoding_str = parameter.substr(11, parameter.size());

	    if (encoding_str == "linear-down")
	    {
		command_parameters.m_base_strategy = sCommandParameters::STRATEGY_LINEAR_DOWN;
	    }
	    else if (encoding_str == "linear-up")
	    {
		command_parameters.m_base_strategy = sCommandParameters::STRATEGY_LINEAR_UP;
	    }
	    else if (encoding_str == "binary")
	    {
		command_parameters.m_base_strategy = sCommandParameters::STRATEGY_BINARY;
	    }
	    else
	    {
		return sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
	    }
	}
	else if (parameter.find("--completion=") == 0)
	{
	    sString encoding_str = parameter.substr(13, parameter.size());

	    if (encoding_str == "simultaneous")
	    {
		command_parameters.m_completion = sCommandParameters::COMPLETION_SIMULTANEOUS;
	    }
	    else if (encoding_str == "unirobot")
	    {
		command_parameters.m_completion = sCommandParameters::COMPLETION_UNIROBOT;
	    }
	    else if (encoding_str == "whca")
	    {
		command_parameters.m_completion = sCommandParameters::COMPLETION_WHCA;
	    }
	    else if (encoding_str == "complete")
	    {
		command_parameters.m_completion = sCommandParameters::COMPLETION_COMPLETE;
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
	else
	{
	    return sSOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR;
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

    if (argc >= 2)
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
	result = solve_MultirobotInstance_SAT(command_parameters);

	if (sFAILED(result))
	{
	    return result;
	}
    }
    else
    {
	print_Help();
    }
    print_ConcludingMessage();

    return sRESULT_SUCCESS;
}

