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
/* statistics.cpp / 0.20-kruh_055                                             */
/*----------------------------------------------------------------------------*/
//
// Statistical data collection and analytical tools.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/times.h>

#include "config.h"
#include "compile.h"
#include "statistics.h"
#include "compress.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sMultirobotSolutionAnalyzer

    sMultirobotSolutionAnalyzer::sMultirobotSolutionAnalyzer()
    {
	// nothing
    }


    sMultirobotSolutionAnalyzer::sMultirobotSolutionAnalyzer(const sMultirobotSolutionAnalyzer &solution_analyzer)
	: m_total_makespan(solution_analyzer.m_total_makespan)
	, m_total_distance(solution_analyzer.m_total_distance)
	, m_total_trajectory(solution_analyzer.m_total_trajectory)
	, m_total_cost(solution_analyzer.m_total_cost)
	, m_average_parallelism(solution_analyzer.m_average_parallelism)
	, m_average_distance(solution_analyzer.m_average_distance)
	, m_average_trajectory(solution_analyzer.m_average_trajectory)
	, m_distribution_Parallelisms(solution_analyzer.m_distribution_Parallelisms)
	, m_distribution_Distances(solution_analyzer.m_distribution_Distances)
	, m_distribution_Trajectories(solution_analyzer.m_distribution_Trajectories)
    {
	// nothing
    }
    

    const sMultirobotSolutionAnalyzer& sMultirobotSolutionAnalyzer::operator=(const sMultirobotSolutionAnalyzer &solution_analyzer)
    {
	m_total_makespan = solution_analyzer.m_total_makespan;
	m_total_distance = solution_analyzer.m_total_distance;
	m_total_trajectory = solution_analyzer.m_total_trajectory;
	m_total_cost = solution_analyzer.m_total_cost;

	m_average_parallelism = solution_analyzer.m_average_parallelism;
	m_average_distance = solution_analyzer.m_average_distance;
	m_average_trajectory = solution_analyzer.m_average_trajectory;
	m_distribution_Parallelisms = solution_analyzer.m_distribution_Parallelisms;
	m_distribution_Distances = solution_analyzer.m_distribution_Distances;
	m_distribution_Trajectories = solution_analyzer.m_distribution_Trajectories;
	
	return *this;
    }
    

    void sMultirobotSolutionAnalyzer::analyze_Solution(const sMultirobotSolution &solution,
						       const sRobotArrangement   &initial_arrangement,
						       sUndirectedGraph          &environment)
    {
	m_distribution_Parallelisms.clear();
	m_distribution_Distances.clear();
	m_distribution_Trajectories.clear();

	int total_N_Moves = 0;

	int N_Steps = solution.m_Steps.size();       
	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    int step_size = step.m_Moves.size();

	    Distribution_map::iterator distribution_record = m_distribution_Parallelisms.find(step_size);
	    if (distribution_record != m_distribution_Parallelisms.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Parallelisms[step_size] = 1;
	    }
	    total_N_Moves += step_size;
	}
	m_average_parallelism = (double)total_N_Moves / N_Steps;

	sRobotArrangement final_arrangement;
	solution.execute_Solution(initial_arrangement, final_arrangement);

	#ifdef sVERBOSE
	{
	    printf("Final arrangement:\n");
	    final_arrangement.to_Screen_brief();
	}
	#endif

	sMultirobotInstance instance(environment, initial_arrangement, final_arrangement);

	sUndirectedGraph::VertexIDs_vector source_IDs;
	sUndirectedGraph::VertexIDs_vector goal_IDs;
	instance.collect_Endpoints(source_IDs, goal_IDs);

	environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	const sUndirectedGraph::Distances_2d_vector &source_Distances = environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = environment.get_GoalShortestPaths();

	m_total_distance = 0;

	int N_Robots = initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id < N_Robots; ++robot_id)
	{
	    int distance = source_Distances[initial_arrangement.get_RobotLocation(robot_id)][final_arrangement.get_RobotLocation(robot_id)];
	    Distribution_map::iterator distribution_record = m_distribution_Distances.find(distance);
	    if (distribution_record != m_distribution_Distances.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Distances[distance] = 1;
	    }
	    m_total_distance += distance;
	}
	m_average_distance = (double)m_total_distance / N_Robots;

	Trajectories_map robot_Trajectories;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		move->m_robot_id;

		Trajectories_map::iterator trajectory_record = robot_Trajectories.find(move->m_robot_id);
		if (trajectory_record != robot_Trajectories.end())
		{
		    ++trajectory_record->second;
		}
		else
		{
		    robot_Trajectories[move->m_robot_id] = 1;
		}
	    }
	}

	int total_trajectory = 0;
	for (Trajectories_map::const_iterator trajectory_record = robot_Trajectories.begin(); trajectory_record != robot_Trajectories.end(); ++trajectory_record)
	{
	    Distribution_map::iterator distribution_record = m_distribution_Trajectories.find(trajectory_record->second);
	    if (distribution_record != m_distribution_Trajectories.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Trajectories[trajectory_record->second] = 1;
	    }
	    total_trajectory += trajectory_record->second;
	}
	m_average_trajectory = (double)total_trajectory / N_Robots;

	m_total_makespan = N_Steps;
	m_total_trajectory = total_N_Moves;

	m_total_cost = N_Steps * N_Robots;

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_last_move_step = -1;

	    int st = 0;
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    if (move->m_robot_id == robot_id && move->m_dest_vrtx_id == final_arrangement.get_RobotLocation(robot_id) && move->m_dest_vrtx_id != move->m_src_vrtx_id)
		    {
			robot_last_move_step = st;
		    }
		}
		++st;
	    }
	    int placed_steps = N_Steps - robot_last_move_step - 1;
	    m_total_cost -= placed_steps;
	}
	printf("Total cost = %d\n", m_total_cost);
    }

    
    int sMultirobotSolutionAnalyzer::calc_TotalCost(const sMultirobotSolution &solution,
						    const sRobotArrangement   &initial_arrangement,
						    sUndirectedGraph          &sUNUSED(environment))
    {
	int N_Steps = solution.m_Steps.size();       
/*
	m_distribution_Parallelisms.clear();
	m_distribution_Distances.clear();
	m_distribution_Trajectories.clear();

	int total_N_Moves = 0;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    int step_size = step.m_Moves.size();

	    Distribution_map::iterator distribution_record = m_distribution_Parallelisms.find(step_size);
	    if (distribution_record != m_distribution_Parallelisms.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Parallelisms[step_size] = 1;
	    }
	    total_N_Moves += step_size;
	}
	m_average_parallelism = (double)total_N_Moves / N_Steps;
*/
	sRobotArrangement final_arrangement;
	solution.execute_Solution(initial_arrangement, final_arrangement);
/*
	printf("beta 1\n");
	sMultirobotInstance instance(environment, initial_arrangement, final_arrangement);

po	sUndirectedGraph::VertexIDs_vector source_IDs;
	sUndirectedGraph::VertexIDs_vector goal_IDs;
	instance.collect_Endpoints(source_IDs, goal_IDs);
	printf("beta 2\n");

	environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sUndirectedGraph::Distances_2d_vector &source_Distances = environment.get_SourceShortestPaths();
	sUndirectedGraph::Distances_2d_vector &goal_Distances = environment.get_GoalShortestPaths();
*/
	m_total_distance = 0;

	int N_Robots = initial_arrangement.get_RobotCount();
/*
	for (int robot_id = 1; robot_id < N_Robots; ++robot_id)
	{
	    int distance = all_pairs_Distances[initial_arrangement.get_RobotLocation(robot_id)][final_arrangement.get_RobotLocation(robot_id)];
	    Distribution_map::iterator distribution_record = m_distribution_Distances.find(distance);
	    if (distribution_record != m_distribution_Distances.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Distances[distance] = 1;
	    }
	    m_total_distance += distance;
	}
	m_average_distance = (double)m_total_distance / N_Robots;

	Trajectories_map robot_Trajectories;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		move->m_robot_id;

		Trajectories_map::iterator trajectory_record = robot_Trajectories.find(move->m_robot_id);
		if (trajectory_record != robot_Trajectories.end())
		{
		    ++trajectory_record->second;
		}
		else
		{
		    robot_Trajectories[move->m_robot_id] = 1;
		}
	    }
	}

	int total_trajectory = 0;
	for (Trajectories_map::const_iterator trajectory_record = robot_Trajectories.begin(); trajectory_record != robot_Trajectories.end(); ++trajectory_record)
	{
	    Distribution_map::iterator distribution_record = m_distribution_Trajectories.find(trajectory_record->second);
	    if (distribution_record != m_distribution_Trajectories.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Trajectories[trajectory_record->second] = 1;
	    }
	    total_trajectory += trajectory_record->second;
	}
	m_average_trajectory = (double)total_trajectory / N_Robots;

	m_total_makespan = N_Steps;
	m_total_trajectory = total_N_Moves;
*/
	m_total_cost = N_Steps * N_Robots;


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_last_move_step = -1;

	    int st = 0;
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    if (move->m_robot_id == robot_id && move->m_dest_vrtx_id == final_arrangement.get_RobotLocation(robot_id) && move->m_dest_vrtx_id != move->m_src_vrtx_id)
		    {
			robot_last_move_step = st;
		    }
		}
		++st;
	    }
	    int placed_steps = N_Steps - robot_last_move_step - 1;
	    m_total_cost -= placed_steps;
	}
//	printf("Total cost = %d\n", m_total_cost);
	return m_total_cost;
    }


    int sMultirobotSolutionAnalyzer::calc_TotalFuel(const sMultirobotSolution &solution,
						    const sRobotArrangement   &initial_arrangement,
						    sUndirectedGraph          &sUNUSED(environment))
    {
	int N_Steps = solution.m_Steps.size();       
/*
	m_distribution_Parallelisms.clear();
	m_distribution_Distances.clear();
	m_distribution_Trajectories.clear();

	int total_N_Moves = 0;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    int step_size = step.m_Moves.size();

	    Distribution_map::iterator distribution_record = m_distribution_Parallelisms.find(step_size);
	    if (distribution_record != m_distribution_Parallelisms.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Parallelisms[step_size] = 1;
	    }
	    total_N_Moves += step_size;
	}
	m_average_parallelism = (double)total_N_Moves / N_Steps;
*/
	sRobotArrangement final_arrangement;
	solution.execute_Solution(initial_arrangement, final_arrangement);
/*
	printf("beta 1\n");
	sMultirobotInstance instance(environment, initial_arrangement, final_arrangement);

po	sUndirectedGraph::VertexIDs_vector source_IDs;
	sUndirectedGraph::VertexIDs_vector goal_IDs;
	instance.collect_Endpoints(source_IDs, goal_IDs);
	printf("beta 2\n");

	environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	sUndirectedGraph::Distances_2d_vector &source_Distances = environment.get_SourceShortestPaths();
	sUndirectedGraph::Distances_2d_vector &goal_Distances = environment.get_GoalShortestPaths();
*/
	m_total_distance = 0;

	int N_Robots = initial_arrangement.get_RobotCount();
/*
	for (int robot_id = 1; robot_id < N_Robots; ++robot_id)
	{
	    int distance = all_pairs_Distances[initial_arrangement.get_RobotLocation(robot_id)][final_arrangement.get_RobotLocation(robot_id)];
	    Distribution_map::iterator distribution_record = m_distribution_Distances.find(distance);
	    if (distribution_record != m_distribution_Distances.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Distances[distance] = 1;
	    }
	    m_total_distance += distance;
	}
	m_average_distance = (double)m_total_distance / N_Robots;

	Trajectories_map robot_Trajectories;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		move->m_robot_id;

		Trajectories_map::iterator trajectory_record = robot_Trajectories.find(move->m_robot_id);
		if (trajectory_record != robot_Trajectories.end())
		{
		    ++trajectory_record->second;
		}
		else
		{
		    robot_Trajectories[move->m_robot_id] = 1;
		}
	    }
	}

	int total_trajectory = 0;
	for (Trajectories_map::const_iterator trajectory_record = robot_Trajectories.begin(); trajectory_record != robot_Trajectories.end(); ++trajectory_record)
	{
	    Distribution_map::iterator distribution_record = m_distribution_Trajectories.find(trajectory_record->second);
	    if (distribution_record != m_distribution_Trajectories.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Trajectories[trajectory_record->second] = 1;
	    }
	    total_trajectory += trajectory_record->second;
	}
	m_average_trajectory = (double)total_trajectory / N_Robots;

	m_total_makespan = N_Steps;
	m_total_trajectory = total_N_Moves;
*/
	m_total_fuel = N_Steps * N_Robots;


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_last_move_step = -1;

	    int st = 0;
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    if (move->m_robot_id == robot_id && move->m_dest_vrtx_id == final_arrangement.get_RobotLocation(robot_id) && move->m_dest_vrtx_id != move->m_src_vrtx_id)
		    {
			robot_last_move_step = st;
		    }
		}
		++st;
	    }
	    int placed_steps = N_Steps - robot_last_move_step - 1;
	    m_total_fuel -= placed_steps;
	}
//	printf("Total fuel = %d\n", m_total_fuel);
	return m_total_fuel;
    }    

/*
    int sMultirobotSolutionAnalyzer::calc_TotalCost(const sMultirobotSolution &solution,
						    const sRobotArrangement   &initial_arrangement,
						    sUndirectedGraph          &environment)
    {
	m_distribution_Parallelisms.clear();
	m_distribution_Distances.clear();
	m_distribution_Trajectories.clear();

	int total_N_Moves = 0;

	int N_Steps = solution.m_Steps.size();       
	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    int step_size = step.m_Moves.size();

	    Distribution_map::iterator distribution_record = m_distribution_Parallelisms.find(step_size);
	    if (distribution_record != m_distribution_Parallelisms.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Parallelisms[step_size] = 1;
	    }
	    total_N_Moves += step_size;
	}
	m_average_parallelism = (double)total_N_Moves / N_Steps;

	sRobotArrangement final_arrangement;
	solution.execute_Solution(initial_arrangement, final_arrangement);

	sMultirobotInstance instance(environment, initial_arrangement, final_arrangement);

	sUndirectedGraph::VertexIDs_vector source_IDs;
	sUndirectedGraph::VertexIDs_vector goal_IDs;
	instance.collect_Endpoints(source_IDs, goal_IDs);

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances, source_IDs, goal_IDs);

	m_total_distance = 0;

	int N_Robots = initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id < N_Robots; ++robot_id)
	{
	    int distance = all_pairs_Distances[initial_arrangement.get_RobotLocation(robot_id)][final_arrangement.get_RobotLocation(robot_id)];
	    Distribution_map::iterator distribution_record = m_distribution_Distances.find(distance);
	    if (distribution_record != m_distribution_Distances.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Distances[distance] = 1;
	    }
	    m_total_distance += distance;
	}
	m_average_distance = (double)m_total_distance / N_Robots;

	Trajectories_map robot_Trajectories;

	for (int i = 0; i < N_Steps; ++i)
	{
	    const sMultirobotSolution::Step &step = solution.m_Steps[i];

	    for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		move->m_robot_id;

		Trajectories_map::iterator trajectory_record = robot_Trajectories.find(move->m_robot_id);
		if (trajectory_record != robot_Trajectories.end())
		{
		    ++trajectory_record->second;
		}
		else
		{
		    robot_Trajectories[move->m_robot_id] = 1;
		}
	    }
	}

	int total_trajectory = 0;
	for (Trajectories_map::const_iterator trajectory_record = robot_Trajectories.begin(); trajectory_record != robot_Trajectories.end(); ++trajectory_record)
	{
	    Distribution_map::iterator distribution_record = m_distribution_Trajectories.find(trajectory_record->second);
	    if (distribution_record != m_distribution_Trajectories.end())
	    {
		++distribution_record->second;
	    }
	    else
	    {
		m_distribution_Trajectories[trajectory_record->second] = 1;
	    }
	    total_trajectory += trajectory_record->second;
	}
	m_average_trajectory = (double)total_trajectory / N_Robots;

	m_total_makespan = N_Steps;
	m_total_trajectory = total_N_Moves;

	m_total_cost = N_Steps * N_Robots;


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_last_move_step = -1;

	    int st = 0;
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    if (move->m_robot_id == robot_id && move->m_dest_vrtx_id == final_arrangement.get_RobotLocation(robot_id) && move->m_dest_vrtx_id != move->m_src_vrtx_id)
		    {
			robot_last_move_step = st;
		    }
		}
		++st;
	    }
	    int placed_steps = N_Steps - robot_last_move_step - 1;
	    m_total_cost -= placed_steps;
	}
	printf("Total cost = %d\n", m_total_cost);
	return m_total_cost;
    }
*/

    void sMultirobotSolutionAnalyzer::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sMultirobotSolutionAnalyzer::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMultirobot solution analysis: (\n", indent.c_str());

	fprintf(fw, "%s%stotal makespan           = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), m_total_makespan);
	fprintf(fw, "%s%stotal distance           = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), m_total_distance);
	fprintf(fw, "%s%stotal trajectory         = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), m_total_trajectory);
	fprintf(fw, "%s%stotal cost               = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), m_total_cost);

	fprintf(fw, "%s%saverage parallelism      = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), m_average_parallelism);
	fprintf(fw, "%s%saverage distance         = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), m_average_distance);
	fprintf(fw, "%s%saverage trajectory       = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), m_average_trajectory);

	fprintf(fw, "%s%sparallelism distribution  = ", indent.c_str(), sRELOC_INDENT.c_str());
	to_Stream_distribution(fw, m_distribution_Parallelisms, "");

	fprintf(fw, "%s%sdistance distribution     = ", indent.c_str(), sRELOC_INDENT.c_str());
	to_Stream_distribution(fw, m_distribution_Distances, "");

	fprintf(fw, "%s%strajectory distribution   = ", indent.c_str(), sRELOC_INDENT.c_str());
	to_Stream_distribution(fw, m_distribution_Trajectories, "");

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sMultirobotSolutionAnalyzer::to_Screen_distribution(const Distribution_map &distribution, const sString &indent) const
    {
	to_Stream_distribution(stdout, distribution, indent);
    }


    void sMultirobotSolutionAnalyzer::to_Stream_distribution(FILE *fw, const Distribution_map &distribution, const sString &indent) const
    {
	int last_value = 0;

	fprintf(fw, "%s[ ", indent.c_str());

	for (Distribution_map::const_iterator distribution_record = distribution.begin(); distribution_record != distribution.end(); ++distribution_record)
	{
	    for (int value = last_value + 1; value < distribution_record->first; ++value)
	    {
		fprintf(fw, "0 ");
	    }
	    fprintf(fw, "%d ", distribution_record->second);
	    last_value = distribution_record->first;
	}
	fprintf(fw, "]\n");
    }


/*----------------------------------------------------------------------------*/
// sPhaseStatistics

    const double sPhaseStatistics::SECONDS_UNDEFINED = 1.0;
    const sString sPhaseStatistics::ROOT_PHASE_NAME = "root_phase";


/*----------------------------------------------------------------------------*/

    sPhaseStatistics::Phase::Phase(const sString &name, Phase *parent_phase)
	: m_name(name)
	, m_WC_Seconds(0.0)
	, m_CPU_Seconds(0.0)
	, m_total_sat_solver_Calls(0)
	, m_SAT_sat_solver_Calls(0)
	, m_UNSAT_sat_solver_Calls(0)
	, m_INDET_sat_solver_Calls(0)
	, m_move_Executions(0)
	, m_produced_cnf_Variables(0)
	, m_produced_cnf_Clauses(0)
	, m_search_Steps(0)
	, m_parent_phase(parent_phase)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sPhaseStatistics::sPhaseStatistics()
    {
	m_root_phase = new Phase(ROOT_PHASE_NAME, NULL);
	m_current_phase = m_root_phase;
	restart_CurrentPhase();
    }


    sPhaseStatistics::~sPhaseStatistics()
    {
	for (Phases_map::iterator phase = m_root_phase->m_sub_Phases.begin(); phase != m_root_phase->m_sub_Phases.end(); ++phase)
	{
	    delete phase->second;
	}
	delete m_root_phase;
    }


    sPhaseStatistics::Phase& sPhaseStatistics::get_CurrentPhase(void)
    {
	return *m_current_phase;
    }


    void sPhaseStatistics::enter_Root(void)
    {
	while (m_current_phase->m_parent_phase != NULL)
	{
	    leave_Phase();
	}
    }


    void sPhaseStatistics::enter_Phase(const sString &phase_name)
    {
	if (phase_name != m_current_phase->m_name)
	{
	    suspend_CurrentPhase();

	    Phases_map::iterator phase = m_current_phase->m_sub_Phases.find(phase_name);
	    if (phase != m_current_phase->m_sub_Phases.end())
	    {
		m_current_phase = phase->second;
	    }
	    else
	    {
		Phase *created_phase = new Phase(phase_name, m_current_phase);
		std::pair<Phases_map::iterator, bool> phase = m_current_phase->m_sub_Phases.insert(Phases_map::value_type(phase_name, created_phase));
		m_current_phase = phase.first->second;
	    }

	    restart_CurrentPhase();
	}
    }


    void sPhaseStatistics::leave_Phase(void)
    {
	suspend_CurrentPhase();
	    
	if (m_current_phase->m_parent_phase != NULL)
	{
	    m_current_phase = m_current_phase->m_parent_phase;
	}

	restart_CurrentPhase();
    }


    double sPhaseStatistics::get_WC_Seconds(void)
    {
	struct timeval timeval;
	struct timezone timezone;

	if (gettimeofday(&timeval, &timezone) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return timeval.tv_sec + timeval.tv_usec / static_cast<double>(1000000.0);
	}
    }


    void sPhaseStatistics::restart_CurrentPhase(void)
    {
	m_curr_phase_start_WC = get_WC_Seconds();
	m_curr_phase_start_CPU = get_CPU_Seconds();
    }


    void sPhaseStatistics::suspend_CurrentPhase(void)
    {
	m_curr_phase_finish_WC = get_WC_Seconds();
	m_curr_phase_finish_CPU = get_CPU_Seconds();

	m_current_phase->m_WC_Seconds += m_curr_phase_finish_WC - m_curr_phase_start_WC;
	m_current_phase->m_CPU_Seconds += m_curr_phase_finish_CPU - m_curr_phase_start_CPU;
    }


    double sPhaseStatistics::get_CPU_Seconds(void)
    {
	struct tms tms_record;

	if (times(&tms_record) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return ((tms_record.tms_utime + tms_record.tms_stime + tms_record.tms_cutime + tms_record.tms_cstime) / (double)sysconf(_SC_CLK_TCK));
	}
    }


    void sPhaseStatistics::to_Screen(const sString &indent)
    {
	to_Stream(stdout, indent);
    }


    void sPhaseStatistics::to_Stream(FILE *fw, const sString &indent)
    {
	suspend_CurrentPhase();

	fprintf(fw, "%sPhase statistics (current phase = '%s') [\n", indent.c_str(), m_current_phase->m_name.c_str());
	to_Stream_subphases(fw, *m_root_phase, indent + sRELOC_INDENT);
	fprintf(fw, "%s]\n", indent.c_str());

	restart_CurrentPhase();
    }


    void sPhaseStatistics::to_Screen_subphases(const Phase &phase, const sString &indent)
    {
	to_Stream_subphases(stdout, phase, indent);
    }


    void sPhaseStatistics::to_Stream_subphases(FILE *fw, const Phase &phase, const sString &indent)
    {
	fprintf(fw, "%s%sPhase (name = '%s') [\n", indent.c_str(), sRELOC_INDENT.c_str(), phase.m_name.c_str());
	fprintf(fw, "%s%s%sTotal SAT solver calls         = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_total_sat_solver_Calls);
	fprintf(fw, "%s%s%sSatisfiable SAT solver calls   = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_SAT_sat_solver_Calls);
	fprintf(fw, "%s%s%sUnsatisfiable SAT solver calls = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_UNSAT_sat_solver_Calls);
	fprintf(fw, "%s%s%sIndeterminate SAT solver calls = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_INDET_sat_solver_Calls);
	fprintf(fw, "%s%s%sMove executions                = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_move_Executions);
	fprintf(fw, "%s%s%sProduced CNF variables         = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_produced_cnf_Variables);
	fprintf(fw, "%s%s%sProduced CNF clauses           = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_produced_cnf_Clauses);
	fprintf(fw, "%s%s%sSearch steps                   = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_search_Steps);
	fprintf(fw, "%s%s%sWall clock TIME (seconds)      = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_WC_Seconds);
	fprintf(fw, "%s%s%sCPU/machine TIME (seconds)     = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_CPU_Seconds);
	fprintf(fw, "%s%s]\n", indent.c_str(), sRELOC_INDENT.c_str());

	if (!phase.m_sub_Phases.empty())
	{
	    fprintf(fw, "%s%sSub-phases {\n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (Phases_map::const_iterator sub_phase = phase.m_sub_Phases.begin(); sub_phase != phase.m_sub_Phases.end(); ++sub_phase)
	    {
		to_Stream_subphases(fw, *sub_phase->second, indent + sRELOC_INDENT);
	    }
	    fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());
	}
    }


/*----------------------------------------------------------------------------*/
// Global objects

    sPhaseStatistics s_GlobalPhaseStatistics;


/*----------------------------------------------------------------------------*/
// Global functions

    double sGet_WC_Seconds(void)
    {
	return sPhaseStatistics::get_WC_Seconds();
    }


    double sGet_CPU_Seconds(void)
    {
	return sPhaseStatistics::get_CPU_Seconds();
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
