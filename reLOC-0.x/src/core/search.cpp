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
/* search.cpp / 0.21-robik_054                                                */
/*----------------------------------------------------------------------------*/
//
// Search-based solving of multi-robot path-finding problem.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "types.h"
#include "result.h"
#include "cnf.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"
#include "search.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{



/*----------------------------------------------------------------------------*/
// sMultirobotSolver

    sMultirobotSolver::sMultirobotSolver()
    {
	// nothing
    }


    void sMultirobotSolver::setup_Solver(void)
    {
	// nothing;
    }


    void sMultirobotSolver::setup_Instance(const sMultirobotInstance &multirobot_instnace)
    {
	m_multirobot_instance = multirobot_instnace;
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_ID

    sMultirobotSolver_ID::sMultirobotSolver_ID()
	: m_max_depth_limit(0),
	  m_finished_Robot_cnt(0)
    {
	// nothing
    }


    void sMultirobotSolver_ID::setup_Solver(int max_depth_limit)
    {
	sMultirobotSolver::setup_Solver();
	m_max_depth_limit = max_depth_limit;
    }


    void sMultirobotSolver_ID::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_current_arrangement = multirobot_instance.m_initial_arrangement;

	m_finished_Robot_cnt = 0;

	int N_Robots = m_current_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    if (m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id))
	    {
		++m_finished_Robot_cnt;
	    }
	}
    }

    
    bool sMultirobotSolver_ID::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
	for (int depth_limit = 1; m_max_depth_limit < 0 || depth_limit <= m_max_depth_limit; ++depth_limit)
	{
	    bool solution_found = search_Goal(0, depth_limit, multirobot_solution);

	    if (solution_found)
	    {
		return true;
	    }
	}

	return false;
    }


    bool sMultirobotSolver_ID::search_Goal(int current_depth, int depth_limit, sMultirobotSolution &multirobot_solution)
    {
#ifdef sSTATISTICS
	static sUInt_64 search_Step_cnt = 0;
	++search_Step_cnt;
#endif    

#ifdef sVERBOSE	
	static sUInt_64 verbose_period = 65536;
	static sUInt_64 Verbose_cnt = 0;

	if (Verbose_cnt++ >= verbose_period)
	{
	    printf("Current depth/limit (steps): ");
	    for (int i = 0; i < depth_limit; ++i)
	    {
		printf(".");
	    }
	    printf(" %d/%d (%llu)\n", current_depth, depth_limit, search_Step_cnt);

	    Verbose_cnt = 0;
	    verbose_period *= 1.8;
	    ++verbose_period;
	}
#endif

	if (m_finished_Robot_cnt == m_current_arrangement.get_RobotCount())
	{
	    return true;
	}
	else
	{
	    if (current_depth < depth_limit)
	    {
		int N_Robots = m_current_arrangement.get_RobotCount();
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    int robot_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		    sVertex* robot_vertex = m_multirobot_instance.m_environment.get_Vertex(robot_vertex_id);
		    
		    for (sVertex::Neighbors_list::const_iterator neighbor = robot_vertex->m_Neighbors.begin(); neighbor != robot_vertex->m_Neighbors.end(); ++neighbor)
		    {
			int neighbor_vertex_id = (*neighbor)->m_target->m_id;
			
			if (m_current_arrangement.get_VertexOccupancy(neighbor_vertex_id) == sRobotArrangement::VACANT_VERTEX)
			{
			    conduct_Move(robot_id, robot_vertex_id, neighbor_vertex_id);			    
			    bool result = search_Goal(current_depth + 1, depth_limit, multirobot_solution);
			    
			    if (result)
			    {
				multirobot_solution.add_Move(current_depth, sMultirobotSolution::Move(robot_id, robot_vertex_id, neighbor_vertex_id));
				return true;
			    }
			    else
			    {
				conduct_Move(robot_id, neighbor_vertex_id, robot_vertex_id);
			    }		 
			}
		    }
		}
		return false;
	    }
	    else
	    {
		return false;
	    }
	}

	sASSERT(false);
	return false;
    }


    void sMultirobotSolver_ID::conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id)
    {
	const sRobotArrangement &goal_arrangement = m_multirobot_instance.m_goal_arrangement;

	if (src_vrtx_id == goal_arrangement.get_RobotLocation(robot_id) && src_vrtx_id != dest_vrtx_id)
	{
	    --m_finished_Robot_cnt;
	}
	else
	{
	    if (dest_vrtx_id == goal_arrangement.get_RobotLocation(robot_id) && dest_vrtx_id != src_vrtx_id)
	    {
		++m_finished_Robot_cnt;
	    }
	}
	m_current_arrangement.move_Robot(robot_id, dest_vrtx_id);
    }


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Trivial

    sAstarHeuristic_Trivial::sAstarHeuristic_Trivial()
    {
	// nothing
    }


    void sAstarHeuristic_Trivial::setup_Instance(const sMultirobotInstance &sUNUSED(multirobot_instance))
    {
	// nothing
    }


    int sAstarHeuristic_Trivial::calc_Heuristic(int sUNUSED(source_vertex_id), int sUNUSED(destination_vertex_id))
    {
	return 0;
    }


    int sAstarHeuristic_Trivial::calc_Heuristic(const sRobotArrangement &sUNUSED(current_arrangement), const sRobotArrangement &sUNUSED(goal_arrangement))
    {
	return 0;
    }


    int sAstarHeuristic_Trivial::calc_Heuristic(const sRobotArrangement &sUNUSED(current_arrangement), const sRobotGoal &sUNUSED(robot_goal))
    {
	return 0;
    }


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Placement

    sAstarHeuristic_Placement::sAstarHeuristic_Placement()
    {
	// nothing
    }


    void sAstarHeuristic_Placement::setup_Instance(const sMultirobotInstance &sUNUSED(multirobot_instance))
    {
	// nothing
    }


    int sAstarHeuristic_Placement::calc_Heuristic(int source_vertex_id, int destination_vertex_id)
    {
	if (source_vertex_id == destination_vertex_id)
	{
	    return 0;
	}
	else
	{
	    return 1;
	}
    }


    int sAstarHeuristic_Placement::calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement)
    {
	int placed_Robot_cnt = 0;

	int N_Robots = current_arrangement.get_RobotCount();
	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    if (current_arrangement.get_RobotLocation(r_id) == goal_arrangement.get_RobotLocation(r_id))
	    {
		++placed_Robot_cnt;
	    }
	}

	return N_Robots - placed_Robot_cnt;
    }


    int sAstarHeuristic_Placement::calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal)
    {
	int placed_Robot_cnt = 0;

	int N_Robots = current_arrangement.get_RobotCount();
	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    const sRobotGoal::Vertices_set &goal_Vertices = robot_goal.get_RobotGoal(r_id);

	    if (goal_Vertices.find(current_arrangement.get_RobotLocation(r_id)) != goal_Vertices.end())
	    {
		++placed_Robot_cnt;
	    }
	}

	return N_Robots - placed_Robot_cnt;
    }


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Distance

    sAstarHeuristic_Distance::sAstarHeuristic_Distance()
    {
	// nothing
    }


    void sAstarHeuristic_Distance::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	const sUndirectedGraph &environment = multirobot_instance.m_environment;
	int N_Vertices = environment.get_VertexCount();

	for (int v_id = 0; v_id < N_Vertices; ++v_id)
	{
	    MatrixRow_vector matrix_Row;
	    environment.calc_SingleSourceShortestPaths(v_id, matrix_Row);
	    m_distance_Matrix.push_back(matrix_Row);
	}
    }


    int sAstarHeuristic_Distance::calc_Heuristic(int source_vertex_id, int destination_vertex_id)
    {
	return m_distance_Matrix[source_vertex_id][destination_vertex_id];
    }


    int sAstarHeuristic_Distance::calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement)
    {
	int N_Robots = current_arrangement.get_RobotCount();

	int minimum_distance = sINT_32_MAX;
	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int distance = m_distance_Matrix[current_arrangement.get_RobotLocation(r_id)][goal_arrangement.get_RobotLocation(r_id)];
	    if (minimum_distance > distance)
	    {
		minimum_distance = distance;
	    }
	}

	return minimum_distance;
    }


    int sAstarHeuristic_Distance::calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal)
    {
	int N_Robots = current_arrangement.get_RobotCount();

	int minimum_minimum_distance = sINT_32_MAX;
	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int minimum_distance = sINT_32_MAX;
	    const sRobotGoal::Vertices_set &goal_Vertices = robot_goal.get_RobotGoal(r_id);

	    for (sRobotGoal::Vertices_set::const_iterator goal_vertex_id = goal_Vertices.begin(); goal_vertex_id != goal_Vertices.end(); ++goal_vertex_id)
	    {
		int distance = m_distance_Matrix[current_arrangement.get_RobotLocation(r_id)][*goal_vertex_id];
		if (minimum_distance > distance)
		{
		    minimum_distance = distance;
		}
	    }
	    if (minimum_minimum_distance > minimum_distance)
	    {
		minimum_minimum_distance = minimum_distance;
	    }
	}
	return minimum_minimum_distance;
    }


    void sAstarHeuristic_Distance::to_String_matrix(const sString &indent) const
    {
	sASSERT(!m_distance_Matrix.empty());

	printf("%sDistance matrix:\n", indent.c_str());
	printf("%s\t", indent.c_str());

	for (int column = 0; column < m_distance_Matrix[0].size(); ++column)
	{
	    printf("%d\t", column);
	}
	printf("\n");

	for (int row = 0; row < m_distance_Matrix.size(); ++row)
	{
	    printf("%s%d:\t", indent.c_str(), row);
	    for (MatrixRow_vector::const_iterator cell = m_distance_Matrix[row].begin(); cell != m_distance_Matrix[row].end(); ++cell)
	    {
		printf("%d\t", *cell);
	    }
	    printf("\n");
	}
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_Astar

    const sMultirobotSolver_Astar::ProductionMove sMultirobotSolver_Astar::UNDEFINED_PRODUCTION_MOVE = sMultirobotSolver_Astar::ProductionMove(NULL, sMultirobotSolution::UNDEFINED_MOVE);


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_Astar::ProductionMove::ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move)
	: m_prev(prev),
	  m_move(move)
    {
	// nothing
    }

/*----------------------------------------------------------------------------*/


    sMultirobotSolver_Astar::StateRecord::StateRecord(const sRobotArrangement &robot_arrangement)
	: m_cost(0),
	  m_production_move(NULL),
	  m_robot_arrangement(robot_arrangement)
    {
	// nothing
    }


    sMultirobotSolver_Astar::StateRecord::StateRecord(int cost, ProductionMove *production_move, const sRobotArrangement &robot_arrangement)
	: m_cost(cost),
	  m_production_move(production_move),
	  m_robot_arrangement(robot_arrangement)
    {
	// nothing
    }


    bool sMultirobotSolver_Astar::StateRecord::operator==(const StateRecord &state_record) const
    {
	return (m_robot_arrangement == state_record.m_robot_arrangement);
    }


    bool sMultirobotSolver_Astar::StateRecord::operator<(const StateRecord &state_record) const
    {
	return (m_robot_arrangement < state_record.m_robot_arrangement);
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_Astar::sMultirobotSolver_Astar()
	: m_heuristic(NULL)
    {
	// nothing
    }


    void sMultirobotSolver_Astar::setup_Solver(sAstarHeuristic *heuristic)
    {
	sMultirobotSolver::setup_Solver();
	m_heuristic = heuristic;
    } 


    void sMultirobotSolver_Astar::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_heuristic->setup_Instance(multirobot_instance);
    }


    bool sMultirobotSolver_Astar::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
	int first_state_cost = m_heuristic->calc_Heuristic(m_multirobot_instance.m_initial_arrangement, m_multirobot_instance.m_goal_arrangement);
	StateRecords_set::iterator first_state_iter = m_open_state_Recs.insert(StateRecord(first_state_cost, NULL, m_multirobot_instance.m_initial_arrangement)).first;
	m_expansion_Queue.insert(StateRecords_multimap::value_type(first_state_cost, first_state_iter));

	while (!m_open_state_Recs.empty())
	{
#ifdef sSTATISTICS
	    static sUInt_64 search_Step_cnt = 0;
	    ++search_Step_cnt;
#endif    

#ifdef sVERBOSE	
	    static sUInt_64 verbose_period = 65536;
	    static sUInt_64 Verbose_cnt = 0;

	    if (Verbose_cnt++ >= verbose_period)
	    {
		printf("Open/close/exp size (steps): %lu/%lu/%lu (%llu)\n", m_open_state_Recs.size(), m_close_state_Recs.size(), m_expansion_Queue.size(), search_Step_cnt);
		
		Verbose_cnt = 0;
		verbose_period *= 1.8;
		++verbose_period;
	    }
#endif
	    StateRecords_multimap::iterator expand_record = m_expansion_Queue.begin();

	    if (expand_Arrangement(*expand_record->second, multirobot_solution))
	    {
		return true;
	    }
	    m_open_state_Recs.erase(expand_record->second);
	    m_expansion_Queue.erase(expand_record);
	}

	return false;
    }


    bool sMultirobotSolver_Astar::expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution)
    {
	int N_Robots = current_state_rec.m_robot_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_vertex_id = current_state_rec.m_robot_arrangement.get_RobotLocation(robot_id);
	    sVertex* robot_vertex = m_multirobot_instance.m_environment.get_Vertex(robot_vertex_id);
	    
	    for (sVertex::Neighbors_list::const_iterator neighbor = robot_vertex->m_Neighbors.begin(); neighbor != robot_vertex->m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_vertex_id = (*neighbor)->m_target->m_id;
		
		if (current_state_rec.m_robot_arrangement.get_VertexOccupancy(neighbor_vertex_id) == sRobotArrangement::VACANT_VERTEX)
		{
		    sRobotArrangement next_arrangement;
		    conduct_Move(robot_id, robot_vertex_id, neighbor_vertex_id, current_state_rec.m_robot_arrangement, next_arrangement);
		    if (next_arrangement == m_multirobot_instance.m_goal_arrangement)
		    {
			int time = collect_Solution(current_state_rec, multirobot_solution);
			multirobot_solution.add_Move(time, sMultirobotSolution::Move(robot_id, robot_vertex_id, neighbor_vertex_id));
			return true;
		    }
		    else
		    {
			if (m_open_state_Recs.find(StateRecord(next_arrangement)) == m_open_state_Recs.end() && m_close_state_Recs.find(StateRecord(next_arrangement)) == m_close_state_Recs.end())
			{
			    m_production_Moves.push_back(ProductionMove(current_state_rec.m_production_move, sMultirobotSolution::Move(robot_id, robot_vertex_id, neighbor_vertex_id)));
			    int next_state_cost = m_heuristic->calc_Heuristic(next_arrangement, m_multirobot_instance.m_goal_arrangement) + current_state_rec.m_cost;
			    StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(StateRecord(next_state_cost, &m_production_Moves.back(), next_arrangement)).first;

			    m_expansion_Queue.insert(StateRecords_multimap::value_type(next_state_cost, next_state_iter));
			}
		    }
		}
	    }
	}
	m_close_state_Recs.insert(current_state_rec);

	/*
	printf("Open list:\n");
	for (StateRecords_set::const_iterator open = m_open_state_Recs.begin(); open != m_open_state_Recs.end(); ++open)
	{
	    open->m_robot_arrangement.to_Screen();
	}
	printf("Close list:\n");
	for (StateRecords_set::const_iterator close = m_close_state_Recs.begin(); close != m_close_state_Recs.end(); ++close)
	{
	    close->m_robot_arrangement.to_Screen();
	}
	printf("Expansion queue:\n");
	for (StateRecords_multimap::const_iterator elem = m_expansion_queue.begin(); elem != m_expansion_queue.end(); ++elem)
	{
	    elem->second->m_robot_arrangement.to_Screen();
	}
	*/
	return false;
    }


    int sMultirobotSolver_Astar::collect_Solution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution)
    {
	ProductionMove *production_move = current_state_rec.m_production_move;
	std::vector<ProductionMove*> production_Moves;

	for (; production_move != NULL; production_move = production_move->m_prev)
	{
	    production_Moves.push_back(production_move);
	}

	int time = 0;
	for (vector<ProductionMove*>::const_reverse_iterator prod_move = production_Moves.rbegin(); prod_move != production_Moves.rend(); ++prod_move, ++time)
	{
	    multirobot_solution.add_Move(time, (*prod_move)->m_move);
	}
	return time;
    }


    void sMultirobotSolver_Astar::conduct_Move(int robot_id, int sUNUSED(src_vrtx_id), int dest_vrtx_id, const sRobotArrangement &current_arrangement, sRobotArrangement &next_arrangement)
    {
	next_arrangement = current_arrangement;
	next_arrangement.move_Robot(robot_id, dest_vrtx_id);
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_HCAstar

    const sMultirobotSolver_HCAstar::ProductionMove sMultirobotSolver_HCAstar::UNDEFINED_PRODUCTION_MOVE = sMultirobotSolver_HCAstar::ProductionMove(NULL, sMultirobotSolution::UNDEFINED_MOVE);


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_HCAstar::ProductionMove::ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move)
	: m_prev(prev)
	, m_move(move)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_HCAstar::StateRecord::StateRecord(int location_id, int time_step)
	: m_location_id(location_id)
	, m_time_step(time_step)
	, m_production_move(NULL)
    {
	// nothing
    }


    bool sMultirobotSolver_HCAstar::StateRecord::operator==(const StateRecord &state_record) const
    {
	return (m_time_step == state_record.m_time_step && m_location_id == state_record.m_location_id);
    }


    bool sMultirobotSolver_HCAstar::StateRecord::operator<(const StateRecord &state_record) const
    {
	return (m_time_step < state_record.m_time_step || (m_time_step == state_record.m_time_step && m_location_id < state_record.m_location_id));
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_HCAstar::ReservationRecord::ReservationRecord(int vertex_id, int time_step)
	: m_vertex_id(vertex_id)
	, m_time_step(time_step)
    {
	// nothing
    }


    bool sMultirobotSolver_HCAstar::ReservationRecord::operator==(const ReservationRecord &reservation_record) const
    {
	return (m_vertex_id == reservation_record.m_vertex_id && m_time_step == reservation_record.m_time_step);
    }


    bool sMultirobotSolver_HCAstar::ReservationRecord::operator<(const ReservationRecord &reservation_record) const
    {
	return (m_time_step < reservation_record.m_time_step || (m_time_step == reservation_record.m_time_step && m_vertex_id < reservation_record.m_vertex_id));
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_HCAstar::sMultirobotSolver_HCAstar()
	: m_heuristic(NULL)
    {
	// nothing
    }

    
    void sMultirobotSolver_HCAstar::setup_Solver(sAstarHeuristic *heuristic)
    {
	sMultirobotSolver::setup_Solver();
	m_heuristic = heuristic;
    }


    void sMultirobotSolver_HCAstar::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_heuristic->setup_Instance(multirobot_instance);

	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    m_reservation_Table.insert(ReservationRecord(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), 0));
	}
    }


    bool sMultirobotSolver_HCAstar::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    if (!solve_Robot(robot_id, multirobot_solution))
	    {
		printf("Robot %d not solved\n", robot_id);
		return false;
	    }
	}
	return true;
    }


    bool sMultirobotSolver_HCAstar::solve_Robot(int robot_id, sMultirobotSolution &multirobot_solution)
    {
	m_open_state_Recs.clear();
	m_close_state_Recs.clear();
	m_expansion_Queue.clear();
	m_production_Moves.clear();

	StateRecord first_state_rec(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), 0);
	first_state_rec.m_cost = m_heuristic->calc_Heuristic(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));
	StateRecords_set::iterator first_state_iter = m_open_state_Recs.insert(first_state_rec).first;
	m_expansion_Queue.insert(StateRecords_multimap::value_type(first_state_rec.m_cost, first_state_iter));

	while (!m_open_state_Recs.empty())
	{
	    StateRecords_multimap::iterator expand_record = m_expansion_Queue.begin();

	    if (expand_State(robot_id, *expand_record->second, m_reservation_Table, multirobot_solution))
	    {
		return true;
	    }
	    m_open_state_Recs.erase(expand_record->second);
	    m_expansion_Queue.erase(expand_record);
	}

	return false;
    }


    bool sMultirobotSolver_HCAstar::expand_State(int robot_id, const StateRecord &current_state_rec, ReservationTable_set &reservation_Table, sMultirobotSolution &multirobot_solution)
    {
	printf("Expanding state:%d,%d\n", current_state_rec.m_time_step, current_state_rec.m_location_id);
	sVertex *robot_location = m_multirobot_instance.m_environment.get_Vertex(current_state_rec.m_location_id);

	for (sVertex::Neighbors_list::const_iterator neighbor = robot_location->m_Neighbors.begin(); neighbor != robot_location->m_Neighbors.end(); ++neighbor)
	{
	    if (   reservation_Table.find(ReservationRecord((*neighbor)->m_target->m_id, current_state_rec.m_time_step)) == reservation_Table.end()
		&& reservation_Table.find(ReservationRecord((*neighbor)->m_target->m_id, current_state_rec.m_time_step + 1)) == reservation_Table.end())
	    {
		StateRecord next_state_rec((*neighbor)->m_target->m_id, current_state_rec.m_time_step + 1);
		
		if (m_open_state_Recs.find(next_state_rec) == m_open_state_Recs.end() && m_close_state_Recs.find(next_state_rec) == m_close_state_Recs.end())
		{
		    m_production_Moves.push_back(ProductionMove(current_state_rec.m_production_move, sMultirobotSolution::Move(robot_id, current_state_rec.m_location_id, (*neighbor)->m_target->m_id)));
		    next_state_rec.m_production_move = &m_production_Moves.back();
		    next_state_rec.m_cost = m_heuristic->calc_Heuristic((*neighbor)->m_target->m_id, m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id)) + current_state_rec.m_cost;	    
		    if ((*neighbor)->m_target->m_id == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			collect_Subsolution(next_state_rec, multirobot_solution, reservation_Table);
			return true;
		    }																						
		    StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(next_state_rec).first;		    
		    m_expansion_Queue.insert(StateRecords_multimap::value_type(next_state_rec.m_cost, next_state_iter));
		}
	    }
	}
	if (   reservation_Table.find(ReservationRecord(current_state_rec.m_location_id, current_state_rec.m_time_step)) == reservation_Table.end()
	    && reservation_Table.find(ReservationRecord(current_state_rec.m_location_id, current_state_rec.m_time_step + 1)) == reservation_Table.end())
	{
	    StateRecord next_state_rec(current_state_rec.m_location_id, current_state_rec.m_time_step + 1);
	    
	    if (m_open_state_Recs.find(next_state_rec) == m_open_state_Recs.end() && m_close_state_Recs.find(next_state_rec) == m_close_state_Recs.end())
	    {
		m_production_Moves.push_back(ProductionMove(current_state_rec.m_production_move, sMultirobotSolution::Move(robot_id, current_state_rec.m_location_id, current_state_rec.m_location_id)));
		next_state_rec.m_production_move = &m_production_Moves.back();
		next_state_rec.m_cost = m_heuristic->calc_Heuristic(current_state_rec.m_location_id, m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id)) + current_state_rec.m_cost;
		
		StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(next_state_rec).first;		    
		m_expansion_Queue.insert(StateRecords_multimap::value_type(next_state_rec.m_cost, next_state_iter));
	    }
	}
	m_close_state_Recs.insert(current_state_rec);

	return false;
    }


    int sMultirobotSolver_HCAstar::collect_Subsolution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, ReservationTable_set &reservation_Table)
    {
	ProductionMove *production_move = current_state_rec.m_production_move;
	vector<ProductionMove*> production_Moves;

	for (; production_move != NULL; production_move = production_move->m_prev)
	{
	    production_Moves.push_back(production_move);
	}

	int time = 0;
	for (vector<ProductionMove*>::const_reverse_iterator prod_move = production_Moves.rbegin(); prod_move != production_Moves.rend(); ++prod_move, ++time)
	{
	    if ((*prod_move)->m_move.m_src_vrtx_id != (*prod_move)->m_move.m_dest_vrtx_id)
	    {
		multirobot_solution.add_Move(time, (*prod_move)->m_move);
		printf("%d-%d\n", (*prod_move)->m_move.m_src_vrtx_id, time);
		reservation_Table.insert(ReservationRecord((*prod_move)->m_move.m_src_vrtx_id, time));
	    }
	    else
	    {
		printf("Waiting...\n");
		printf("%d-%d\n", (*prod_move)->m_move.m_src_vrtx_id, time);
		reservation_Table.insert(ReservationRecord((*prod_move)->m_move.m_src_vrtx_id, time));
	    }
	}
	reservation_Table.insert(ReservationRecord((*production_Moves.begin())->m_move.m_dest_vrtx_id, time));
	printf("%d-%d\n", (*production_Moves.begin())->m_move.m_dest_vrtx_id, time);
	printf("Reservation table size:%ld\n", reservation_Table.size());

	return time;
    }


    void sMultirobotSolver_HCAstar::to_Screen(const ReservationTable_set &reservation_Table, const sString &indent) const
    {
	int last_time_step = -1;

	printf("%sReservation table [", indent.c_str());
	for (ReservationTable_set::const_iterator reservation_record = reservation_Table.begin(); reservation_record != reservation_Table.end(); ++reservation_record)
	{
	    if (last_time_step != reservation_record->m_time_step)
	    {
		printf("\n%s%s", indent.c_str(), sRELOC_INDENT.c_str());
		last_time_step = reservation_record->m_time_step;
	    }
	    printf("(%d,%d) ", reservation_record->m_time_step, reservation_record->m_vertex_id);
	}
	printf("\n%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_WHCAstar

    const sMultirobotSolver_WHCAstar::ProductionMove sMultirobotSolver_WHCAstar::UNDEFINED_PRODUCTION_MOVE = sMultirobotSolver_WHCAstar::ProductionMove(NULL, sMultirobotSolution::UNDEFINED_MOVE);


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_WHCAstar::ProductionMove::ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move)
	: m_prev(prev)
	, m_move(move)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_WHCAstar::StateRecord::StateRecord(int location_id, int time_step)
	: m_location_id(location_id)
	, m_time_step(time_step)
	, m_production_move(NULL)
    {
	// nothing
    }


    bool sMultirobotSolver_WHCAstar::StateRecord::operator==(const StateRecord &state_record) const
    {
	return (m_time_step == state_record.m_time_step && m_location_id == state_record.m_location_id);
    }


    bool sMultirobotSolver_WHCAstar::StateRecord::operator<(const StateRecord &state_record) const
    {
	return (m_time_step < state_record.m_time_step || (m_time_step == state_record.m_time_step && m_location_id < state_record.m_location_id));
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_WHCAstar::ReservationRecord::ReservationRecord(int vertex_id, int time_step)
	: m_vertex_id(vertex_id)
	, m_time_step(time_step)
    {
	// nothing
    }


    bool sMultirobotSolver_WHCAstar::ReservationRecord::operator==(const ReservationRecord &reservation_record) const
    {
	return (m_vertex_id == reservation_record.m_vertex_id && m_time_step == reservation_record.m_time_step);
    }


    bool sMultirobotSolver_WHCAstar::ReservationRecord::operator<(const ReservationRecord &reservation_record) const
    {
	return (m_time_step < reservation_record.m_time_step || (m_time_step == reservation_record.m_time_step && m_vertex_id < reservation_record.m_vertex_id));
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_WHCAstar::sMultirobotSolver_WHCAstar()
	: m_heuristic(NULL)
    {
	// nothing
    }

    
    void sMultirobotSolver_WHCAstar::setup_Solver(sAstarHeuristic *heuristic, int window_size, int max_N_Iterations)
    {
	sMultirobotSolver::setup_Solver();
	m_heuristic = heuristic;
	m_max_N_Iterations = max_N_Iterations;
	m_window_size = window_size;
    }


    void sMultirobotSolver_WHCAstar::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_heuristic->setup_Instance(multirobot_instance);
    }


    bool sMultirobotSolver_WHCAstar::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("WHCA*_solving");
	}
        #endif

	int start_step = 0;

	for (int iteration = 0; iteration < m_max_N_Iterations; ++iteration)
	{
	    sRobotArrangement intermediate_arrangement;
	    sMultirobotSolution sub_solution;

	    if (!solve_Window(sub_solution))
	    {
                #ifdef sSTATISTICS
		{
		    s_GlobalPhaseStatistics.leave_Phase();
		}
                #endif
		return false;
	    }

	    int execution_N_Steps = sMIN(m_window_size / 2, sub_solution.get_StepCount());
	    sub_solution.execute_Solution(m_multirobot_instance.m_initial_arrangement, intermediate_arrangement, execution_N_Steps);

	    int solved_Robot_cnt = 0;
	    int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (intermediate_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id))
		{
		    ++solved_Robot_cnt;
		}
	    }	    

	    m_multirobot_instance.m_initial_arrangement = intermediate_arrangement;
	    sMultirobotSolution partial_sub_solution = sMultirobotSolution(start_step, sub_solution.extract_Subsolution(0, execution_N_Steps - 1));
	    multirobot_solution = sMultirobotSolution(multirobot_solution, partial_sub_solution);

	    start_step += execution_N_Steps;

	    if (solved_Robot_cnt >= N_Robots)
	    {
                #ifdef sSTATISTICS
		{
		    s_GlobalPhaseStatistics.leave_Phase();
		}
                #endif
		return true;
	    }
	}
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return false;
    }


    bool sMultirobotSolver_WHCAstar::solve_Window(sMultirobotSolution &multirobot_solution)
    {
	m_reservation_Table.clear();

	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    m_reservation_Table.insert(ReservationRecord(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), 0));
	}

	RobotPriority_multimap priority_Queue;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int distance = m_heuristic->calc_Heuristic(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id),
						       m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));
	    priority_Queue.insert(RobotPriority_multimap::value_type(distance, robot_id));
	}
	for (RobotPriority_multimap::const_iterator queue_record = priority_Queue.begin(); queue_record != priority_Queue.end(); ++queue_record)
	{
	    if (!solve_Robot(queue_record->second, multirobot_solution))
	    {
		return false;
	    }
	}
	return true;
    }


    bool sMultirobotSolver_WHCAstar::solve_Robot(int robot_id, sMultirobotSolution &multirobot_solution)
    {
	m_open_state_Recs.clear();
	m_close_state_Recs.clear();
	m_expansion_Queue.clear();
	m_production_Moves.clear();

	StateRecord first_state_rec(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), 0);
	first_state_rec.m_cost = m_heuristic->calc_Heuristic(m_multirobot_instance.m_initial_arrangement.get_RobotLocation(robot_id), m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));
	StateRecords_set::iterator first_state_iter = m_open_state_Recs.insert(first_state_rec).first;
	m_expansion_Queue.insert(StateRecords_multimap::value_type(first_state_rec.m_cost, first_state_iter));

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_search_Steps;
	}
        #endif

	while (!m_open_state_Recs.empty())
	{
	    StateRecords_multimap::iterator expand_record = m_expansion_Queue.begin();

	    if (expand_record->second->m_time_step >= m_window_size)
	    {
		collect_Subsolution(*expand_record->second, multirobot_solution, m_reservation_Table);
		return true;
	    }
	    else
	    {
		expand_State(robot_id, *expand_record->second, m_reservation_Table);
	    }
	    m_open_state_Recs.erase(expand_record->second);
	    m_expansion_Queue.erase(expand_record);
	}
	return false;
    }


    bool sMultirobotSolver_WHCAstar::expand_State(int robot_id, const StateRecord &current_state_rec, const ReservationTable_set &reservation_Table)
    {
	bool expanded = false;
	sVertex *robot_location = m_multirobot_instance.m_environment.get_Vertex(current_state_rec.m_location_id);

	for (sVertex::Neighbors_list::const_iterator neighbor = robot_location->m_Neighbors.begin(); neighbor != robot_location->m_Neighbors.end(); ++neighbor)
	{
	    if (   reservation_Table.find(ReservationRecord((*neighbor)->m_target->m_id, current_state_rec.m_time_step)) == reservation_Table.end()
		&& reservation_Table.find(ReservationRecord((*neighbor)->m_target->m_id, current_state_rec.m_time_step + 1)) == reservation_Table.end()
		&& reservation_Table.find(ReservationRecord((*neighbor)->m_target->m_id, current_state_rec.m_time_step + 2)) == reservation_Table.end())
	    {
		StateRecord next_state_rec((*neighbor)->m_target->m_id, current_state_rec.m_time_step + 1);
		
		if (m_open_state_Recs.find(next_state_rec) == m_open_state_Recs.end() && m_close_state_Recs.find(next_state_rec) == m_close_state_Recs.end())
		{
		    m_production_Moves.push_back(ProductionMove(current_state_rec.m_production_move, sMultirobotSolution::Move(robot_id, current_state_rec.m_location_id, (*neighbor)->m_target->m_id)));
		    next_state_rec.m_production_move = &m_production_Moves.back();
		    next_state_rec.m_cost = m_heuristic->calc_Heuristic((*neighbor)->m_target->m_id, m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id)) + current_state_rec.m_cost;		    
		    StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(next_state_rec).first;		    
		    m_expansion_Queue.insert(StateRecords_multimap::value_type(next_state_rec.m_cost, next_state_iter));
		    expanded = true;
		}
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_search_Steps;
	    }
            #endif
	}
	if (   reservation_Table.find(ReservationRecord(current_state_rec.m_location_id, current_state_rec.m_time_step)) == reservation_Table.end()
	    && reservation_Table.find(ReservationRecord(current_state_rec.m_location_id, current_state_rec.m_time_step + 1)) == reservation_Table.end()
	    && reservation_Table.find(ReservationRecord(current_state_rec.m_location_id, current_state_rec.m_time_step + 2)) == reservation_Table.end())
	{
	    StateRecord next_state_rec(current_state_rec.m_location_id, current_state_rec.m_time_step + 1);
	    
	    if (m_open_state_Recs.find(next_state_rec) == m_open_state_Recs.end() && m_close_state_Recs.find(next_state_rec) == m_close_state_Recs.end())
	    {
		m_production_Moves.push_back(ProductionMove(current_state_rec.m_production_move, sMultirobotSolution::Move(robot_id, current_state_rec.m_location_id, current_state_rec.m_location_id)));
		next_state_rec.m_production_move = &m_production_Moves.back();
		next_state_rec.m_cost = m_heuristic->calc_Heuristic(current_state_rec.m_location_id, m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id)) + current_state_rec.m_cost;
	
		StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(next_state_rec).first;		    
		m_expansion_Queue.insert(StateRecords_multimap::value_type(next_state_rec.m_cost, next_state_iter));
		expanded = true;
	    }
	}
        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_search_Steps;
	}
        #endif
	m_close_state_Recs.insert(current_state_rec);

	return expanded;	    
    }
    

    int sMultirobotSolver_WHCAstar::collect_Subsolution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, ReservationTable_set &reservation_Table)
    {
	ProductionMove *production_move = current_state_rec.m_production_move;
	vector<ProductionMove*> production_Moves;

	for (; production_move != NULL; production_move = production_move->m_prev)
	{
	    production_Moves.push_back(production_move);
	}

	int time = 0;
	for (vector<ProductionMove*>::const_reverse_iterator prod_move = production_Moves.rbegin(); prod_move != production_Moves.rend(); ++prod_move, ++time)
	{
	    if ((*prod_move)->m_move.m_src_vrtx_id != (*prod_move)->m_move.m_dest_vrtx_id)
	    {
		multirobot_solution.add_Move(time, (*prod_move)->m_move);
		reservation_Table.insert(ReservationRecord((*prod_move)->m_move.m_src_vrtx_id, time));
	    }
	    else
	    {
		reservation_Table.insert(ReservationRecord((*prod_move)->m_move.m_src_vrtx_id, time));
	    }
	}
	reservation_Table.insert(ReservationRecord((*production_Moves.begin())->m_move.m_dest_vrtx_id, time));

	return time;
    }


    void sMultirobotSolver_WHCAstar::to_Screen(const ReservationTable_set &reservation_Table, const sString &indent) const
    {
	int last_time_step = -1;

	printf("%sReservation table [", indent.c_str());
	for (ReservationTable_set::const_iterator reservation_record = reservation_Table.begin(); reservation_record != reservation_Table.end(); ++reservation_record)
	{
	    if (last_time_step != reservation_record->m_time_step)
	    {
		printf("\n%s%s", indent.c_str(), sRELOC_INDENT.c_str());
		last_time_step = reservation_record->m_time_step;
	    }
	    printf("(%d,%d) ", reservation_record->m_time_step, reservation_record->m_vertex_id);
	}
	printf("\n%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_IDAstar

    sMultirobotSolver_IDAstar::sMultirobotSolver_IDAstar()
	: m_max_cost_limit(0),
	  m_finished_Robot_cnt(0)
    {
	// nothing
    }


    void sMultirobotSolver_IDAstar::setup_Solver(int max_cost_limit, sAstarHeuristic *heuristic)
    {
	sMultirobotSolver::setup_Solver();
	m_heuristic = heuristic;
	m_max_cost_limit = max_cost_limit;
    }


    void sMultirobotSolver_IDAstar::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_heuristic->setup_Instance(multirobot_instance);

	m_current_arrangement = multirobot_instance.m_initial_arrangement;

	m_finished_Robot_cnt = 0;

	int N_Robots = m_current_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    if (m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id))
	    {
		++m_finished_Robot_cnt;
	    }
	}
    }

    
    bool sMultirobotSolver_IDAstar::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
	for (int cost_limit = 0; m_max_cost_limit < 0 || cost_limit <= m_max_cost_limit;)
	{
	    int next_cost_limit;
	    bool solution_found = search_Goal(0, cost_limit, next_cost_limit, multirobot_solution);

	    if (solution_found)
	    {
		return true;
	    }

	    cost_limit = next_cost_limit;
	}

	return false;
    }


    bool sMultirobotSolver_IDAstar::search_Goal(int current_depth, int cost_limit, int &next_cost_limit, sMultirobotSolution &multirobot_solution)
    {
#ifdef sSTATISTICS
	static sUInt_64 search_Step_cnt = 0;
	++search_Step_cnt;
#endif    

#ifdef sVERBOSE	
	static sUInt_64 verbose_period = 65536;
	static sUInt_64 Verbose_cnt = 0;

	if (Verbose_cnt++ >= verbose_period)
	{
	    printf("Current depth/limit (steps): ");
	    for (int i = 0; i < cost_limit; ++i)
	    {
		printf(".");
	    }
	    printf(" %d/%d (%llu)\n", current_depth, cost_limit, search_Step_cnt);

	    Verbose_cnt = 0;
	    verbose_period *= 1.8;
	    ++verbose_period;
	}
#endif

	if (m_finished_Robot_cnt == m_current_arrangement.get_RobotCount())
	{
	    return true;
	}
	else
	{
	    next_cost_limit = INT_MAX;

	    int N_Robots = m_current_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		sVertex* robot_vertex = m_multirobot_instance.m_environment.get_Vertex(robot_vertex_id);
		
		for (sVertex::Neighbors_list::const_iterator neighbor = robot_vertex->m_Neighbors.begin(); neighbor != robot_vertex->m_Neighbors.end(); ++neighbor)
		{
		    int neighbor_vertex_id = (*neighbor)->m_target->m_id;
		    
		    if (m_current_arrangement.get_VertexOccupancy(neighbor_vertex_id) == sRobotArrangement::VACANT_VERTEX)
		    {
			conduct_Move(robot_id, robot_vertex_id, neighbor_vertex_id);
			int cost = current_depth + m_heuristic->calc_Heuristic(m_current_arrangement, m_multirobot_instance.m_goal_arrangement);
			
			if (cost <= cost_limit)
			{
			    int node_cost_limit;
			    bool result = search_Goal(current_depth + 1, cost_limit, node_cost_limit, multirobot_solution);
			    
			    if (result)
			    {
				multirobot_solution.add_Move(current_depth, sMultirobotSolution::Move(robot_id, robot_vertex_id, neighbor_vertex_id));
				return true;
			    }
			    next_cost_limit = sMIN(next_cost_limit, node_cost_limit);
			}
			else
			{
			    if (cost > cost_limit)
			    {
				next_cost_limit = sMIN(next_cost_limit, cost);
			    }
			}
			conduct_Move(robot_id, neighbor_vertex_id, robot_vertex_id);
		    }
		}
	    }
	    return false;
	}
	
	sASSERT(false);
	return false;
    }


    void sMultirobotSolver_IDAstar::conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id)
    {
	const sRobotArrangement &goal_arrangement = m_multirobot_instance.m_goal_arrangement;

	if (src_vrtx_id == goal_arrangement.get_RobotLocation(robot_id) && src_vrtx_id != dest_vrtx_id)
	{
	    --m_finished_Robot_cnt;
	}
	else
	{
	    if (dest_vrtx_id == goal_arrangement.get_RobotLocation(robot_id) && dest_vrtx_id != src_vrtx_id)
	    {
		++m_finished_Robot_cnt;
	    }
	}
	m_current_arrangement.move_Robot(robot_id, dest_vrtx_id);
    }




/*----------------------------------------------------------------------------*/
// sMultirobotSolver_DecomposedAstar

    const sMultirobotSolver_DecomposedAstar::ProductionMove sMultirobotSolver_DecomposedAstar::UNDEFINED_PRODUCTION_MOVE = sMultirobotSolver_DecomposedAstar::ProductionMove(0, NULL, sMultirobotSolution::UNDEFINED_MOVE);


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_DecomposedAstar::ProductionMove::ProductionMove(int depth, ProductionMove *prev, sMultirobotSolution::Move move)
	: m_depth(depth),
	  m_prev(prev),
	  m_move(move)
    {
	// nothing
    }

/*----------------------------------------------------------------------------*/


    sMultirobotSolver_DecomposedAstar::StateRecord::StateRecord(const sRobotArrangement &robot_arrangement)
	: m_depth(0),
	  m_cost(0),
	  m_next_robot_id(1),
	  m_production_move(NULL),
	  m_robot_arrangement(robot_arrangement)
    {
	// nothing
    }


    sMultirobotSolver_DecomposedAstar::StateRecord::StateRecord(int next_robot_id, const sRobotArrangement &robot_arrangement, const Vertices_set &compromised_Vertices)
	: m_depth(0),
	  m_cost(0),
	  m_next_robot_id(next_robot_id),
	  m_production_move(NULL),
	  m_robot_arrangement(robot_arrangement),
	  m_compromised_Vertices(compromised_Vertices)
    {
	// nothing
    }


    sMultirobotSolver_DecomposedAstar::StateRecord::StateRecord(int cost, int depth, int next_robot_id, ProductionMove *production_move, const sRobotArrangement &robot_arrangement)
	: m_depth(depth),
	  m_cost(cost),
	  m_next_robot_id(next_robot_id),
	  m_production_move(production_move),
	  m_robot_arrangement(robot_arrangement)
    {
	// nothing
    }


    sMultirobotSolver_DecomposedAstar::StateRecord::StateRecord(int cost, int depth, int next_robot_id, ProductionMove *production_move, const sRobotArrangement &robot_arrangement, const Vertices_set &compromised_Vertices)
	: m_depth(depth),
	  m_cost(cost),
	  m_next_robot_id(next_robot_id),
	  m_production_move(production_move),
	  m_robot_arrangement(robot_arrangement),
	  m_compromised_Vertices(compromised_Vertices)
    {
	// nothing
    }


    bool sMultirobotSolver_DecomposedAstar::StateRecord::operator==(const StateRecord &state_record) const
    {
	if (m_robot_arrangement == state_record.m_robot_arrangement && m_next_robot_id == state_record.m_next_robot_id)
	{
	    if (m_compromised_Vertices.size() == state_record.m_compromised_Vertices.size())
	    {
		Vertices_set::const_iterator vertex_A = m_compromised_Vertices.begin();
		Vertices_set::const_iterator vertex_B = state_record.m_compromised_Vertices.begin();

		for (; vertex_A != m_compromised_Vertices.end(); ++vertex_A)
		{
		    if (*vertex_A != *vertex_B)
		    {
			return false;
		    }
		    sASSERT(vertex_B != state_record.m_compromised_Vertices.end());
		    ++vertex_B;
		}
		sASSERT(vertex_B == state_record.m_compromised_Vertices.end());		
		return true;
	    }
	    else
	    {
		return false;
	    }
	}
	else
	{
	    return false;
	}
    }


    bool sMultirobotSolver_DecomposedAstar::StateRecord::operator<(const StateRecord &state_record) const
    {
	if (m_robot_arrangement < state_record.m_robot_arrangement)
	{
	    return true;
	}
	else
	{
	    if (state_record.m_robot_arrangement < m_robot_arrangement)
	    {
		return false;
	    }
	    else
	    {
		if (m_next_robot_id < state_record.m_next_robot_id)
		{
		    return true;
		}
		else
		{
		    if (m_next_robot_id > state_record.m_next_robot_id)
		    {
			return false;
		    }
		    else
		    {
			sASSERT(m_robot_arrangement == state_record.m_robot_arrangement && m_next_robot_id == state_record.m_next_robot_id);

			if (m_compromised_Vertices.size() < state_record.m_compromised_Vertices.size())
			{
			    return true;
			}
			else
			{
			    if (m_compromised_Vertices.size() > state_record.m_compromised_Vertices.size())
			    {
				return false;
			    }
			    else
			    {
				sASSERT(m_compromised_Vertices.size() == state_record.m_compromised_Vertices.size());
				Vertices_set::const_iterator vertex_A = m_compromised_Vertices.begin();
				Vertices_set::const_iterator vertex_B = state_record.m_compromised_Vertices.begin();
			
				for (; vertex_A != m_compromised_Vertices.end(); ++vertex_A)
				{
				    if (*vertex_A < *vertex_B)
				    {
					return true;
				    }
				    else
				    {
					if (*vertex_A > *vertex_B)
					{
					    return false;
					}
					else
					{
					    sASSERT(*vertex_A == *vertex_B);
					}
				    }
				    sASSERT(vertex_B != state_record.m_compromised_Vertices.end());
				    ++vertex_B;
				}
				sASSERT(vertex_B == state_record.m_compromised_Vertices.end());
				return false;
			    }
			}
		    }
		}
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolver_DecomposedAstar::sMultirobotSolver_DecomposedAstar(int total_timeout)
	: m_total_timeout(total_timeout)
	, m_heuristic(NULL)
    {
	// nothing
    }


    void sMultirobotSolver_DecomposedAstar::setup_Solver(sAstarHeuristic *heuristic)
    {
	sMultirobotSolver::setup_Solver();
	m_heuristic = heuristic;
    } 


    void sMultirobotSolver_DecomposedAstar::setup_Instance(const sMultirobotInstance &multirobot_instance)
    {
	sMultirobotSolver::setup_Instance(multirobot_instance);
	m_heuristic->setup_Instance(multirobot_instance);
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::solve_Instance(sMultirobotSolution &multirobot_solution)
    {
	setup_Subinstance(m_multirobot_instance);
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("decomposed_solving");
	}
        #endif

	m_start_seconds = sGet_CPU_Seconds();
	m_finish_seconds = sGet_CPU_Seconds();

	SolutionResult result = solve_Subinstance(multirobot_solution);

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return result;
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::solve_DecomposedInstance(sMultirobotSolution &multirobot_solution)
    {
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("independent_solving");
	}
        #endif

	m_start_seconds = sGet_CPU_Seconds();
	m_finish_seconds = sGet_CPU_Seconds();

	Grouping_vector robot_Grouping;
	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();

	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    RobotGroup_set robot_group;
	    robot_group.insert(r_id);

	    robot_Grouping.push_back(robot_group);
	}
	SolutionResult result =  solve_DecomposedInstance(robot_Grouping, multirobot_solution);

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return result;
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::solve_DecomposedInstance(Grouping_vector &robot_Grouping, sMultirobotSolution &multirobot_solution)
    {
	Solutions_vector group_Solutions;
	Instances_vector group_Instances;

	for (Grouping_vector::const_iterator robot_group = robot_Grouping.begin(); robot_group != robot_Grouping.end(); ++robot_group)
	{
	    sMultirobotInstance multirobot_subinstance;
	    prepare_Subinstance(*robot_group, multirobot_subinstance);
	    group_Instances.push_back(multirobot_subinstance);

	    setup_Subinstance(multirobot_subinstance);

	    sMultirobotSolution group_subsolution;
	    switch (solve_Subinstance(group_subsolution))
	    {
	    case RESULT_SOLVABLE:
	    {
		break;
	    }
	    case RESULT_UNSOLVABLE:
	    {
		return RESULT_UNSOLVABLE;
	    }	    
	    case RESULT_INDETERMINATE:
	    {
		return RESULT_INDETERMINATE;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	    group_Solutions.push_back(group_subsolution);
	}
	while (robot_Grouping.size() > 1)
	{
	    switch (resolve_GroupCollisions(robot_Grouping, group_Instances, group_Solutions))
	    {
	    case RESOLUTION_UNSOLVABLE:
	    {
		return RESULT_UNSOLVABLE;
	    }
	    case RESOLUTION_MERGED:
	    {
		break;
	    }
	    case RESOLUTION_INDETERMINATE:
	    {
		return RESULT_INDETERMINATE;
	    }
	    case RESOLUTION_NONCOLLIDING:
	    {
		combine_Subsolutions(robot_Grouping, group_Solutions, multirobot_solution);
		return RESULT_SOLVABLE;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}
	combine_Subsolutions(robot_Grouping, group_Solutions, multirobot_solution);
	return RESULT_SOLVABLE;
    }


    void sMultirobotSolver_DecomposedAstar::setup_Subinstance(const sMultirobotInstance &multirobot_subinstance)
    {
	m_multirobot_subinstance = multirobot_subinstance;
	m_heuristic->setup_Instance(multirobot_subinstance);

	m_production_Moves.clear();
	m_expansion_Queue.clear();
	m_open_state_Recs.clear();
	m_close_state_Recs.clear();
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::solve_Subinstance(sMultirobotSolution &multirobot_solution)
    {
	int first_state_cost = m_heuristic->calc_Heuristic(m_multirobot_subinstance.m_initial_arrangement, m_multirobot_subinstance.m_goal_specification);

	StateRecords_set::iterator first_state_iter = m_open_state_Recs.insert(StateRecord(first_state_cost, 0, 0, NULL, m_multirobot_subinstance.m_initial_arrangement, Vertices_set())).first;
	m_expansion_Queue.insert(StateRecords_multimap::value_type(first_state_cost, first_state_iter));

	while (!m_open_state_Recs.empty())
	{
	    m_finish_seconds = sGet_CPU_Seconds();

	    if (m_finish_seconds - m_start_seconds  > m_total_timeout)
	    {
		return RESULT_INDETERMINATE;
	    }

#ifdef sSTATISTICS
	    static sUInt_64 search_Step_cnt = 0;
	    ++search_Step_cnt;
#endif    

#ifdef sVERBOSE	
	    static sUInt_64 verbose_period = 1;
	    static sUInt_64 Verbose_cnt = 0;

	    if (Verbose_cnt++ >= verbose_period)
	    {
		printf("Open/close/exp size (steps): %lu/%lu/%lu (%llu)\n", m_open_state_Recs.size(), m_close_state_Recs.size(), m_expansion_Queue.size(), search_Step_cnt);
		
		Verbose_cnt = 0;
		verbose_period *= 1.2;
		++verbose_period;
	    }
#endif
	    StateRecords_multimap::iterator expand_record = m_expansion_Queue.begin();
	    sASSERT(!m_expansion_Queue.empty());

	    if (expand_Arrangement(*expand_record->second, multirobot_solution))
	    {
		return RESULT_SOLVABLE;
	    }
	    m_open_state_Recs.erase(expand_record->second);
	    m_expansion_Queue.erase(expand_record);
	}

	return RESULT_UNSOLVABLE;
    }


    bool sMultirobotSolver_DecomposedAstar::expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution)
    {
	int N_Robots = current_state_rec.m_robot_arrangement.get_RobotCount();
	int next_robot_id = current_state_rec.m_next_robot_id + 1;
	int next_depth = current_state_rec.m_depth;
	Vertices_set next_compromised_Vertices = current_state_rec.m_compromised_Vertices;

	if (next_robot_id > N_Robots)
	{
	    next_robot_id = 1;
	    next_compromised_Vertices.clear();
	    ++next_depth;
	}

	int robot_vertex_id = current_state_rec.m_robot_arrangement.get_RobotLocation(next_robot_id);
	sVertex* robot_vertex = m_multirobot_subinstance.m_environment.get_Vertex(robot_vertex_id);

	{
	    sRobotArrangement next_arrangement(current_state_rec.m_robot_arrangement);

	    if (   m_open_state_Recs.find(StateRecord(next_robot_id, next_arrangement, next_compromised_Vertices)) == m_open_state_Recs.end()
		&& m_close_state_Recs.find(StateRecord(next_robot_id, next_arrangement, next_compromised_Vertices)) == m_close_state_Recs.end())
	    {
		int next_state_cost = m_heuristic->calc_Heuristic(next_arrangement, m_multirobot_subinstance.m_goal_specification);
		m_production_Moves.push_back(ProductionMove(next_depth, current_state_rec.m_production_move, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, robot_vertex_id)));

		StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(StateRecord(next_state_cost, next_depth, next_robot_id, &m_production_Moves.back(), next_arrangement, next_compromised_Vertices)).first;
		m_expansion_Queue.insert(StateRecords_multimap::value_type(next_depth + next_state_cost, next_state_iter));
	    }
	}
	for (sVertex::Neighbors_list::const_iterator neighbor = robot_vertex->m_Neighbors.begin(); neighbor != robot_vertex->m_Neighbors.end(); ++neighbor)
	{
	    int neighbor_vertex_id = (*neighbor)->m_target->m_id;

	    if (   current_state_rec.m_robot_arrangement.get_VertexOccupancy(neighbor_vertex_id) == sRobotArrangement::VACANT_VERTEX
	        && next_compromised_Vertices.find(neighbor_vertex_id) == next_compromised_Vertices.end())
	    {
		sRobotArrangement next_arrangement;
		conduct_Move(next_robot_id, robot_vertex_id, neighbor_vertex_id, current_state_rec.m_robot_arrangement, next_arrangement);		

		Vertices_set compromised_Vertices(next_compromised_Vertices);
		compromised_Vertices.insert(robot_vertex_id);

		if (m_multirobot_subinstance.m_goal_specification.is_Satisfied(next_arrangement))
		{
		    collect_Solution(current_state_rec, multirobot_solution);
		    multirobot_solution.add_Move(next_depth, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, neighbor_vertex_id));
		    return true;
		}
		else
		{
		    if (   m_open_state_Recs.find(StateRecord(next_robot_id, next_arrangement, compromised_Vertices)) == m_open_state_Recs.end()
			&& m_close_state_Recs.find(StateRecord(next_robot_id, next_arrangement, compromised_Vertices)) == m_close_state_Recs.end())
		    {
			m_production_Moves.push_back(ProductionMove(next_depth, current_state_rec.m_production_move, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, neighbor_vertex_id)));
			int next_state_cost = m_heuristic->calc_Heuristic(next_arrangement, m_multirobot_subinstance.m_goal_specification);

			StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(StateRecord(next_state_cost, next_depth, next_robot_id, &m_production_Moves.back(), next_arrangement, compromised_Vertices)).first;
			m_expansion_Queue.insert(StateRecords_multimap::value_type(next_depth + next_state_cost, next_state_iter));
		    }
		}
	    }
	}
	m_close_state_Recs.insert(current_state_rec);
	/*
	printf("Open list:\n");
	for (StateRecords_set::const_iterator open = m_open_state_Recs.begin(); open != m_open_state_Recs.end(); ++open)
	{
	    open->m_robot_arrangement.to_Screen();
	}
	printf("Close list:\n");
	for (StateRecords_set::const_iterator close = m_close_state_Recs.begin(); close != m_close_state_Recs.end(); ++close)
	{
	    close->m_robot_arrangement.to_Screen();
	}
	printf("Expansion queue:\n");
	for (StateRecords_multimap::const_iterator elem = m_expansion_queue.begin(); elem != m_expansion_queue.end(); ++elem)
	{
	    elem->second->m_robot_arrangement.to_Screen();
	}
	*/
	return false;
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::solve_Subinstance(sMultirobotSolution &multirobot_solution, const Occupation_2d_vector &Occupation)
    {
	int N_Robots = m_multirobot_subinstance.m_initial_arrangement.get_RobotCount();
	for (int r_id = 1; r_id <= N_Robots; ++r_id)
	{
	    int robot_vertex_id = m_multirobot_subinstance.m_initial_arrangement.get_RobotLocation(r_id);
	    if (Occupation[0][robot_vertex_id] != 0 || Occupation[1][robot_vertex_id] != 0)
	    {       	
		return RESULT_UNSOLVABLE;
	    }
	}
	int first_state_cost = m_heuristic->calc_Heuristic(m_multirobot_subinstance.m_initial_arrangement, m_multirobot_subinstance.m_goal_specification);

	StateRecords_set::iterator first_state_iter = m_open_state_Recs.insert(StateRecord(first_state_cost, 0, 0, NULL, m_multirobot_subinstance.m_initial_arrangement, Vertices_set())).first;
	m_expansion_Queue.insert(StateRecords_multimap::value_type(first_state_cost, first_state_iter));

	while (!m_open_state_Recs.empty())
	{
	    m_finish_seconds = sGet_CPU_Seconds();

	    if (m_finish_seconds - m_start_seconds  > m_total_timeout)
	    {
		return RESULT_INDETERMINATE;
	    }

#ifdef sSTATISTICS
	    static sUInt_64 search_Step_cnt = 0;
	    ++search_Step_cnt;
#endif    

#ifdef sVERBOSE	
	    static sUInt_64 verbose_period = 1;
	    static sUInt_64 Verbose_cnt = 0;

	    if (Verbose_cnt++ >= verbose_period)
	    {
		printf("** Open/close/exp size (steps): %lu/%lu/%lu (%llu)\n", m_open_state_Recs.size(), m_close_state_Recs.size(), m_expansion_Queue.size(), search_Step_cnt);
		
		Verbose_cnt = 0;
		verbose_period *= 1.2;
		++verbose_period;
	    }
#endif
	    StateRecords_multimap::iterator expand_record = m_expansion_Queue.begin();

	    if (expand_Arrangement(*expand_record->second, multirobot_solution, Occupation))
	    {
		return RESULT_SOLVABLE;
	    }
	    m_open_state_Recs.erase(expand_record->second);
	    m_expansion_Queue.erase(expand_record);
	}

	return RESULT_UNSOLVABLE;
    }


    bool sMultirobotSolver_DecomposedAstar::expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, const Occupation_2d_vector &Occupation)
    {
	int N_Robots = current_state_rec.m_robot_arrangement.get_RobotCount();
	int next_robot_id = current_state_rec.m_next_robot_id + 1;
	int next_depth = current_state_rec.m_depth;
	Vertices_set next_compromised_Vertices = current_state_rec.m_compromised_Vertices;

	if (next_robot_id > N_Robots)
	{
	    next_robot_id = 1;
	    next_compromised_Vertices.clear();
	    ++next_depth;
	}

	int max_depth = Occupation.size();
	if (next_depth < max_depth - 1)
	{
	    int robot_vertex_id = current_state_rec.m_robot_arrangement.get_RobotLocation(next_robot_id);
	    sVertex* robot_vertex = m_multirobot_subinstance.m_environment.get_Vertex(robot_vertex_id);

	    if (   Occupation[next_depth][robot_vertex_id] == 0
		&& (next_depth >= max_depth - 1 || Occupation[next_depth + 1][robot_vertex_id] == 0)
		&& (next_depth >= max_depth - 2 || Occupation[next_depth + 2][robot_vertex_id] == 0))
	    {
		sRobotArrangement next_arrangement(current_state_rec.m_robot_arrangement);
		
		if (   m_open_state_Recs.find(StateRecord(next_robot_id, next_arrangement, next_compromised_Vertices)) == m_open_state_Recs.end()
		    && m_close_state_Recs.find(StateRecord(next_robot_id, next_arrangement, next_compromised_Vertices)) == m_close_state_Recs.end())
		{
		    int next_state_cost = m_heuristic->calc_Heuristic(next_arrangement, m_multirobot_subinstance.m_goal_specification);
		    m_production_Moves.push_back(ProductionMove(next_depth, current_state_rec.m_production_move, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, robot_vertex_id)));

		    StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(StateRecord(next_state_cost, next_depth, next_robot_id, &m_production_Moves.back(), next_arrangement, next_compromised_Vertices)).first;
		    
		    m_expansion_Queue.insert(StateRecords_multimap::value_type(next_depth + next_state_cost, next_state_iter));
		}
	    }
	    for (sVertex::Neighbors_list::const_iterator neighbor = robot_vertex->m_Neighbors.begin(); neighbor != robot_vertex->m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_vertex_id = (*neighbor)->m_target->m_id;
		
		if (   current_state_rec.m_robot_arrangement.get_VertexOccupancy(neighbor_vertex_id) == sRobotArrangement::VACANT_VERTEX
		    && next_compromised_Vertices.find(neighbor_vertex_id) == next_compromised_Vertices.end())
		{
		    if (   Occupation[next_depth][neighbor_vertex_id] == 0
			&& (next_depth >= max_depth - 1 || Occupation[next_depth + 1][neighbor_vertex_id] == 0)
			&& (next_depth >= max_depth - 2 || Occupation[next_depth + 2][neighbor_vertex_id] == 0))
		    {
			sRobotArrangement next_arrangement;
			conduct_Move(next_robot_id, robot_vertex_id, neighbor_vertex_id, current_state_rec.m_robot_arrangement, next_arrangement);

			Vertices_set compromised_Vertices(next_compromised_Vertices);
			compromised_Vertices.insert(robot_vertex_id);
		    
			if (m_multirobot_subinstance.m_goal_specification.is_Satisfied(next_arrangement))
			{
			    collect_Solution(current_state_rec, multirobot_solution);
			    multirobot_solution.add_Move(next_depth, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, neighbor_vertex_id));
			    return true;
			}
			else
			{
			    if (   m_open_state_Recs.find(StateRecord(next_robot_id, next_arrangement, compromised_Vertices)) == m_open_state_Recs.end()
				&& m_close_state_Recs.find(StateRecord(next_robot_id, next_arrangement, compromised_Vertices)) == m_close_state_Recs.end())
			    {
				m_production_Moves.push_back(ProductionMove(next_depth, current_state_rec.m_production_move, sMultirobotSolution::Move(next_robot_id, robot_vertex_id, neighbor_vertex_id)));
				int next_state_cost = m_heuristic->calc_Heuristic(next_arrangement, m_multirobot_subinstance.m_goal_specification);
			
				StateRecords_set::iterator next_state_iter = m_open_state_Recs.insert(StateRecord(next_state_cost, next_depth, next_robot_id, &m_production_Moves.back(), next_arrangement, compromised_Vertices)).first;
				
				m_expansion_Queue.insert(StateRecords_multimap::value_type(next_depth + next_state_cost, next_state_iter));
			    }
			}
		    }
		}
	    }
	}
	m_close_state_Recs.insert(current_state_rec);
	/*
	printf("Open list:\n");
	for (StateRecords_set::const_iterator open = m_open_state_Recs.begin(); open != m_open_state_Recs.end(); ++open)
	{
	    open->m_robot_arrangement.to_Screen();
	}
	printf("Close list:\n");
	for (StateRecords_set::const_iterator close = m_close_state_Recs.begin(); close != m_close_state_Recs.end(); ++close)
	{
	    close->m_robot_arrangement.to_Screen();
	}
	printf("Expansion queue:\n");
	for (StateRecords_multimap::const_iterator elem = m_expansion_queue.begin(); elem != m_expansion_queue.end(); ++elem)
	{
	    elem->second->m_robot_arrangement.to_Screen();
	}
	*/
	return false;
    }


    void sMultirobotSolver_DecomposedAstar::collect_Solution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution)
    {
	ProductionMove *production_move = current_state_rec.m_production_move;
	vector<ProductionMove*> production_Moves;

	for (; production_move != NULL; production_move = production_move->m_prev)
	{
	    production_Moves.push_back(production_move);
	}
	for (vector<ProductionMove*>::const_reverse_iterator prod_move = production_Moves.rbegin(); prod_move != production_Moves.rend(); ++prod_move)
	{
	    if ((*prod_move)->m_move.m_src_vrtx_id != (*prod_move)->m_move.m_dest_vrtx_id)
	    {
		multirobot_solution.add_Move((*prod_move)->m_depth, (*prod_move)->m_move);
	    }
	}
    }


    void sMultirobotSolver_DecomposedAstar::conduct_Move(int robot_id, int sUNUSED(src_vrtx_id), int dest_vrtx_id, sRobotArrangement &next_arrangement) const
    {
	next_arrangement.move_Robot(robot_id, dest_vrtx_id);
    }


    void sMultirobotSolver_DecomposedAstar::conduct_Move(int robot_id, int sUNUSED(src_vrtx_id), int dest_vrtx_id, const sRobotArrangement &current_arrangement, sRobotArrangement &next_arrangement) const
    {
	next_arrangement = current_arrangement;
	next_arrangement.move_Robot(robot_id, dest_vrtx_id);
    }


    void sMultirobotSolver_DecomposedAstar::reset_Occupation(int max_depth, int N_Vertices, Occupation_2d_vector &Occupation) const
    {
	Occupation.resize(max_depth);

	for (int i = 0; i < max_depth; ++i)
	{
	    Occupation[i].resize(N_Vertices);
	    for (int j = 0; j < N_Vertices; ++j)
	    {
		Occupation[i][j] = 0;
	    }
	}
    }


    void sMultirobotSolver_DecomposedAstar::fill_Occupation(int max_depth, const sRobotArrangement &initial_arrangement, const sMultirobotSolution &multirobot_solution, Occupation_2d_vector &Occupation) const
    {
	sRobotArrangement current_arrangement = initial_arrangement;

	int N_Vertices = initial_arrangement.get_VertexCount();
	int depth = 0;

	for (sMultirobotSolution::Steps_vector::const_iterator step = multirobot_solution.m_Steps.begin(); step != multirobot_solution.m_Steps.end(); ++step)
	{
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		int robot_id = current_arrangement.get_VertexOccupancy(v_id);
		if (robot_id != 0)
		{
		    Occupation[depth][v_id] = robot_id;
		}
	    }
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		conduct_Move(move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, current_arrangement);
	    }
	    if (++depth > max_depth)
	    {
		break;
	    }
	}
	for (; depth < max_depth; ++depth)
	{
	    for (int v_id = 0; v_id < N_Vertices; ++v_id)
	    {
		int robot_id = current_arrangement.get_VertexOccupancy(v_id);
		if (robot_id != 0)
		{
		    Occupation[depth][v_id] = robot_id;
		}
	    }
	}
    }


    sMultirobotSolver_DecomposedAstar::ResolutionResult sMultirobotSolver_DecomposedAstar::resolve_GroupCollisions(Grouping_vector &robot_Grouping, Instances_vector &group_Instances, Solutions_vector &group_Solutions)
    {
	int N_Groups = group_Solutions.size();

	for (int group_index_A = 0; group_index_A < N_Groups - 1; ++group_index_A)
	{
	    for (int group_index_B = group_index_A + 1; group_index_B < N_Groups; ++group_index_B)
	    {
		if (are_GroupsColliding(group_index_A, group_index_B, group_Instances, group_Solutions))
		{
#ifdef sVERBOSE
		    {
			printf("Groups %d and %d collide.\n", group_index_A, group_index_B);
		    }
#endif
		    switch (resolve_GroupCollision(group_index_A, group_index_B, robot_Grouping, group_Instances, group_Solutions))
		    {
		    case RESULT_UNSOLVABLE:
		    {
#ifdef sVERBOSE
			{
			    printf("Unable to resolve collision between groups %d and %d.\n", group_index_A, group_index_B);
			}
#endif
			robot_Grouping[group_index_A].insert(robot_Grouping[group_index_B].begin(), robot_Grouping[group_index_B].end());
			robot_Grouping[group_index_B] = *robot_Grouping.rbegin();
			robot_Grouping.pop_back();

			sMultirobotInstance group_instance_A;
			prepare_Subinstance(robot_Grouping[group_index_A], group_instance_A);

			group_Instances[group_index_A] = group_instance_A;	
			group_Instances[group_index_B] = *group_Instances.rbegin();
			group_Instances.pop_back();

			setup_Subinstance(group_instance_A);

#ifdef sVERBOSE
			{
			    printf("Merging groups %d and %d.\n", group_index_A, group_index_B);
			    printf("Searching solution for merged group %d+%d.\n", group_index_A, group_index_B);
			}
#endif
			sMultirobotSolution group_subsolution_A;
			switch (solve_Subinstance(group_subsolution_A))
			{
			case RESULT_SOLVABLE:
			{
			    group_Solutions[group_index_A] = group_subsolution_A;
			    group_Solutions[group_index_B] = *group_Solutions.rbegin();
			    group_Solutions.pop_back();

			    return RESOLUTION_MERGED;
			}
			case RESULT_UNSOLVABLE:
			{
			    return RESOLUTION_UNSOLVABLE;
			}
			case RESULT_INDETERMINATE:
			{
			    return RESOLUTION_INDETERMINATE;
			}
			default:
			{
			    sASSERT(false);
			}
			}
			break;
		    }
		    case RESULT_SOLVABLE:
		    {
#ifdef sVERBOSE
			{
			    printf("Collision between groups %d and %d resolved.\n", group_index_A, group_index_B);
			}
#endif
			break;
		    }
		    case RESULT_INDETERMINATE:
		    {
			return RESOLUTION_INDETERMINATE;
		    }
		    default:
		    {
			sASSERT(false);
		    }
		    }
		}
	    }
	}
	return RESOLUTION_NONCOLLIDING;
    }


    bool sMultirobotSolver_DecomposedAstar::are_GroupsColliding(int group_index_A, int group_index_B, const Instances_vector &group_Instances, Solutions_vector &group_Solutions) const
    {
	Occupation_2d_vector Occupation;

	int max_depth = sMAX(group_Solutions[group_index_A].get_StepCount(), group_Solutions[group_index_B].get_StepCount()) + 1;
	reset_Occupation(max_depth, m_multirobot_instance.m_environment.get_VertexCount(), Occupation);
	fill_Occupation(max_depth, group_Instances[group_index_A].m_initial_arrangement, group_Solutions[group_index_A], Occupation);

#ifdef sVERBOSE
	{	
	    for (int i = 0; i < Occupation.size(); ++i)
	    {
		for (int j = 0; j < Occupation[i].size(); ++j)
		{
		    printf("%d ", Occupation[i][j]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
#endif
	
	sRobotArrangement current_arrangement = group_Instances[group_index_B].m_initial_arrangement;
	int N_Robots = current_arrangement.get_RobotCount();

	int depth = 0;
	for (sMultirobotSolution::Steps_vector::const_iterator step = group_Solutions[group_index_B].m_Steps.begin(); step != group_Solutions[group_index_B].m_Steps.end(); ++step)
	{
	    for (int r_id = 1; r_id <= N_Robots; ++r_id)
	    {
		if (   (depth > 0 && Occupation[depth - 1][current_arrangement.get_RobotLocation(r_id)] != 0)
		    || Occupation[depth][current_arrangement.get_RobotLocation(r_id)] != 0
		    || (depth < max_depth - 1 && Occupation[depth + 1][current_arrangement.get_RobotLocation(r_id)] != 0))
		{
		    return true;
		}
	    }
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		conduct_Move(move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, current_arrangement);
	    }
	    /*
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		if (   Occupation[depth][move->m_src_vrtx_id] != 0 || (depth < max_depth && Occupation[depth + 1][move->m_src_vrtx_id] != 0)
		    || Occupation[depth][move->m_dest_vrtx_id] != 0 || (depth < max_depth && Occupation[depth + 1][move->m_dest_vrtx_id] != 0))
		{
		    return true;
		}
	    }
	    */
	    ++depth;
	}	
	for (; depth < max_depth; ++depth)
	{
	    int N_Robots = current_arrangement.get_RobotCount();
	    for (int r_id = 1; r_id <= N_Robots; ++r_id)
	    {
		if (   (depth > 0 && Occupation[depth - 1][current_arrangement.get_RobotLocation(r_id)] != 0)
		    || Occupation[depth][current_arrangement.get_RobotLocation(r_id)] != 0
		    || (depth < max_depth - 1 && Occupation[depth + 1][current_arrangement.get_RobotLocation(r_id)] != 0))
		{
		    return true;
		}
	    }
	}
	return false;
    }


    sMultirobotSolver_DecomposedAstar::SolutionResult sMultirobotSolver_DecomposedAstar::resolve_GroupCollision(int group_index_A, int group_index_B, const Grouping_vector &robot_Grouping, const Instances_vector &group_Instances, Solutions_vector &group_Solutions)
    {
	Occupation_2d_vector Occupation_A;
	int N_Groups = group_Instances.size();

	int max_depth = 0;
	for (int g = 0; g < N_Groups; ++g)
	{
	    int depth = group_Solutions[g].get_StepCount();
	    if (depth > max_depth)
	    {
		max_depth = depth;
	    }
	}
	++max_depth;
	reset_Occupation(max_depth, m_multirobot_instance.m_environment.get_VertexCount(), Occupation_A);

	for (int g = 0; g < N_Groups; ++g)
	{
	    if (g != group_index_A)
	    {
		fill_Occupation(max_depth, group_Instances[g].m_initial_arrangement, group_Solutions[g], Occupation_A);
	    }
	}
	sMultirobotInstance group_subinstance_A;
	prepare_Subinstance(robot_Grouping[group_index_A], group_subinstance_A);

	setup_Subinstance(group_subinstance_A);

#ifdef sVERBOSE	
	{
	    printf("Solution of group %d\n", group_index_A);
	    group_Solutions[group_index_A].to_Screen();
	    printf("Solution of group %d\n", group_index_B);
	    group_Solutions[group_index_B].to_Screen();

	    printf("Occupation table complementary for group %d\n", group_index_A);
	    for (int i = 0; i < Occupation_A.size(); ++i)
	    {
		for (int j = 0; j < Occupation_A[i].size(); ++j)
		{
		    printf("%d ", Occupation_A[i][j]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
#endif

	sMultirobotSolution group_subsolution_A;
	switch (solve_Subinstance(group_subsolution_A, Occupation_A))
	{
	case RESULT_SOLVABLE:
	{
#ifdef sVERBOSE	
	    {
		printf("Alternative solution of group %d\n", group_index_A);
		group_subsolution_A.to_Screen();
	    }
#endif
	    group_Solutions[group_index_A] = group_subsolution_A;

	    if (!are_GroupsColliding(group_index_A, group_index_B, group_Instances, group_Solutions))
	    {
		return RESULT_SOLVABLE;
	    }
	    break;
	}
	case RESULT_UNSOLVABLE:
	{
	    break;
	}
	case RESULT_INDETERMINATE:
	{
	    return RESULT_INDETERMINATE;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	Occupation_2d_vector Occupation_B;
	reset_Occupation(max_depth, m_multirobot_instance.m_environment.get_VertexCount(), Occupation_B);

	for (int g = 0; g < N_Groups; ++g)
	{
	    if (g != group_index_B)
	    {
		fill_Occupation(max_depth, group_Instances[g].m_initial_arrangement, group_Solutions[g], Occupation_B);
	    }
	}

#ifdef sVERBOSE	
	{
	    printf("Occupation table complementary for group %d\n", group_index_B);
	    for (int i = 0; i < Occupation_B.size(); ++i)
	    {
		for (int j = 0; j < Occupation_B[i].size(); ++j)
		{
		    printf("%d ", Occupation_B[i][j]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
#endif
	    
	sMultirobotInstance group_subinstance_B;
	prepare_Subinstance(robot_Grouping[group_index_B], group_subinstance_B);
	
	setup_Subinstance(group_subinstance_B);
	
	sMultirobotSolution group_subsolution_B;
	switch (solve_Subinstance(group_subsolution_B, Occupation_B))
	{
	case RESULT_SOLVABLE:
	{
#ifdef sVERBOSE	
	    {
		printf("Alternative solution of group %d\n", group_index_B);
		group_subsolution_B.to_Screen();
	    }
#endif
	    group_Solutions[group_index_B] = group_subsolution_B;

	    if (!are_GroupsColliding(group_index_A, group_index_B, group_Instances, group_Solutions))
	    {
		return RESULT_SOLVABLE;
	    }
	    break;
	}
	case RESULT_UNSOLVABLE:
	{
	    return RESULT_UNSOLVABLE;
	}
	case RESULT_INDETERMINATE:
	{
	    return RESULT_INDETERMINATE;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	return RESULT_UNSOLVABLE;
    }


    void sMultirobotSolver_DecomposedAstar::combine_Subsolutions(const Grouping_vector &robot_Grouping, const Solutions_vector &group_Solutions, sMultirobotSolution &multirobot_solution)
    {
	int N_Groups = robot_Grouping.size();

	for (int g = 0; g < N_Groups; ++g)
	{
	    Robots_vector robot_mapping;
	    robot_mapping.push_back(0);

	    int r_id = 1;
	    const RobotGroup_set &robot_group = robot_Grouping[g];
	    for (RobotGroup_set::const_iterator robot = robot_group.begin(); robot != robot_group.end(); ++robot)
	    {
		robot_mapping.push_back(*robot);
		++r_id;
	    }

	    int time = 0;
	    const sMultirobotSolution::Steps_vector &group_solution_Steps = group_Solutions[g].m_Steps;	    
	    for (sMultirobotSolution::Steps_vector::const_iterator step = group_solution_Steps.begin();  step != group_solution_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    multirobot_solution.add_Move(time, sMultirobotSolution::Move(robot_mapping[move->m_robot_id], move->m_src_vrtx_id, move->m_dest_vrtx_id));
		}
		++time;
	    }
	}
    }


    void sMultirobotSolver_DecomposedAstar::prepare_Subinstance(const RobotGroup_set robot_group, sMultirobotInstance &group_subinstance) const
    {
	sRobotArrangement sub_initial_arrangement(m_multirobot_instance.m_initial_arrangement.get_VertexCount(), robot_group.size());
	sRobotGoal sub_robot_goal(m_multirobot_instance.m_initial_arrangement.get_VertexCount(), robot_group.size());

	int r_id = 1;

	for (RobotGroup_set::const_iterator robot = robot_group.begin(); robot != robot_group.end(); ++robot)
	{
	    sub_initial_arrangement.place_Robot(r_id, m_multirobot_instance.m_initial_arrangement.get_RobotLocation(*robot));
	    sub_robot_goal.charge_Robot(r_id, m_multirobot_instance.m_goal_specification.get_RobotGoal(*robot));
	    
	    ++r_id;
	}
	group_subinstance = sMultirobotInstance(m_multirobot_instance.m_environment, sub_initial_arrangement, sub_robot_goal);
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
