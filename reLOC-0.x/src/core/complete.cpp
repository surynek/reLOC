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
/* complete.cpp / 0.21-robik_020                                              */
/*----------------------------------------------------------------------------*/
//
// Fragments of complete polynomial time algorithms.
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
#include "complete.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{



/*----------------------------------------------------------------------------*/
// sCompleteSolver

    sCompleteSolver::Traversal::Traversal(int source_id, int target_id)
	: m_source_id(source_id)
	, m_target_id(target_id)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sCompleteSolver::Attempt::Attempt()
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sCompleteSolver::sCompleteSolver()
	: m_last_time_step(0)
	, m_undo_enabled(false)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    void sCompleteSolver::setup_Solver(void)
    {
	sMultirobotSolver::setup_Solver();
    }


    void sCompleteSolver::setup_Instance(const sMultirobotInstance &multirobot_instnace)
    {
	sMultirobotSolver::setup_Instance(multirobot_instnace);
	m_current_arrangement = m_multirobot_instance.m_initial_arrangement;

	m_last_time_step = 0;
    }


    bool sCompleteSolver::solve(void)
    {
//	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	VertexIDs_set locked_Vertices;
//	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();

	RobotIDs_vector sorted_Robots;
	search_BreadthFirst(m_multirobot_instance.m_goal_arrangement, sorted_Robots);
	/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sorted_Robots.push_back(robot_id);
	}
	*/
	for (RobotIDs_vector::const_iterator robot = sorted_Robots.begin(); robot != sorted_Robots.end(); ++robot)
	{
	    int robot_id = *robot;

	    int source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
	    int goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id);

	    if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
	    {
		return false;
	    }
	    locked_Vertices.insert(goal_vertex_id);
	    sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

	}
	return true;
    }


    static int all_neigh_lock = 0;
    static int some_neigh_unlock = 0;

    bool sCompleteSolver::solve_Twin(void)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	VertexIDs_set locked_Vertices;
//	int N_Robots = m_multirobot_instance.m_initial_arrangement.get_RobotCount();

	RobotIDs_vector sorted_Robots;
	search_BreadthFirst(m_multirobot_instance.m_goal_arrangement, sorted_Robots);
	/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sorted_Robots.push_back(robot_id);
	}
	*/
	for (RobotIDs_vector::const_iterator robot = sorted_Robots.begin(); robot != sorted_Robots.end();)
	{
	    int robot_id = *robot++;

	    if (robot != sorted_Robots.end())
	    {
		Attempt attempt_uni, attempt_twin, attempt_twin_2;
		int next_robot_id = *robot++;
			
		{ // TWIN
		    int source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		    int goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id);

		    int next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
		    int next_goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id);

		    enable_Attempt();
	    
		    sVertex *source_vertex = environment.get_Vertex(source_vertex_id);
		    bool all_neighbors_locked = true;
		    
		    for (sVertex::Neighbors_list::iterator neighbor = source_vertex->m_Neighbors.begin(); neighbor != source_vertex->m_Neighbors.end(); ++neighbor)
		    {
			sVertex *target = (*neighbor)->m_target;
			
			if (locked_Vertices.find(target->m_id) == locked_Vertices.end())
			{
			    all_neighbors_locked = false;
			    
			    VertexIDs_set twin_locked_Vertices = locked_Vertices;
			    twin_locked_Vertices.insert(source_vertex_id);

			    if (relocate_Robot(next_source_vertex_id, target->m_id, twin_locked_Vertices))
			    {
				if (m_current_arrangement.get_VertexOccupancy(target->m_id) == next_robot_id && m_current_arrangement.get_VertexOccupancy(source_vertex_id) == robot_id)
				{
				    if (!relocate_TwinRobots(source_vertex_id, target->m_id, goal_vertex_id, locked_Vertices))
				    {
					source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
					
					if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
					{
					    return false;
					}	
				    }
				    else
				    {
					printf(".");
				    }
				}
				else
				{
				    source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
				    
				    if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
				    {
					return false;
				    }		
				}
			    }
			    else
			    {
				source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
				
				if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
				{
				    return false;
				}
			    }
			    locked_Vertices.insert(goal_vertex_id);
			    sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

			    next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
			    
			    if (!relocate_Robot(next_source_vertex_id, next_goal_vertex_id, locked_Vertices))
			    {
				return false;
			    }
			    locked_Vertices.insert(next_goal_vertex_id);
			    sASSERT(m_current_arrangement.get_RobotLocation(next_robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id));
			    some_neigh_unlock++;
			    break;
			}
		    }
		    if (all_neighbors_locked)
		    {	
			++all_neigh_lock;
			source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);

			if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
			{
			    return false;
			}
			locked_Vertices.insert(goal_vertex_id);
			sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

			next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);

			if (!relocate_Robot(next_source_vertex_id, next_goal_vertex_id, locked_Vertices))
			{
			    return false;
			}
			locked_Vertices.insert(next_goal_vertex_id);
			sASSERT(m_current_arrangement.get_RobotLocation(next_robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id));
		    }
		    attempt_twin = m_movement_Attempts.back();
		    disable_Attempt();
		}

		{ // TWIN 2
		    int temp_robot_id = next_robot_id;
		    next_robot_id = robot_id;
		    robot_id = temp_robot_id;

		    int source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		    int goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id);

		    int next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
		    int next_goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id);

		    enable_Attempt();
	    
		    sVertex *source_vertex = environment.get_Vertex(source_vertex_id);
		    bool all_neighbors_locked = true;
		    
		    for (sVertex::Neighbors_list::iterator neighbor = source_vertex->m_Neighbors.begin(); neighbor != source_vertex->m_Neighbors.end(); ++neighbor)
		    {
			sVertex *target = (*neighbor)->m_target;
			
			if (locked_Vertices.find(target->m_id) == locked_Vertices.end())
			{
			    all_neighbors_locked = false;
			    
			    VertexIDs_set twin_locked_Vertices = locked_Vertices;
			    twin_locked_Vertices.insert(source_vertex_id);
			    

			    if (relocate_Robot(next_source_vertex_id, target->m_id, twin_locked_Vertices))
			    {
				if (m_current_arrangement.get_VertexOccupancy(target->m_id) == next_robot_id && m_current_arrangement.get_VertexOccupancy(source_vertex_id) == robot_id)
				{
				    if (!relocate_TwinRobots(source_vertex_id, target->m_id, goal_vertex_id, locked_Vertices))
				    {
					source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
					
					if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
					{
					    return false;
					}	
				    }
				}
				else
				{
				    source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
				    
				    if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
				    {
					return false;
				    }		
				}
			    }
			    else
			    {
				source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
				
				if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
				{
				    return false;
				}
			    }
			    locked_Vertices.insert(goal_vertex_id);
			    sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

			    next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
			    
			    if (!relocate_Robot(next_source_vertex_id, next_goal_vertex_id, locked_Vertices))
			    {
				return false;
			    }
			    locked_Vertices.insert(next_goal_vertex_id);
			    sASSERT(m_current_arrangement.get_RobotLocation(next_robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id));
			    some_neigh_unlock++;
			    break;
			}
		    }
		    if (all_neighbors_locked)
		    {	
			++all_neigh_lock;
			source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);

			if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
			{
			    return false;
			}
			locked_Vertices.insert(goal_vertex_id);
			sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

			next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);

			if (!relocate_Robot(next_source_vertex_id, next_goal_vertex_id, locked_Vertices))
			{
			    return false;
			}
			locked_Vertices.insert(next_goal_vertex_id);
			sASSERT(m_current_arrangement.get_RobotLocation(next_robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id));
		    }
		    attempt_twin_2 = m_movement_Attempts.back();
		    disable_Attempt();
		}

		{ // NO TWIN
		    int source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		    int goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id);

		    int next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
		    int next_goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id);

		    enable_Attempt();
		    
		    if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
		    {
			    return false;
		    }
		    locked_Vertices.insert(goal_vertex_id);
		    sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));

		    next_source_vertex_id = m_current_arrangement.get_RobotLocation(next_robot_id);
		    
		    if (!relocate_Robot(next_source_vertex_id, next_goal_vertex_id, locked_Vertices))
		    {
			return false;
		    }
		    locked_Vertices.insert(next_goal_vertex_id);
		    sASSERT(m_current_arrangement.get_RobotLocation(next_robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(next_robot_id));

		    attempt_uni = m_movement_Attempts.back();
		    disable_Attempt();
		}

		if (attempt_uni.m_attempt_Traversals.size() <= attempt_twin.m_attempt_Traversals.size())
		{
		    if (attempt_uni.m_attempt_Traversals.size() <= attempt_twin_2.m_attempt_Traversals.size())
		    {
			commit(attempt_uni);
		    }
		    else
		    {
			commit(attempt_twin_2);
		    }
		}
		else
		{
		    if (attempt_twin.m_attempt_Traversals.size() <= attempt_twin_2.m_attempt_Traversals.size())
		    {
			commit(attempt_twin);
		    }
		    else
		    {
			commit(attempt_twin_2);
		    }
		}
	    }
	    else
	    {
		int source_vertex_id = m_current_arrangement.get_RobotLocation(robot_id);
		int goal_vertex_id = m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id);

		if (!relocate_Robot(source_vertex_id, goal_vertex_id, locked_Vertices))
		{
		    return false;
		}
		locked_Vertices.insert(goal_vertex_id);
		sASSERT(m_current_arrangement.get_RobotLocation(robot_id) == m_multirobot_instance.m_goal_arrangement.get_RobotLocation(robot_id));
	    }
	}

	return true;
    }


    bool sCompleteSolver::relocate_Robot(int source_vertex_id, int goal_vertex_id, VertexIDs_set &locked_Vertices)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	if (source_vertex_id != goal_vertex_id)
	{
	    search_SingleSourcePaths(goal_vertex_id);
	    
	    sVertex *next = environment.get_Vertex(source_vertex_id);
	    
	    if (next->m_distance >= 0)
	    {
		int next_id = next->m_id;
		
		VertexIDs_vector path_vertex_IDs;
		
		do
		{
		    path_vertex_IDs.push_back(next_id);
		    next_id = next->m_prev_id;
		    next = environment.get_Vertex(next_id);
		}
		while (next_id != goal_vertex_id);
		path_vertex_IDs.push_back(next_id);
		/*
		for (VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin(); path_vertex != path_vertex_IDs.end(); ++path_vertex)
		{
.		    printf("%d ", *path_vertex);
		}
		printf("\n");
		*/
		sASSERT(path_vertex_IDs.size() >= 2);
		
		VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin();
		int path_vertex_id = *path_vertex++;
		int next_vertex_id;
		
		while (path_vertex != path_vertex_IDs.end())
		{
		    next_vertex_id = *path_vertex++;
		    
		    if (!push(path_vertex_id, next_vertex_id, locked_Vertices))
		    {
			if (!swap(path_vertex_id, next_vertex_id))
			{
			    return false;
			}
		    }
		    path_vertex_id = next_vertex_id;
		}
	    }
	}
	return true;
    }


    bool sCompleteSolver::relocate_TwinRobots(int first_source_vertex_id, int second_source_vertex_id, int goal_vertex_id, VertexIDs_set &locked_Vertices)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	if (first_source_vertex_id != goal_vertex_id)
	{
	    search_SingleSourcePaths(goal_vertex_id);
	    
	    sVertex *next = environment.get_Vertex(first_source_vertex_id);
	    
	    if (next->m_distance >= 0)
	    {
		int next_id = next->m_id;
		
		VertexIDs_vector path_vertex_IDs;
		
		do
		{
		    path_vertex_IDs.push_back(next_id);
		    next_id = next->m_prev_id;
		    next = environment.get_Vertex(next_id);
		}
		while (next_id != goal_vertex_id);
		path_vertex_IDs.push_back(next_id);
		/*
		printf("Path source -> goal: ");
		for (VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin(); path_vertex != path_vertex_IDs.end(); ++path_vertex)
		{
		    printf("%d ", *path_vertex);
		}
		printf("\n");
		*/
		sASSERT(path_vertex_IDs.size() >= 2);
		
		VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin();
		int path_vertex_id = *path_vertex++;
		int next_vertex_id;
		
		while (path_vertex != path_vertex_IDs.end())
		{
		    next_vertex_id = *path_vertex++;
		    
		    if (!push_Twin(path_vertex_id, second_source_vertex_id, next_vertex_id, locked_Vertices))
		    {
			if (!swap_Twin(path_vertex_id, second_source_vertex_id, next_vertex_id))
			{
			    return false;
			}
		    }
		    second_source_vertex_id = path_vertex_id;
		    path_vertex_id = next_vertex_id;
		}
	    }
	}
	return true;
    }


    bool sCompleteSolver::push(int source_id, int target_id, VertexIDs_set &locked_Vertices)
    {
	if (m_current_arrangement.get_VertexOccupancy(source_id) == 0)
	{
	    return true;
	}
	if (m_current_arrangement.get_VertexOccupancy(target_id) != 0)
	{
	    locked_Vertices.insert(source_id);

	    if (!clear_Vertex(target_id, locked_Vertices))
	    {
		return false;
	    }
	}
	make_Move(source_id, target_id);

	return true;
    }


    bool sCompleteSolver::push_Twin(int first_source_id, int second_source_id, int target_id, VertexIDs_set &locked_Vertices)
    {
	if (m_current_arrangement.get_VertexOccupancy(first_source_id) == 0 && m_current_arrangement.get_VertexOccupancy(second_source_id) == 0)
	{
	    return true;
	}
	if (m_current_arrangement.get_VertexOccupancy(target_id) != 0)
	{
	    locked_Vertices.insert(first_source_id);
	    locked_Vertices.insert(second_source_id);

	    if (!clear_Vertex(target_id, locked_Vertices))
	    {
		return false;
	    }
	}
	make_Move(first_source_id, target_id);
	make_Move(second_source_id, first_source_id);

	return true;
    }


    bool sCompleteSolver::multipush(int source_1_id, int source_2_id, int target_id, int &pretarget_id)
    {
	int first_vertex_id;
	int second_vertex_id;

	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	VertexIDs_set locked_Vertices;
	locked_Vertices.insert(source_2_id);
	search_SingleSourcePaths(target_id, locked_Vertices);

	if (environment.get_Vertex(source_1_id)->m_distance < 0)
	{
	    VertexIDs_set locked_Vertices;
	    locked_Vertices.insert(source_1_id);
	    search_SingleSourcePaths(target_id, locked_Vertices);

	    if (environment.get_Vertex(source_2_id)->m_distance < 0)
	    {
		return false;
	    }
	    else
	    {
		first_vertex_id = source_2_id;
		second_vertex_id = source_1_id;
	    }
	}
	else
	{
	    first_vertex_id = source_1_id;
	    second_vertex_id = source_2_id;
	}
/*
	if (environment.get_Vertex(source_1_id)->m_distance < environment.get_Vertex(source_2_id)->m_distance)
	{
	    first_vertex_id = source_1_id;
	    second_vertex_id = source_2_id;
	}
	else
	{
	    first_vertex_id = source_2_id;
	    second_vertex_id = source_1_id;
	}
*/
	if (first_vertex_id == target_id)
	{
	    pretarget_id = second_vertex_id;
	    return true;
	}

	sVertex *first_vertex = environment.get_Vertex(first_vertex_id);
	sVertex *previous = first_vertex;

	int previous_id = first_vertex->m_id;
	VertexIDs_vector path_vertex_IDs;

	previous_id = previous->m_prev_id;
	previous = environment.get_Vertex(previous_id);

	while (previous_id != target_id)
	{
	    path_vertex_IDs.push_back(previous_id);
	    previous_id = previous->m_prev_id;
	    previous = environment.get_Vertex(previous_id);
	}
	path_vertex_IDs.push_back(previous_id);
	/*
	printf("Path: ");
	for (VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin(); path_vertex != path_vertex_IDs.end(); ++path_vertex)
	{
	    printf("%d ", *path_vertex);
	}
	printf("\n");
	*/
	VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin();

	while (path_vertex != path_vertex_IDs.end())
	{
	    previous_id = *path_vertex;
	    previous = environment.get_Vertex(previous_id);

	    if (m_current_arrangement.get_VertexOccupancy(previous_id) != 0)
	    {
		VertexIDs_set locked_Vertices;
		locked_Vertices.insert(first_vertex->m_id);
		locked_Vertices.insert(second_vertex_id);

		if (!clear_Vertex(previous_id, locked_Vertices))
		{
		    return false;
		}
	    }
	    make_Move(first_vertex->m_id, previous_id);
	    make_Move(second_vertex_id, first_vertex->m_id);

	    second_vertex_id = first_vertex->m_id;
	    first_vertex = previous;

	    ++path_vertex;
	}
	pretarget_id = second_vertex_id;

	return true;
    }


    bool sCompleteSolver::multipush_Twin(int source_1_id, int source_2_id, int source_3_id, int target_id, int &pretarget_id, int &prepretarget_id)
    {
	int first_vertex_id;
	int second_vertex_id;
	int third_vertex_id;

	sUndirectedGraph &environment = m_multirobot_instance.m_environment;

	VertexIDs_set locked_Vertices;
	locked_Vertices.insert(source_2_id);
	locked_Vertices.insert(source_3_id);
	search_SingleSourcePaths(target_id, locked_Vertices);

	if (environment.get_Vertex(source_1_id)->m_distance < 0)
	{
	    VertexIDs_set locked_Vertices;
	    locked_Vertices.insert(source_2_id);
	    locked_Vertices.insert(source_1_id);
	    search_SingleSourcePaths(target_id, locked_Vertices);

	    if (environment.get_Vertex(source_3_id)->m_distance < 0)
	    {
		return false;
	    }
	    else
	    {
		first_vertex_id = source_3_id;
		second_vertex_id = source_2_id;
		third_vertex_id = source_1_id;
	    }
	}
	else
	{
	    first_vertex_id = source_1_id;
	    second_vertex_id = source_2_id;
	    third_vertex_id = source_3_id;
	}
/*
	if (environment.get_Vertex(source_1_id)->m_distance < environment.get_Vertex(source_3_id)->m_distance)
	{	    
	    first_vertex_id = source_1_id;
	    second_vertex_id = source_2_id;
	    third_vertex_id = source_3_id;
	}
	else
	{
	    first_vertex_id = source_3_id;
	    second_vertex_id = source_2_id;
	    third_vertex_id = source_1_id;
	}
*/

	if (first_vertex_id == target_id)
	{
	    pretarget_id = second_vertex_id;
	    prepretarget_id = third_vertex_id;
	    return true;
	}

	sVertex *first_vertex = environment.get_Vertex(first_vertex_id);
	sVertex *previous = first_vertex;

	int previous_id = first_vertex->m_id;
	VertexIDs_vector path_vertex_IDs;

	previous_id = previous->m_prev_id;
	previous = environment.get_Vertex(previous_id);

	while (previous_id != target_id)
	{
	    path_vertex_IDs.push_back(previous_id);
	    previous_id = previous->m_prev_id;
	    previous = environment.get_Vertex(previous_id);
	}
	path_vertex_IDs.push_back(previous_id);
	/*
	printf("Path: ");
	for (VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin(); path_vertex != path_vertex_IDs.end(); ++path_vertex)
	{
	    printf("%d ", *path_vertex);
	}
	printf("\n");
	*/
	VertexIDs_vector::const_iterator path_vertex = path_vertex_IDs.begin();

	while (path_vertex != path_vertex_IDs.end())
	{
	    previous_id = *path_vertex;
	    previous = environment.get_Vertex(previous_id);

	    if (m_current_arrangement.get_VertexOccupancy(previous_id) != 0)
	    {
		VertexIDs_set locked_Vertices;
		locked_Vertices.insert(first_vertex->m_id);
		locked_Vertices.insert(second_vertex_id);
		locked_Vertices.insert(third_vertex_id);

		if (!clear_Vertex(previous_id, locked_Vertices))
		{
		    return false;
		}
	    }
	    make_Move(first_vertex->m_id, previous_id);
	    make_Move(second_vertex_id, first_vertex->m_id);
	    make_Move(third_vertex_id, second_vertex_id);

	    third_vertex_id = second_vertex_id;
	    second_vertex_id = first_vertex->m_id;
	    first_vertex = previous;

	    ++path_vertex;
	}
	prepretarget_id = third_vertex_id;
	pretarget_id = second_vertex_id;

	return true;
    }


    bool sCompleteSolver::swap(int source_id, int target_id)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
//	int N_Vertices = environment.get_VertexCount();

	VertexIDs_vector sorted_Vertices;
	search_BreadthFirst(source_id, sorted_Vertices);

	//	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	//	{
	for (VertexIDs_vector::const_iterator sort_vertex = sorted_Vertices.begin(); sort_vertex != sorted_Vertices.end(); ++sort_vertex)
	{
	    int vertex_id = *sort_vertex;
	    sVertex *vertex = environment.get_Vertex(vertex_id);

	    if (vertex->m_Neighbors.size() >= 3)
	    {
		int pretarget_id;

		enable_Attempt();
		enable_Undo();
		if (multipush(source_id, target_id, vertex->m_id, pretarget_id))
		{
		    int first_vacant_id, second_vacant_id;
		    if (clear(pretarget_id, vertex->m_id, first_vacant_id, second_vacant_id))
		    {
			disable_Undo();
			exchange(pretarget_id, vertex->m_id, first_vacant_id, second_vacant_id);
			undo();
			commit();
			return true;
		    }
		    else
		    {
			disable_Undo();
			disable_Attempt();
		    }
		}
		else
		{
		    disable_Undo();
		    disable_Attempt();
		}
	    }
	}
	return false;
    }


    bool sCompleteSolver::swap_Twin(int first_source_id, int second_source_id, int target_id)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
//	int N_Vertices = environment.get_VertexCount();

	VertexIDs_vector sorted_Vertices;
	search_BreadthFirst(first_source_id, sorted_Vertices);

	int target_robot = m_current_arrangement.get_VertexOccupancy(target_id);

	//	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	//	{
	for (VertexIDs_vector::const_iterator sort_vertex = sorted_Vertices.begin(); sort_vertex != sorted_Vertices.end(); ++sort_vertex)
	{
	    int vertex_id = *sort_vertex;
	    sVertex *vertex = environment.get_Vertex(vertex_id);

	    if (vertex->m_Neighbors.size() >= 4)
	    {
		int pretarget_id, prepretarget_id;

		enable_Attempt();
		enable_Undo();
		if (multipush_Twin(target_id, first_source_id, second_source_id, vertex->m_id, pretarget_id, prepretarget_id))
		{
		    int first_vacant_id, second_vacant_id, third_vacant_id;
		    if (clear_Twin(pretarget_id, prepretarget_id, vertex->m_id, first_vacant_id, second_vacant_id, third_vacant_id))
		    {
			disable_Undo();

			if (m_current_arrangement.get_VertexOccupancy(vertex->m_id) == target_robot)
			{
			    exchange_Twin_1(pretarget_id, prepretarget_id, vertex->m_id, first_vacant_id, second_vacant_id, third_vacant_id);
			}
			else
			{
			    exchange_Twin_2(pretarget_id, prepretarget_id, vertex->m_id, first_vacant_id, second_vacant_id, third_vacant_id);
			}
			undo();
			commit();
			return true;
		    }
		    else
		    {
			disable_Undo();
			disable_Attempt();
		    }
		}
		else
		{
		    disable_Undo();
		    disable_Attempt();
		}
	    }
	}
	return false;
    }


    bool sCompleteSolver::clear(int pretarget_id, int vertex_id, int &first_vacant_id, int &second_vacant_id)
    {
	VertexIDs_set locked_Vertices;

	first_vacant_id = -1;
	second_vacant_id = -1;

	locked_Vertices.insert(pretarget_id);
	locked_Vertices.insert(vertex_id);

	int robot_s_id = m_current_arrangement.get_VertexOccupancy(pretarget_id);
	sASSERT(robot_s_id > 0);
	
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	sVertex *vertex = environment.get_Vertex(vertex_id);

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (m_current_arrangement.get_VertexOccupancy(target->m_id) == 0 && target->m_id != pretarget_id)
	    {
		if (first_vacant_id == -1)
		{
		    first_vacant_id = target->m_id;
		    locked_Vertices.insert(target->m_id);
		}
		else
		{
		    second_vacant_id = target->m_id;
		    return true;
		}
	    }
	}

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (locked_Vertices.find(target->m_id) == locked_Vertices.end())
	    {
//		sASSERT(m_current_arrangement.get_VertexOccupancy(target->m_id) != 0);

		if (clear_Vertex(target->m_id, locked_Vertices))
		{
		    if (first_vacant_id == -1)
		    {
			first_vacant_id = target->m_id;
			locked_Vertices.insert(target->m_id);
		    }
		    else
		    {
			second_vacant_id = target->m_id;
			return true;
		    }   
		}
	    }	    
	}
	if (first_vacant_id == -1)
	{
	    return false;
	}
	sASSERT(first_vacant_id != -1 && second_vacant_id == -1);

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (target->m_id != first_vacant_id && locked_Vertices.find(target->m_id) == locked_Vertices.end())
	    {
		enable_Attempt();

		VertexIDs_set first_locked_Vertices;
		first_locked_Vertices.insert(pretarget_id);
		first_locked_Vertices.insert(vertex_id);

		if (clear_Vertex(target->m_id, first_locked_Vertices))
		{
		    VertexIDs_set second_locked_Vertices = first_locked_Vertices;
		    second_locked_Vertices.insert(target->m_id);

		    if (clear_Vertex(first_vacant_id, second_locked_Vertices))
		    {
			commit();
			sASSERT(second_vacant_id == -1);

			if (second_vacant_id == -1)
			{
			    second_vacant_id = target->m_id;
			    return true;
			}
		    }
		    else
		    {
			disable_Attempt();
		    }
		}
		else
		{
		    disable_Attempt();
		}
	    }
	}

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (target->m_id != first_vacant_id && locked_Vertices.find(target->m_id) == locked_Vertices.end())
	    {
		enable_Attempt();

		make_Move(vertex_id, first_vacant_id);
		make_Move(pretarget_id, vertex_id);

		VertexIDs_set first_locked_Vertices;
		first_locked_Vertices.insert(vertex_id);
		first_locked_Vertices.insert(first_vacant_id);

		if (clear_Vertex(target->m_id, first_locked_Vertices))
		{
		    VertexIDs_set second_locked_Vertices = first_locked_Vertices;
		    second_locked_Vertices.insert(target->m_id);

		    if (clear_Vertex(first_vacant_id, second_locked_Vertices))
		    {
			commit();
			sASSERT(second_vacant_id == -1);

			if (second_vacant_id == -1)
			{
			    second_vacant_id = target->m_id;
			    return true;
			}
		    }
		    else
		    {
			disable_Attempt();
		    }
		}
		else
		{
		    disable_Attempt();
		}
	    }
	}
	VertexIDs_set first_locked_Vertices;
	first_locked_Vertices.insert(vertex_id);

	if (!clear_Vertex(pretarget_id, first_locked_Vertices))
	{
	    return false;
	}
	make_Move(vertex_id, pretarget_id);

	VertexIDs_set second_locked_Vertices = first_locked_Vertices;
	second_locked_Vertices.insert(pretarget_id);
	second_locked_Vertices.insert(m_current_arrangement.get_RobotLocation(robot_s_id));

	if (!clear_Vertex(first_vacant_id, second_locked_Vertices))
	{
	    return false;
	}
	int other_vertex_id = -1;

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (target->m_id != pretarget_id && target->m_id != first_vacant_id)
	    {
		other_vertex_id = target->m_id;
	    }
	}
	if (other_vertex_id == -1)
	{
	    return false;
	}
	sASSERT(other_vertex_id != -1);

#ifdef sDEBUG
	{
	    int other_robot_id = m_current_arrangement.get_VertexOccupancy(other_vertex_id);
	    sASSERT(other_robot_id != -1);
	}
#endif

	make_Move(other_vertex_id, vertex_id);
	make_Move(vertex_id, first_vacant_id);

	make_Move(pretarget_id, vertex_id);
	make_Move(m_current_arrangement.get_RobotLocation(robot_s_id), pretarget_id);

	VertexIDs_set third_locked_Vertices = first_locked_Vertices;
	third_locked_Vertices.insert(pretarget_id);
	third_locked_Vertices.insert(other_vertex_id);

	if (clear_Vertex(first_vacant_id, third_locked_Vertices))
	{
	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		sVertex *target = (*neighbor)->m_target;
		
		if (m_current_arrangement.get_VertexOccupancy(target->m_id) == 0 && target->m_id != first_vacant_id)
		{
		    second_vacant_id = target->m_id;
		    return true;
		}
	    }
	    return true;
	}
	return false;
    }


    bool sCompleteSolver::clear_Twin(int pretarget_id, int prepretarget_id, int vertex_id, int &first_vacant_id, int &second_vacant_id, int &third_vacant_id)
    {
	VertexIDs_set locked_Vertices;

	first_vacant_id = -1;
	second_vacant_id = -1;
	third_vacant_id = -1;

	locked_Vertices.insert(prepretarget_id);
	locked_Vertices.insert(pretarget_id);
	locked_Vertices.insert(vertex_id);

#ifdef sDEBUG
	{
	    int robot_s_id = m_current_arrangement.get_VertexOccupancy(pretarget_id);
	    sASSERT(robot_s_id > 0);
	}
#endif
	
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	sVertex *vertex = environment.get_Vertex(vertex_id);

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (m_current_arrangement.get_VertexOccupancy(target->m_id) == 0 && target->m_id != pretarget_id && target->m_id != prepretarget_id)
	    {
		if (first_vacant_id == -1)
		{
		    first_vacant_id = target->m_id;
		    locked_Vertices.insert(target->m_id);
		}
		else
		{
		    if (second_vacant_id == -1)
		    {
			second_vacant_id = target->m_id;
			locked_Vertices.insert(target->m_id);
		    }
		    else
		    {
			third_vacant_id = target->m_id;
			return true;
		    }
		}
	    }
	}

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (locked_Vertices.find(target->m_id) == locked_Vertices.end())
	    {
//		sASSERT(m_current_arrangement.get_VertexOccupancy(target->m_id) != 0);

		if (clear_Vertex(target->m_id, locked_Vertices))
		{
		    if (first_vacant_id == -1)
		    {
			first_vacant_id = target->m_id;
			locked_Vertices.insert(target->m_id);
		    }
		    else
		    {
			if (second_vacant_id == -1)
			{
			    second_vacant_id = target->m_id;
			    locked_Vertices.insert(target->m_id);
			}
			else
			{
			    third_vacant_id = target->m_id;
			    return true;
			}
		    }
		}
	    }	    
	}

	if (second_vacant_id == -1)
	{
	    return false;
	}
	sASSERT(first_vacant_id != -1 && second_vacant_id != -1 && third_vacant_id == -1);

	for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	{
	    sVertex *target = (*neighbor)->m_target;

	    if (target->m_id != first_vacant_id && target->m_id != second_vacant_id && locked_Vertices.find(target->m_id) == locked_Vertices.end())
	    {
		{
		    enable_Attempt();

		    VertexIDs_set first_locked_Vertices;
		    first_locked_Vertices.insert(target->m_id);
		    first_locked_Vertices.insert(vertex_id);
		    first_locked_Vertices.insert(first_vacant_id);
		    first_locked_Vertices.insert(pretarget_id);
		    first_locked_Vertices.insert(prepretarget_id);

		    make_Move(vertex_id, first_vacant_id);
		    make_Move(target->m_id, vertex_id);
		    make_Move(vertex_id, second_vacant_id);
		    
		    if (clear_Vertex(second_vacant_id, first_locked_Vertices))
		    {
			make_Move(first_vacant_id, vertex_id);
			commit();
			
			if (third_vacant_id == -1)
			{
			    third_vacant_id = target->m_id;
			    return true;
			}
		    }
		    else
		    {
			disable_Attempt();
		    }
		}
		{
		    enable_Attempt();

		    VertexIDs_set first_locked_Vertices;
		    first_locked_Vertices.insert(target->m_id);
		    first_locked_Vertices.insert(vertex_id);
		    first_locked_Vertices.insert(second_vacant_id);
		    first_locked_Vertices.insert(pretarget_id);
		    first_locked_Vertices.insert(prepretarget_id);

		    make_Move(vertex_id, second_vacant_id);
		    make_Move(target->m_id, vertex_id);
		    make_Move(vertex_id, first_vacant_id);
		    
		    if (clear_Vertex(first_vacant_id, first_locked_Vertices))
		    {
			make_Move(second_vacant_id, vertex_id);
			commit();
			
			if (third_vacant_id == -1)
			{
			    third_vacant_id = target->m_id;
			    return true;
			}
		    }
		    else
		    {
			disable_Attempt();
		    }
		}
	    }
	}
	return false;
    }


    void sCompleteSolver::exchange(int pretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id)
    {
	make_Move(vertex_id, first_vacant_id);
	make_Move(pretarget_id, vertex_id);
	make_Move(vertex_id, second_vacant_id);
	make_Move(first_vacant_id, vertex_id);
	make_Move(vertex_id, pretarget_id);
	make_Move(second_vacant_id, vertex_id);
    }


    void sCompleteSolver::exchange_Twin_1(int pretarget_id, int prepretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id, int third_vacant_id)
    {
	make_Move(vertex_id, first_vacant_id);
	make_Move(pretarget_id, vertex_id);
	make_Move(prepretarget_id, pretarget_id);
	make_Move(vertex_id, second_vacant_id);
	make_Move(pretarget_id, vertex_id);
	make_Move(vertex_id, third_vacant_id);

//--------

	make_Move(first_vacant_id, vertex_id);
	make_Move(vertex_id, pretarget_id);
	make_Move(pretarget_id, prepretarget_id);
	make_Move(third_vacant_id, vertex_id);
	make_Move(vertex_id, pretarget_id);
	make_Move(second_vacant_id, vertex_id);
    }


    void sCompleteSolver::exchange_Twin_2(int pretarget_id, int prepretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id, int third_vacant_id)
    {
	make_Move(vertex_id, first_vacant_id);
	make_Move(pretarget_id, vertex_id);
	make_Move(prepretarget_id, pretarget_id);
	make_Move(vertex_id, second_vacant_id);
	make_Move(pretarget_id, vertex_id);
	make_Move(vertex_id, third_vacant_id);

//--------

	make_Move(second_vacant_id, vertex_id);
	make_Move(vertex_id, pretarget_id);
	make_Move(pretarget_id, prepretarget_id);
	make_Move(first_vacant_id, vertex_id);
	make_Move(vertex_id, pretarget_id);
	make_Move(third_vacant_id, vertex_id);
    }


    bool sCompleteSolver::clear_Vertex(int source_id, VertexIDs_set &locked_Vertices)
    {
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	int N_Vertices = environment.get_VertexCount();

	search_SingleSourcePaths(source_id, locked_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sVertex *vertex = environment.get_Vertex(vertex_id);
	    if (m_current_arrangement.get_VertexOccupancy(vertex_id) == 0 && vertex->m_distance >= 0)
	    {
		int previous_id = vertex_id;
		sVertex *previous = environment.get_Vertex(previous_id);

		while (previous->m_id != source_id)
		{
		    if (m_current_arrangement.get_VertexOccupancy(previous->m_prev_id) != 0)
		    {
			make_Move(previous->m_prev_id, previous_id);
		    }
		    previous_id = previous->m_prev_id;
		    previous = environment.get_Vertex(previous_id);
		}
		return true;
	    }
	}
	return false;
    }


    void sCompleteSolver::search_SingleSourcePaths(int source_id)
    {
	VertexIDs_set empty_locked_Vertices;
	search_SingleSourcePaths(source_id, empty_locked_Vertices);
    }


    void sCompleteSolver::search_SingleSourcePaths(int source_id, VertexIDs_set &locked_Vertices)
    {
	VertexIDs_list vertex_Queue;
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	int N_Vertices = environment.get_VertexCount();

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sVertex *vertex = environment.get_Vertex(vertex_id);

	    vertex->m_distance = -1;
	    vertex->m_prev_id = -1;
	}

	if (locked_Vertices.find(source_id) == locked_Vertices.end())
	{
	    vertex_Queue.push_back(source_id);
	    sVertex *source = environment.get_Vertex(source_id);
	    source->m_distance = 0;
	    source->m_prev_id = -1;
	}
	while (!vertex_Queue.empty())
	{
	    int vertex_id = *vertex_Queue.begin();

	    vertex_Queue.pop_front();
	    sVertex *vertex = environment.get_Vertex(vertex_id);
	    
	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		sVertex *target = (*neighbor)->m_target;

		if (target->m_distance == -1)
		{
		    if (locked_Vertices.find(target->m_id) == locked_Vertices.end())
		    {
			target->m_distance = vertex->m_distance + 1;
			target->m_prev_id = vertex_id;

			vertex_Queue.push_back(target->m_id);
		    }
		}
	    }	    
	}
    }


    void sCompleteSolver::search_BreadthFirst(const sRobotArrangement &robot_arrangement, RobotIDs_vector &sorted_Robots)
    {
	VertexIDs_list vertex_Queue;
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	int N_Vertices = environment.get_VertexCount();

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sVertex *vertex = environment.get_Vertex(vertex_id);

	    vertex->m_distance = -1;
	    vertex->m_prev_id = -1;
	}

	int source_id = robot_arrangement.get_RobotLocation(1);
	
	vertex_Queue.push_back(source_id);
	sVertex *source = environment.get_Vertex(source_id);
	source->m_distance = 0;
	source->m_prev_id = -1;
	sorted_Robots.push_back(1);

	while (!vertex_Queue.empty())
	{
	    int vertex_id = *vertex_Queue.begin();

	    vertex_Queue.pop_front();
	    sVertex *vertex = environment.get_Vertex(vertex_id);
	    
	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		sVertex *target = (*neighbor)->m_target;

		if (target->m_distance == -1)
		{
		    int robot_id = robot_arrangement.get_VertexOccupancy(target->m_id);
		    if (robot_id >= 1)
		    {
			sorted_Robots.push_back(robot_id);
		    }
		    target->m_distance = vertex->m_distance + 1;
		    target->m_prev_id = vertex_id;

		    vertex_Queue.push_back(target->m_id);
		}
	    }	    
	}
    }


    void sCompleteSolver::search_BreadthFirst(int source_id, VertexIDs_vector &sorted_Vertices)
    {
	VertexIDs_list vertex_Queue;
	sUndirectedGraph &environment = m_multirobot_instance.m_environment;
	int N_Vertices = environment.get_VertexCount();

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sVertex *vertex = environment.get_Vertex(vertex_id);

	    vertex->m_distance = -1;
	    vertex->m_prev_id = -1;
	}

	vertex_Queue.push_back(source_id);
	sVertex *source = environment.get_Vertex(source_id);
	source->m_distance = 0;
	source->m_prev_id = -1;
	sorted_Vertices.push_back(source_id);

	while (!vertex_Queue.empty())
	{
	    int vertex_id = *vertex_Queue.begin();

	    vertex_Queue.pop_front();
	    sVertex *vertex = environment.get_Vertex(vertex_id);
	    
	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		sVertex *target = (*neighbor)->m_target;

		if (target->m_distance == -1)
		{
		    sorted_Vertices.push_back(target->m_id);

		    target->m_distance = vertex->m_distance + 1;
		    target->m_prev_id = vertex_id;

		    vertex_Queue.push_back(target->m_id);
		}
	    }	    
	}
    }


    void sCompleteSolver::undo(void)
    {
	m_undo_enabled = false;

	if (!m_movement_Attempts.empty())
	{
	    for (Traversals_vector::const_reverse_iterator traversal = m_movement_Attempts.back().m_undo_Traversals.rbegin(); traversal != m_movement_Attempts.back().m_undo_Traversals.rend(); ++traversal)
	    {
		make_Move(traversal->m_target_id, traversal->m_source_id);
	    }
	}
	else
	{
	    for (Traversals_vector::const_reverse_iterator traversal = m_undo_Traversals.rbegin(); traversal != m_undo_Traversals.rend(); ++traversal)
	    {
		make_Move(traversal->m_target_id, traversal->m_source_id);
	    }
	}
    }


    void sCompleteSolver::enable_Undo(void)
    {
	m_undo_enabled = true;
	if (!m_movement_Attempts.empty())
	{
	    m_movement_Attempts.back().m_undo_Traversals.clear();
	}
	else
	{
	    m_undo_Traversals.clear();
	}
    }


    void sCompleteSolver::disable_Undo(void)
    {
	m_undo_enabled = false;
    }


    void sCompleteSolver::commit(void)
    {
	sASSERT(!m_movement_Attempts.empty());

	m_current_arrangement = m_movement_Attempts.back().m_saved_arrangement;
	Traversals_vector attempt_Traversals = m_movement_Attempts.back().m_attempt_Traversals;
	Traversals_vector undo_Traversals = m_movement_Attempts.back().m_undo_Traversals;

	m_movement_Attempts.pop_back();

	for (Traversals_vector::const_iterator traversal = attempt_Traversals.begin(); traversal != attempt_Traversals.end(); ++traversal)
	{
	    make_Move_(traversal->m_source_id, traversal->m_target_id);
	}
	if (m_movement_Attempts.empty())
	{
	    for (Traversals_vector::const_iterator traversal = undo_Traversals.begin(); traversal != undo_Traversals.end(); ++traversal)
	    {
		m_undo_Traversals.push_back(*traversal);
	    }
	}
	else
	{
	    for (Traversals_vector::const_iterator traversal = undo_Traversals.begin(); traversal != undo_Traversals.end(); ++traversal)
	    {
		m_movement_Attempts.back().m_undo_Traversals.push_back(*traversal);
	    }
	}
    }


    void sCompleteSolver::commit(const Attempt &attempt)
    {
	m_current_arrangement = attempt.m_saved_arrangement;
	const Traversals_vector &attempt_Traversals = attempt.m_attempt_Traversals;

	for (Traversals_vector::const_iterator traversal = attempt_Traversals.begin(); traversal != attempt_Traversals.end(); ++traversal)
	{
	    make_Move_(traversal->m_source_id, traversal->m_target_id);
	}
    }


    void sCompleteSolver::enable_Attempt(void)
    {
	m_movement_Attempts.push_back(Attempt());
	m_movement_Attempts.back().m_saved_arrangement = m_current_arrangement;
    }


    void sCompleteSolver::disable_Attempt(void)
    {
	sASSERT(!m_movement_Attempts.empty());

	m_current_arrangement = m_movement_Attempts.back().m_saved_arrangement;
	m_movement_Attempts.pop_back();
    }


    void sCompleteSolver::make_Move(int source_id, int target_id)
    {
	make_Move_(source_id, target_id);

	if (m_undo_enabled)
	{
	    if (!m_movement_Attempts.empty())
	    {
		m_movement_Attempts.back().m_undo_Traversals.push_back(Traversal(source_id, target_id));
	    }
	    else
	    {
		m_undo_Traversals.push_back(Traversal(source_id, target_id));
	    }
	}
    }


    void sCompleteSolver::make_Move_(int source_id, int target_id)
    {
	sASSERT(m_current_arrangement.get_VertexOccupancy(target_id) == 0);

	if (!m_movement_Attempts.empty())
	{
	    m_movement_Attempts.back().m_attempt_Traversals.push_back(Traversal(source_id, target_id));
	    m_current_arrangement.move_Robot(m_current_arrangement.get_VertexOccupancy(source_id), target_id);
	}
	else
	{
	    record_Move(source_id, target_id);
	    m_current_arrangement.move_Robot(m_current_arrangement.get_VertexOccupancy(source_id), target_id);
	}
    }


    void sCompleteSolver::record_Move(int source_id, int target_id)
    {
	sASSERT(m_current_arrangement.get_VertexOccupancy(target_id) == 0);

	if (m_current_arrangement.get_VertexOccupancy(source_id) != 0)
	{
	    m_multirobot_solution.add_Move(m_last_time_step,
					   sMultirobotSolution::Move(m_current_arrangement.get_VertexOccupancy(source_id),
								     source_id,
								     target_id));
	    ++m_last_time_step;
	}
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
