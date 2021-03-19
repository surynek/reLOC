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
/* multirobot.cpp / 0.22-robik_071                                            */
/*----------------------------------------------------------------------------*/
//
// Multirobot coordinated path-finding solving package.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <map>

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


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sRobotArrangement

    sRobotArrangement::sRobotArrangement()
    {
	// nothing
    }
    

    sRobotArrangement::sRobotArrangement(int N_Vertices, int N_Robots, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    for (int i = 0; i < N_Vertices; ++i)
	    {
		Vertices.push_back(i);
	    }

	    int remain = N_Vertices;
	    for (int r = N_Robots; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sRobotArrangement::sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    int remain = 0;
	    for (int i = 0; i < N_Vertices; ++i)
	    {
		if (initial_arrangement.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }
	    for (int r = N_Robots; r >= 1;)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    sRobotArrangement::sRobotArrangement(const sRobotArrangement &initial_arrangement, int N_Vertices, int N_Robots, int N_fixed, bool random)
	: m_robot_Locs(N_Robots + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	if (random)
	{
	    Vertices_vector Vertices;
	    int remain = 0;

	    for (int i = 0; i < N_Vertices; ++i)
	    {
		if (initial_arrangement.get_VertexOccupancy(i) == 0)
		{
		    Vertices.push_back(i);
		    ++remain;
		}
	    }

	    int r = N_Robots;
	    for (int f = 0; f < N_fixed; ++f)
	    {
		place_Robot(r, initial_arrangement.get_RobotLocation(r));
		--r;
	    }
	    while (r >= 1)
	    {
		if (remain <= 0)
		{
		    break;
		}
		int rnd = rand() % remain;
		place_Robot(r--, Vertices[rnd]);

		Vertices[rnd] = Vertices[--remain];
	    }
	}
	else
	{
	    for (int i = 0; i <= N_Robots; ++i)
	    {
		m_robot_Locs[i] = UNDEFINED_LOCATION;
	    }
	}
    }


    bool sRobotArrangement::operator==(const sRobotArrangement &robot_arrangement) const
    {
	sASSERT(m_robot_Locs.size() == robot_arrangement.m_robot_Locs.size());

	Robots_vector::const_iterator robot_A, robot_B;
	
	for (robot_A = m_robot_Locs.begin(), robot_B = robot_arrangement.m_robot_Locs.begin();
	     robot_A != m_robot_Locs.end();
	     ++robot_A, ++robot_B)
	{
	    if (*robot_A != *robot_B)
	    {
		return false;
	    }
	}
	return true;
    }


    bool sRobotArrangement::operator<(const sRobotArrangement &robot_arrangement) const
    {
	sASSERT(m_robot_Locs.size() == robot_arrangement.m_robot_Locs.size());

	Robots_vector::const_iterator robot_A, robot_B;
	
	for (robot_A = m_robot_Locs.begin(), robot_B = robot_arrangement.m_robot_Locs.begin();
	     robot_A != m_robot_Locs.end();
	     ++robot_A, ++robot_B)
	{
	    if (*robot_A < *robot_B)
	    {
		return true;
	    }
	    else
	    {
		if (*robot_A > *robot_B)
		{
		    return false;
		}
	    }
	}

	return false;
    }


    int sRobotArrangement::get_RobotCount(void) const
    {
	return (m_robot_Locs.size() - 1);
    }

    
    int sRobotArrangement::get_VertexCount(void) const
    {
	return m_vertex_Occups.size();
    }
	

    int sRobotArrangement::get_RobotLocation(int robot_id) const
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());

	return m_robot_Locs[robot_id];
    }


    int sRobotArrangement::get_VertexOccupancy(int vertex_id) const
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	return m_vertex_Occups[vertex_id];
    }


    void sRobotArrangement::place_Robot(int robot_id, int vertex_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());
	sASSERT(vertex_id < m_vertex_Occups.size());
	sASSERT(m_vertex_Occups[vertex_id] == VACANT_VERTEX);
	
	m_robot_Locs[robot_id] = vertex_id;
	m_vertex_Occups[vertex_id] = robot_id;
    }


    void sRobotArrangement::place_CapacityRobot(int robot_id, int vertex_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());
	sASSERT(vertex_id < m_vertex_Occups.size());
	
	m_robot_Locs[robot_id] = vertex_id;
//	m_vertex_Occups[vertex_id] = robot_id;
    }    


    void sRobotArrangement::remove_Robot(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());

	m_vertex_Occups[m_robot_Locs[robot_id]] = VACANT_VERTEX;
	m_robot_Locs[robot_id] = UNDEFINED_LOCATION;
    }


    void sRobotArrangement::clean_Vertex(int vertex_id)
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	m_robot_Locs[m_vertex_Occups[vertex_id]] = UNDEFINED_LOCATION;
	m_vertex_Occups[vertex_id] = VACANT_VERTEX;
    }


    void sRobotArrangement::move_Robot(int robot_id, int dest_vertex_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Locs.size());
	sASSERT(dest_vertex_id < m_vertex_Occups.size());

//	printf("%d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);

	#ifdef sDEBUG
	{
	    /*
	    if (m_vertex_Occups[dest_vertex_id] != VACANT_VERTEX)
	    {
		printf("--> %d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);
		printf("Target occupied by: %d\n", m_vertex_Occups[dest_vertex_id]);
	    }
	    */
	    //sASSERT(m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX);
	}
        #endif

	m_vertex_Occups[m_robot_Locs[robot_id]] = VACANT_VERTEX;
	m_robot_Locs[robot_id] = dest_vertex_id;
	m_vertex_Occups[dest_vertex_id] = robot_id;

        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_move_Executions;
	}
	#endif
    }



    bool sRobotArrangement::is_VertexForced(int vertex_id) const
    {
	for (int r = 1; r < m_robot_Locs.size(); ++r)
	{
	    if (m_robot_Locs[r] == vertex_id)
	    {
		return true;
	    }
	}
	return false;
    }


    bool sRobotArrangement::is_VertexUnforced(int vertex_id) const
    {
	return !is_VertexForced(vertex_id);
    }    

    
    void sRobotArrangement::force_Robot(int robot_id, int dest_vertex_id)
    {       
	m_robot_Locs[robot_id] = dest_vertex_id;
    }


    bool sRobotArrangement::verify_Move(int robot_id, int dest_vertex_id, const sUndirectedGraph &graph) const
    {
	if (   (robot_id > 0 && robot_id < m_robot_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_robot_Locs[robot_id]] == robot_id)
	    && graph.is_Adjacent(m_robot_Locs[robot_id], dest_vertex_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d (adjacent:%d)\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id, graph.is_Adjacent(m_robot_Locs[robot_id], dest_vertex_id) ? 1 : 0);
	}
	#endif

	return false;
    }


    bool sRobotArrangement::verify_Move(int robot_id, int dest_vertex_id) const
    {
	if (   (robot_id > 0 && robot_id < m_robot_Locs.size())
	    && (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX)
	    && (m_vertex_Occups[m_robot_Locs[robot_id]] == robot_id))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d # %d -> %d\n", robot_id, m_robot_Locs[robot_id], dest_vertex_id);
	}
	#endif

	return false;
    }


    bool sRobotArrangement::check_Move(
#ifdef sDEBUG
	                               int robot_id,
#else
	                               int,
#endif
				       int dest_vertex_id) const
    {
	if (   (dest_vertex_id < m_vertex_Occups.size())
	    && (m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX))
	{
	    return true;
	}

	#ifdef sDEBUG
	{
	    printf("%d -> %d\n", robot_id, dest_vertex_id);
	}
	#endif

	return false;
    }


    void sRobotArrangement::generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment)
    {
	for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	{
	    const sVertex *vertex = environment.get_Vertex(initial_arrangement.get_RobotLocation(robot_id));
	    
	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }
	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Robot(robot_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


    void sRobotArrangement::generate_DisjointWalk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment)
    {
	for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	{
	    const sVertex *vertex = environment.get_Vertex(initial_arrangement.get_RobotLocation(robot_id));
	    
	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }
	    bool stand_in = true;
	    
	    for (int i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Robot(robot_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
		stand_in = false;
		
		if (initial_arrangement.get_RobotLocation(robot_id) == get_RobotLocation(robot_id))
		{
		    stand_in = true;
		}
	    }	    
	}
    }
    
    
    void sRobotArrangement::generate_NovelWalk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment)
    {
	sRobotArrangement robot_arrangement = initial_arrangement;

	bool stand_in = false;
	
	for (int i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	{	    
	    for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	    {
		const sVertex *vertex = environment.get_Vertex(robot_arrangement.get_RobotLocation(robot_id));

		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;

		    if (robot_arrangement.get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		    {
			robot_arrangement.move_Robot(robot_id, vertex->m_id);
		    }
		}
	    }
	    /*
	    stand_in = false;
	    for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	    {
		if (initial_arrangement.get_RobotLocation(robot_id) == robot_arrangement.get_RobotLocation(robot_id))
		{
		    stand_in = true;
		    break;
		}
	    }
	    */
	}
	*this = robot_arrangement;
    }


    sResult sRobotArrangement::generate_Nonconflicting(int N_Vertices, int N_Robots, const sUndirectedGraph &environment)
    {
	m_robot_Locs.resize(N_Robots + 1);
	m_vertex_Occups.resize(N_Vertices);

	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
	
	Vertices_vector Vertices;
	int remain = 0;
	
	for (int i = 0; i < N_Vertices; ++i)
	{
	    Vertices.push_back(i);
	    ++remain;
	}	
	int r = N_Robots;
	
	while (r >= 1)
	{
	    if (remain <= 0)
	    {
		return -1;
	    }
	    int rnd = rand() % remain;
	    int pos = Vertices[rnd];
	    place_Robot(r, Vertices[rnd]);
	    Vertices[rnd] = Vertices[--remain];

	    sVertex::VertexIDs_vector Conflicts = environment.m_Vertices[pos].m_Conflicts;
	    for (sVertex::VertexIDs_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
	    {
		for (int i = 0; i < remain; ++i)
		{
		    if (Vertices[i] == *conflict)
		    {
			Vertices[i] = Vertices[--remain];
			break;
		    }
		}
	    }
	    --r;
	}
	
	return sRESULT_SUCCESS;
    }
    
    
    void sRobotArrangement::generate_NovelNonconflictingWalk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment)
    {
	sRobotArrangement robot_arrangement = initial_arrangement;

	bool stand_in = false;
	
	for (int i = 0; i < RANDOM_WALK_LENGTH || stand_in; ++i)
	{	    
	    for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	    {
		const sVertex *vertex = environment.get_Vertex(robot_arrangement.get_RobotLocation(robot_id));

		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;

		    if (robot_arrangement.get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		    {
			bool non_conflict = true;
			sVertex::VertexIDs_vector Conflicts = environment.m_Vertices[vertex->m_id].m_Conflicts;
			for (sVertex::VertexIDs_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
			{
			    if (robot_arrangement.get_VertexOccupancy(*conflict) != VACANT_VERTEX && robot_arrangement.get_VertexOccupancy(*conflict) != robot_id)
			    {
				non_conflict = false;
				break;
			    }
			}

			if (non_conflict)
			{			    
			    robot_arrangement.move_Robot(robot_id, vertex->m_id);
			}
		    }
		}
	    }
	    /*
	    stand_in = false;
	    for (int robot_id = 1; robot_id < m_robot_Locs.size(); ++robot_id)
	    {
		if (initial_arrangement.get_RobotLocation(robot_id) == robot_arrangement.get_RobotLocation(robot_id))
		{
		    stand_in = true;
		    break;
		}
	    }
	    */
	}
	*this = robot_arrangement;
    }        


    void sRobotArrangement::generate_Walk(const sRobotArrangement &initial_arrangement, const sUndirectedGraph &environment, int N_fixed)
    {
	for (int robot_id = 1; robot_id < N_fixed; ++robot_id)
	{
	    place_Robot(robot_id, initial_arrangement.get_RobotLocation(robot_id));
	}
	for (int robot_id = N_fixed; robot_id < m_robot_Locs.size(); ++robot_id)
	{
	    const sVertex *vertex = environment.get_Vertex(initial_arrangement.get_RobotLocation(robot_id));

	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    break;
		}
	    }

	    for (int i = 0; i < RANDOM_WALK_LENGTH; ++i)
	    {
		if (get_VertexOccupancy(vertex->m_id) == VACANT_VERTEX)
		{
		    place_Robot(robot_id, vertex->m_id);
		    break;
		}
		if (!vertex->m_Neighbors.empty())
		{
		    int rn = rand() % vertex->m_Neighbors.size();

		    sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin();
		    for (int n = 0; n < rn; ++n)
		    {
			++neighbor;
		    }
		    vertex = (*neighbor)->m_target;
		}
		else
		{
		    sASSERT(false);
		    break;
		}
	    }
	}
    }


  void sRobotArrangement::generate_Equidistant(const sRobotArrangement &initial_arrangement, sUndirectedGraph &environment, int distance)
  {
    int N_Robots = initial_arrangement.get_RobotCount();
    int N_Vertices = environment.get_VertexCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	  int s_id = initial_arrangement.get_RobotLocation(robot_id);

	  VertexIDs_vector equidistant_IDs, free_equidistant_IDs;
	  environment.collect_EquidistantVertices(s_id, distance, equidistant_IDs);

	  for (VertexIDs_vector::const_iterator equidistant = equidistant_IDs.begin(); equidistant != equidistant_IDs.end(); ++equidistant)
	    {
	      if (get_VertexOccupancy(*equidistant) == VACANT_VERTEX)
		{
		  free_equidistant_IDs.push_back(*equidistant);
		}
	    }
	  if (free_equidistant_IDs.empty())
	    {
	      VertexIDs_vector free_vertex_IDs;

	      for (int i = 0; i < N_Vertices; ++i)
		{
		  if (get_VertexOccupancy(i) == VACANT_VERTEX)
		    {
		      free_vertex_IDs.push_back(i);
		    }
		}
	      int rnd = rand() % free_vertex_IDs.size();
	      place_Robot(robot_id, free_vertex_IDs[rnd]);
	    }
	  else
	    {
	      int rnd = rand() % free_equidistant_IDs.size();
	      place_Robot(robot_id, free_equidistant_IDs[rnd]);
	    }
	}
  }


/*----------------------------------------------------------------------------*/

    void sRobotArrangement::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sRobotArrangement::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot arrangement: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Locs.size() - 1, m_vertex_Occups.size());
	fprintf(fw, "%s%s robot locations: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Locs.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    fprintf(fw, "%d#%d ", i, m_robot_Locs[i]);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s%s vertex occupancy: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "%d#%d ", m_vertex_Occups[i], i);
	}
	fprintf(fw, "}\n");

	if (!m_robot_Sizes.empty())
	{
	    fprintf(fw, "%s%s robot sizes: {", indent.c_str(), sRELOC_INDENT.c_str());
	    for (int i = 1; i < N_Robots_1; ++i)
	    {
		fprintf(fw, "%d(%d) ", i, m_robot_Sizes[i]);
	    }
	    fprintf(fw, "}\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sRobotArrangement::to_Screen_brief(const sString &indent) const
    {
	to_Stream_brief(stdout, indent);
    }


    void sRobotArrangement::to_Stream_brief(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot arrangement (brief): (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Locs.size() - 1, m_vertex_Occups.size());
	fprintf(fw, "%s%s robot locations: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Locs.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    fprintf(fw, "%d#%d ", i, m_robot_Locs[i]);
	}
	fprintf(fw, "}\n");

	fprintf(fw, "%s]\n", indent.c_str());
    }    


    sResult sRobotArrangement::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotArrangement::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1)[%d:-1:-1]\n", i, m_vertex_Occups[i]);
	}
    }


    sResult sRobotArrangement::from_File_multirobot(const sString &filename, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }

    
    sResult sRobotArrangement::from_Stream_multirobot(FILE *fr, int component)
    {
	m_robot_Locs.clear();
	m_vertex_Occups.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		++N_Robots;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
//	    printf("read: %d,%d,%d\n", vertex_id, cycle_id, robot_id);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_ARRANGEMENT_SEEK_ERROR;
	}
	c = fgetc(fr);

	int undefined_location = UNDEFINED_LOCATION;
	m_robot_Locs.resize(N_Robots + 1, undefined_location);

	int vacant_vertex = VACANT_VERTEX;	
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[%d", &vertex_id, &cycle_id, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		m_robot_Locs[robot_id] = vertex_id;
		m_vertex_Occups[vertex_id] = robot_id;
	    }
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sRobotArrangement::to_File_capacitated_multirobot(const sString &filename, const sUndirectedGraph &environment, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	to_Stream_capacitated_multirobot(fw, environment, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotArrangement::to_Stream_capacitated_multirobot(FILE *fw, const sUndirectedGraph &environment, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    fprintf(fw, "(%d:-1:%d)[%d:-1:-1]\n", i, environment.m_Vertices[i].m_capacity, m_vertex_Occups[i]);
	}
    }


    sResult sRobotArrangement::from_File_capacitated_multirobot(const sString &filename, sUndirectedGraph &environment, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_ARRANGEMENT_OPEN_ERROR;
	}
	
	result = from_Stream_capacitated_multirobot(fr, environment, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }

    
    sResult sRobotArrangement::from_Stream_capacitated_multirobot(FILE *fr, sUndirectedGraph &environment, int component)
    {
	m_robot_Locs.clear();
	m_vertex_Occups.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[%d", &vertex_id, &cycle_id, &capacity, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d:%d)[%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		++N_Robots;
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
//	    printf("read: %d,%d,%d\n", vertex_id, cycle_id, robot_id);
	}

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_ARRANGEMENT_SEEK_ERROR;
	}
	c = fgetc(fr);

	int undefined_location = UNDEFINED_LOCATION;
	m_robot_Locs.resize(N_Robots + 1, undefined_location);
	int vacant_vertex = VACANT_VERTEX;
	m_vertex_Occups.resize(N_Vertices, vacant_vertex);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[%d", &vertex_id, &cycle_id, &capacity, &robot_id);
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d:%d)[%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &robot_id);
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d:%d)[%d:%d:%d", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &dummy_robot_2_id, &robot_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (robot_id > 0)
	    {
		m_robot_Locs[robot_id] = vertex_id;
		m_vertex_Occups[vertex_id] = robot_id;
	    }
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	    environment.m_Vertices[vertex_id].m_capacity = capacity;
	}

	return sRESULT_SUCCESS;
    }    


    

/*----------------------------------------------------------------------------*/
// sRobotGoal

    sRobotGoal::sRobotGoal()
    {
	// nothing
    }
    

    sRobotGoal::sRobotGoal(int N_Vertices, int N_Robots)
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	// nothing
    }


    sRobotGoal::sRobotGoal(int N_Vertices, int N_Robots, int N_Goals)
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	for (int i = 1; i <= N_Robots; ++i)
	{
	    int rnd_N_goals = 1 + rand() % N_Goals;

	    sRobotArrangement::Vertices_vector vertex_IDs;

	    for (int j = 0; j < N_Vertices; ++j)
	    {
		vertex_IDs.push_back(j);
	    }
	    for (int j = 0; j < rnd_N_goals; ++j)
	    {
		int rnd_vertex_ID = rand() % vertex_IDs.size();
		charge_Robot(i, rnd_vertex_ID);
		vertex_IDs[rnd_vertex_ID] = *vertex_IDs.rbegin();
		vertex_IDs.pop_back();
	    }
	}
    }


    sRobotGoal::sRobotGoal(const sRobotArrangement &sUNUSED(initial_arrangement), int N_Vertices, int N_Robots, int sUNUSED(N_Goals))
	: m_robot_Goals(N_Robots + 1)
	, m_goal_Compats(N_Vertices)
    {
	sASSERT(false);
    }


    sRobotGoal::sRobotGoal(const sRobotArrangement &robot_arrangement)
	: m_robot_Goals(robot_arrangement.get_RobotCount() + 1)
	, m_goal_Compats(robot_arrangement.get_VertexCount())
    {
	int N_Vertices = robot_arrangement.get_VertexCount();

	for (int i = 0; i < N_Vertices; ++i)
	{
	    int robot_id = robot_arrangement.get_VertexOccupancy(i);
	    if (robot_id > 0)
	    {
		assign_Goal(i, robot_id);
	    }
	}
    }


    int sRobotGoal::get_RobotCount(void) const
    {
	return (m_robot_Goals.size() - 1);
    }


    int sRobotGoal::get_VertexCount(void) const
    {
	return m_goal_Compats.size();
    }

    
    const sRobotGoal::Vertices_set& sRobotGoal::get_RobotGoal(int robot_id) const
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	return m_robot_Goals[robot_id];
    }


    sRobotGoal::Vertices_set& sRobotGoal::provide_RobotGoal(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	return m_robot_Goals[robot_id];
    }


    const sRobotGoal::Robots_set& sRobotGoal::get_GoalCompatibility(int goal_id) const
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	return m_goal_Compats[goal_id];
    }


    sRobotGoal::Robots_set& sRobotGoal::provide_GoalCompatibility(int goal_id)
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	return m_goal_Compats[goal_id];
    }


    void sRobotGoal::charge_Robot(int robot_id, int goal_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_robot_Goals[robot_id].insert(goal_id);
	m_goal_Compats[goal_id].insert(robot_id);
    }


    void sRobotGoal::charge_Robot(int robot_id, const Vertices_set &goal_IDs)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	m_robot_Goals[robot_id].insert(goal_IDs.begin(), goal_IDs.end());

	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    sASSERT(*goal_id >= 0 && *goal_id < m_goal_Compats.size());
	    m_goal_Compats[*goal_id].insert(robot_id);
	}
    }


    void sRobotGoal::assign_Goal(int goal_id, int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_goal_Compats[goal_id].insert(robot_id);
	m_robot_Goals[robot_id].insert(goal_id);
    }


    void sRobotGoal::assign_Goal(int goal_id, const Robots_set &robot_IDs)
    {
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_goal_Compats[goal_id].insert(robot_IDs.begin(), robot_IDs.end());

	for (Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
	{
	    sASSERT(*robot_id > 0 && *robot_id < m_robot_Goals.size());
	    m_robot_Goals[*robot_id].insert(goal_id);
	}
    }


    void sRobotGoal::discharge_Robot(int robot_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	const Vertices_set &goal_IDs = m_robot_Goals[robot_id];
	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    m_goal_Compats[*goal_id].erase(robot_id);
	}
	m_robot_Goals[robot_id].clear();
    }


    void sRobotGoal::discharge_Robot(int robot_id, int goal_id)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());
	sASSERT(goal_id >= 0 && goal_id < m_goal_Compats.size());

	m_robot_Goals[robot_id].erase(goal_id);
	m_goal_Compats[goal_id].erase(robot_id);
    }


    void sRobotGoal::discharge_Robot(int robot_id, const Vertices_set &goal_IDs)
    {
	sASSERT(robot_id > 0 && robot_id < m_robot_Goals.size());

	for (Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
	{
	    m_goal_Compats[*goal_id].erase(robot_id);
	}
	m_robot_Goals[robot_id].erase(goal_IDs.begin(), goal_IDs.end());
    }


    bool sRobotGoal::is_Satisfied(const sRobotArrangement &robot_arrangement) const
    {
	int N_Robots = robot_arrangement.get_RobotCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    const Vertices_set &robot_goal = get_RobotGoal(robot_id);
	    if (robot_goal.find(robot_arrangement.get_RobotLocation(robot_id)) == robot_goal.end())
	    {
		return false;
	    }
	}
	return true;
    }


/*----------------------------------------------------------------------------*/

    void sRobotGoal::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sRobotGoal::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot goal: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Goals.size() - 1, m_goal_Compats.size());
	fprintf(fw, "%s%srobot goals: {\n", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Goals.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    const Vertices_set &goal_IDs = m_robot_Goals[i];
	    fprintf(fw, "%s%s%s%d#{", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), i);
	    if (!goal_IDs.empty())
	    {
		Vertices_set::const_iterator goal_id = goal_IDs.begin();

		fprintf(fw, "%d", *goal_id);
		while (++goal_id != goal_IDs.end())
		{
		    fprintf(fw, ",%d", *goal_id);
		}
	    }
	    fprintf(fw, "}\n");
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%svertex compatibilities: {\n", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Vertices = m_goal_Compats.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    const Robots_set &robot_IDs = m_goal_Compats[i];
	    fprintf(fw, "%s%s%s%d@{", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), i);
	    if (!robot_IDs.empty())
	    {
		Robots_set::const_iterator robot_id = robot_IDs.begin();

		fprintf(fw, "%d", *robot_id);
		while (++robot_id != robot_IDs.end())
		{
		    fprintf(fw, ",%d", *robot_id);
		}
	    }
	    fprintf(fw, "}\n");
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sRobotGoal::to_Screen_brief(const sString &indent) const
    {
	to_Stream_brief(stdout, indent);
    }


    void sRobotGoal::to_Stream_brief(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sRobot goal (brief): (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_robot_Goals.size() - 1, m_goal_Compats.size());
	fprintf(fw, "%s%srobot goals: {\n", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Robots_1 = m_robot_Goals.size();
	for (int i = 1; i < N_Robots_1; ++i)
	{
	    const Vertices_set &goal_IDs = m_robot_Goals[i];
	    fprintf(fw, "%s%s%s%d#{", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), i);
	    if (!goal_IDs.empty())
	    {
		Vertices_set::const_iterator goal_id = goal_IDs.begin();

		fprintf(fw, "%d", *goal_id);
		while (++goal_id != goal_IDs.end())
		{
		    fprintf(fw, ",%d", *goal_id);
		}
	    }
	    fprintf(fw, "}\n");
	}
	fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s]\n", indent.c_str());
    }    


    sResult sRobotGoal::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotGoal::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Compats = m_goal_Compats.size();

	for (int i = 0; i < N_Compats; ++i)
	{
	    const Robots_set &robot_IDs = m_goal_Compats[i];
	    fprintf(fw, "(%d:-1)[", i);
	    to_Stream_multirobot(fw, robot_IDs, indent);
	    fprintf(fw, ":-1:-1]\n");
	}
    }


    void sRobotGoal::to_Stream_multirobot(FILE *fw, const Robots_set &robot_IDs, const sString &sUNUSED(indent)) const
    {
	fprintf(fw, "{");
	if (!robot_IDs.empty())
	{
	    Robots_set::const_iterator robot_id = robot_IDs.begin();
	    
	    fprintf(fw, "%d", *robot_id);
	    while (++robot_id != robot_IDs.end())
	    {
		fprintf(fw, ",%d", *robot_id);
	    }
	}
	fprintf(fw, "}");
    }


    sResult sRobotGoal::from_File_multirobot(const sString &filename, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::from_Stream_multirobot(FILE *fr, int component)
    {
	Robots_set all_robot_IDs;

	m_robot_Goals.clear();
	m_goal_Compats.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id = 0;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[", &vertex_id, &cycle_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:", &vertex_id, &cycle_id, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}
	N_Robots = all_robot_IDs.size();

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_GOAL_SEEK_ERROR;
	}
	c = fgetc(fr);

	m_robot_Goals.resize(N_Robots + 1);
	m_goal_Compats.resize(N_Vertices);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d)[", &vertex_id, &cycle_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d)[%d:", &vertex_id, &cycle_id, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d)[%d:%d:", &vertex_id, &cycle_id, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);
		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::from_Stream_multirobot(FILE *fr, Robots_set &robot_IDs)
    {
	fscanf(fr, "{");

	int robot_ID;
	int c = fgetc(fr);

	while (c != '}')
	{
	    if (c != ',')
	    {
		ungetc(c, fr);
	    }
	    fscanf(fr, "%d", &robot_ID);
	    robot_IDs.insert(robot_ID);

	    c = fgetc(fr);
	}
	fscanf(fr, "}");

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::to_File_capacitated_multirobot(const sString &filename, const sUndirectedGraph &environment, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	to_Stream_capacitated_multirobot(fw, environment, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sRobotGoal::to_Stream_capacitated_multirobot(FILE *fw, const sUndirectedGraph &environment, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Compats = m_goal_Compats.size();

	for (int i = 0; i < N_Compats; ++i)
	{
	    const Robots_set &robot_IDs = m_goal_Compats[i];
	    fprintf(fw, "(%d:-1:%d)[", i, environment.m_Vertices[i].m_capacity);
	    to_Stream_multirobot(fw, robot_IDs, indent);
	    fprintf(fw, ":-1:-1]\n");
	}
    }


    sResult sRobotGoal::from_File_capacitated_multirobot(const sString &filename, sUndirectedGraph &environment, int component)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sROBOT_GOAL_OPEN_ERROR;
	}
	
	result = from_Stream_capacitated_multirobot(fr, environment, component);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sRobotGoal::from_Stream_capacitated_multirobot(FILE *fr, sUndirectedGraph &environment, int component)
    {
	Robots_set all_robot_IDs;

	m_robot_Goals.clear();
	m_goal_Compats.clear();

	int N_Robots = 0;
	int N_Vertices = 0;

	int c = fgetc(fr);

	while (c != 'V')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");

	long position = ftell(fr);
	c = fgetc(fr);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id = 0, capacity = 1;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[", &vertex_id, &cycle_id, &capacity);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d:%d)[%d:", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d:%d)[%d:%d:", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    all_robot_IDs.insert(robot_IDs.begin(), robot_IDs.end());
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			all_robot_IDs.insert(robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    ++N_Vertices;
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}
	N_Robots = all_robot_IDs.size();

	if (fseek(fr, position, SEEK_SET) != 0)
	{
	    return sROBOT_GOAL_SEEK_ERROR;
	}
	c = fgetc(fr);

	m_robot_Goals.resize(N_Robots + 1);
	m_goal_Compats.resize(N_Vertices);

	while (c == '(')
	{
	    int vertex_id, cycle_id, robot_id, capacity;

	    switch (component)
	    {
	    case 0:
	    {
		fscanf(fr, "%d:%d:%d)[", &vertex_id, &cycle_id, &capacity);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 1:
	    {
		int dummy_robot_1_id;
		fscanf(fr, "%d:%d:%d)[%d:", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);

		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    case 2:
	    {
		int dummy_robot_1_id, dummy_robot_2_id;
		fscanf(fr, "%d:%d:%d)[%d:%d:", &vertex_id, &cycle_id, &capacity, &dummy_robot_1_id, &dummy_robot_2_id);

		c = fgetc(fr);
		ungetc(c, fr);

		if (c == '{')
		{
		    Robots_set robot_IDs;
		    from_Stream_multirobot(fr, robot_IDs);
		    if (!robot_IDs.empty())
		    {
			assign_Goal(vertex_id, robot_IDs);
		    }
		}
		else
		{
		    fscanf(fr, "%d", &robot_id);
		    if (robot_id > 0)
		    {
			assign_Goal(vertex_id, robot_id);
		    }
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    environment.m_Vertices[vertex_id].m_capacity = capacity;

	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }



    
/*----------------------------------------------------------------------------*/
// sMultirobotEncodingContext_CNFsat

    const int sMultirobotEncodingContext_CNFsat::UNDEFINED_LAYER_COUNT = -1;


/*----------------------------------------------------------------------------*/

    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat()
	: m_N_Layers(UNDEFINED_LAYER_COUNT)
	, m_max_total_cost(0)
	, m_extra_cost(-1)
	, m_max_total_fuel(0)
	, m_extra_fuel(-1)
	, m_fuel_makespan(-1)
	, m_state_clause_generator(NULL)
	, m_advanced_clause_generator(NULL)
	, m_bitwise_clause_generator(NULL)
	, m_bit_clause_generator(NULL)
    {
	m_state_Identifiers[sINT_32_MAX] = NULL;
	m_bit_Identifiers[sINT_32_MAX] = NULL;
	switchTo_StandardGeneratingMode();
    }


    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat(int N_Layers)
	: m_N_Layers(N_Layers)
	, m_max_total_cost(0)
	, m_extra_cost(-1)
	, m_max_total_fuel(0)
	, m_extra_fuel(-1)
	, m_fuel_makespan(-1)
	, m_state_clause_generator(&m_variable_store)
	, m_advanced_clause_generator(&m_variable_store)
	, m_bitwise_clause_generator(&m_variable_store)
	, m_bit_clause_generator(&m_variable_store)
    {
	m_state_Identifiers[sINT_32_MAX] = NULL;
	m_bit_Identifiers[sINT_32_MAX] = NULL;
	switchTo_StandardGeneratingMode();
    }


    sMultirobotEncodingContext_CNFsat::sMultirobotEncodingContext_CNFsat(const sMultirobotEncodingContext_CNFsat &encoding_context)
	: m_N_Layers(encoding_context.m_N_Layers)
	, m_max_total_cost(encoding_context.m_max_total_cost)
	, m_extra_cost(encoding_context.m_extra_cost)
	, m_max_total_fuel(encoding_context.m_max_total_fuel)
	, m_extra_fuel(encoding_context.m_extra_fuel)
	, m_fuel_makespan(encoding_context.m_fuel_makespan)
	, m_variable_store(encoding_context.m_variable_store)
	, m_state_clause_generator(encoding_context.m_state_clause_generator)
	, m_advanced_clause_generator(encoding_context.m_advanced_clause_generator)
	, m_bitwise_clause_generator(encoding_context.m_bitwise_clause_generator)
	, m_bit_clause_generator(encoding_context.m_bit_clause_generator)
	, m_vertex_occupancy(encoding_context.m_vertex_occupancy)
	, m_robot_location(encoding_context.m_robot_location)
	, m_transition_action(encoding_context.m_transition_action)
	, m_vertex_occupancy_by_water(encoding_context.m_vertex_occupancy_by_water)
	, m_vertex_occupancy_by_water_(encoding_context.m_vertex_occupancy_by_water_)	  
	, m_vertex_occupancy_by_gas_(encoding_context.m_vertex_occupancy_by_gas_)	  
	, m_unified_vertex_occupancy_by_water_(encoding_context.m_unified_vertex_occupancy_by_water_)	  
	, m_wet_vertex_occupancy_by_water_(encoding_context.m_wet_vertex_occupancy_by_water_)	  
	, m_vertex_water_cardinality_(encoding_context.m_vertex_water_cardinality_)
	, m_vertex_occupancy_by_robot(encoding_context.m_vertex_occupancy_by_robot)
	, m_robot_location_in_vertex(encoding_context.m_robot_location_in_vertex)
	, m_edge_occupancy_by_water(encoding_context.m_edge_occupancy_by_water)
	, m_edge_occupancy_by_water_(encoding_context.m_edge_occupancy_by_water_)
	, m_edge_occupancy_by_water__(encoding_context.m_edge_occupancy_by_water__)
	, m_unified_edge_occupancy_by_water__(encoding_context.m_unified_edge_occupancy_by_water__)	  
	, m_wet_edge_occupancy_by_water__(encoding_context.m_wet_edge_occupancy_by_water__)	  
	, m_transition_Actions(encoding_context.m_transition_Actions)
    {
	register_InternalIdentifiers();
	switch_GeneratingMode(encoding_context.get_GeneratingMode());
    }


    const sMultirobotEncodingContext_CNFsat& sMultirobotEncodingContext_CNFsat::operator=(const sMultirobotEncodingContext_CNFsat &encoding_context)
    {
	m_N_Layers = encoding_context.m_N_Layers;
	m_max_total_cost = encoding_context.m_max_total_cost;
	m_extra_cost = encoding_context.m_extra_cost;
	m_max_total_fuel = encoding_context.m_max_total_fuel;
	m_extra_fuel = encoding_context.m_extra_fuel;
	m_fuel_makespan = encoding_context.m_fuel_makespan;
	m_variable_store = encoding_context.m_variable_store;
	m_clause_generator = encoding_context.m_clause_generator;
	m_bit_generator = encoding_context.m_bit_generator;
	m_state_clause_generator = encoding_context.m_state_clause_generator;
	m_advanced_clause_generator = encoding_context.m_advanced_clause_generator;
	m_bitwise_clause_generator = encoding_context.m_bitwise_clause_generator;
	m_bit_clause_generator = encoding_context.m_bit_clause_generator;
	m_vertex_occupancy = encoding_context.m_vertex_occupancy;
	m_robot_location = encoding_context.m_robot_location;
	m_robot_location_in_vertex = encoding_context.m_robot_location_in_vertex;
	m_edge_occupancy_by_water = encoding_context.m_edge_occupancy_by_water;
	m_edge_occupancy_by_water_ = encoding_context.m_edge_occupancy_by_water_;
	m_edge_occupancy_by_water__ = encoding_context.m_edge_occupancy_by_water__;
	m_unified_edge_occupancy_by_water__ = encoding_context.m_unified_edge_occupancy_by_water__;	
	m_wet_edge_occupancy_by_water__ = encoding_context.m_wet_edge_occupancy_by_water__;	
	m_transition_action = encoding_context.m_transition_action;
	m_vertex_occupancy_by_water = encoding_context.m_vertex_occupancy_by_water;
	m_vertex_occupancy_by_water_ = encoding_context.m_vertex_occupancy_by_water_;
	m_vertex_occupancy_by_gas_ = encoding_context.m_vertex_occupancy_by_gas_;	
	m_unified_vertex_occupancy_by_water_ = encoding_context.m_unified_vertex_occupancy_by_water_;	
	m_wet_vertex_occupancy_by_water_ = encoding_context.m_wet_vertex_occupancy_by_water_;	
	m_vertex_water_cardinality_ = encoding_context.m_vertex_water_cardinality_;
	m_vertex_occupancy_by_robot = encoding_context.m_vertex_occupancy_by_robot;
	m_transition_Actions = encoding_context.m_transition_Actions;
	switch_GeneratingMode(encoding_context.get_GeneratingMode());

	register_InternalIdentifiers();

	return *this;
    }


    sMultirobotEncodingContext_CNFsat::GeneratingMode sMultirobotEncodingContext_CNFsat::get_GeneratingMode(void) const
    {
	return m_generating_mode;
    }

	
    void sMultirobotEncodingContext_CNFsat::switch_GeneratingMode(GeneratingMode generating_mode)
    {
	m_generating_mode = generating_mode;

	switch (generating_mode)
	{
	case GENERATING_STANDARD:
	{
	    m_clause_generator = &m_state_clause_generator;
	    break;
	}
	case GENERATING_ADVANCED:
	{
	    m_clause_generator = &m_advanced_clause_generator;
	    break;
	}
	case GENERATING_BITWISE:
	{
	    m_clause_generator = &m_bitwise_clause_generator;
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	m_bit_generator = &m_bit_clause_generator;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_StandardGeneratingMode(void)
    {
	m_clause_generator = &m_state_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_STANDARD;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_AdvancedGeneratingMode(void)
    {
	m_clause_generator = &m_advanced_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_ADVANCED;
    }


    void sMultirobotEncodingContext_CNFsat::switchTo_BitwiseGeneratingMode(void)
    {
	m_clause_generator = &m_bitwise_clause_generator;
	m_bit_generator = &m_bit_clause_generator;
	m_generating_mode = GENERATING_BITWISE;
    }


    void sMultirobotEncodingContext_CNFsat::register_InternalIdentifiers(void)
    {
	m_state_Identifiers.clear();
	m_state_Identifiers[sINT_32_MAX] = NULL;

	m_bit_Identifiers.clear();
	m_bit_Identifiers[sINT_32_MAX] = NULL;

	if (!m_vertex_occupancy.is_Anonymous())
	{
	    register_TranslateIdentifier(m_vertex_occupancy);
	}
	if (!m_robot_location.is_Anonymous())
	{
	    register_TranslateIdentifier(m_robot_location);
	}
	if (!m_robot_location_in_vertex.is_Anonymous())
	{
	    register_TranslateIdentifier(m_robot_location_in_vertex);
	}
	if (!m_vertex_occupancy_by_robot.is_Anonymous())
	{
	    register_TranslateIdentifier(m_vertex_occupancy_by_robot);
	}

	for (StateIdentifiers_vector::iterator transition_action = m_transition_Actions.begin(); transition_action != m_transition_Actions.end(); ++transition_action)
	{
	    if (!transition_action->is_Anonymous())
	    {
		register_TranslateIdentifier(*transition_action);
	    }
	}
	for (BitIdentifiers_vector::iterator edge_occupancy = m_edge_occupancy_by_water.begin(); edge_occupancy != m_edge_occupancy_by_water.end(); ++edge_occupancy)
	{
	    if (!edge_occupancy->is_Anonymous())
	    {
		register_TranslateIdentifier(*edge_occupancy);
	    }
	}
	for (BitIdentifiers_2d_vector::iterator edge_occupancy_ = m_edge_occupancy_by_water_.begin(); edge_occupancy_ != m_edge_occupancy_by_water_.end(); ++edge_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator edge_occupancy = edge_occupancy_->begin(); edge_occupancy != edge_occupancy_->end(); ++edge_occupancy)
	    {
		if (!edge_occupancy->is_Anonymous())
		{
		    register_TranslateIdentifier(*edge_occupancy);
		}
	    }
	}

	for (BitIdentifiers_2d_vector::iterator wet_edge_occupancy__ = m_wet_edge_occupancy_by_water__.begin(); wet_edge_occupancy__ != m_wet_edge_occupancy_by_water__.end(); ++wet_edge_occupancy__)
	{
	    for (BitIdentifiers_vector::iterator wet_edge_occupancy = wet_edge_occupancy__->begin(); wet_edge_occupancy != wet_edge_occupancy__->end(); ++wet_edge_occupancy)
	    {
		if (!wet_edge_occupancy->is_Anonymous())
		{
		    register_TranslateIdentifier(*wet_edge_occupancy);
		}
	    }
	}	

	for (BitIdentifiers_2d_vector::iterator vertex_occupancy_ = m_vertex_occupancy_by_water_.begin(); vertex_occupancy_ != m_vertex_occupancy_by_water_.end(); ++vertex_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator vertex_occupancy = vertex_occupancy_->begin(); vertex_occupancy != vertex_occupancy_->end(); ++vertex_occupancy)
	    {
		if (!vertex_occupancy->is_Anonymous())
		{
		    register_TranslateIdentifier(*vertex_occupancy);
		}
	    }
	}

	for (BitIdentifiers_2d_vector::iterator vertex_occupancy_ = m_vertex_occupancy_by_gas_.begin(); vertex_occupancy_ != m_vertex_occupancy_by_gas_.end(); ++vertex_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator vertex_occupancy = vertex_occupancy_->begin(); vertex_occupancy != vertex_occupancy_->end(); ++vertex_occupancy)
	    {
		if (!vertex_occupancy->is_Anonymous())
		{
		    register_TranslateIdentifier(*vertex_occupancy);
		}
	    }
	}	
	for (BitIdentifiers_vector::iterator unified_vertex_occupancy = m_unified_vertex_occupancy_by_water_.begin(); unified_vertex_occupancy != m_unified_vertex_occupancy_by_water_.end(); ++unified_vertex_occupancy)
	{
	    if (!unified_vertex_occupancy->is_Anonymous())
	    {
		register_TranslateIdentifier(*unified_vertex_occupancy);
	    }
	}	
	for (BitIdentifiers_vector::iterator wet_vertex_occupancy = m_wet_vertex_occupancy_by_water_.begin(); wet_vertex_occupancy != m_wet_vertex_occupancy_by_water_.end(); ++wet_vertex_occupancy)
	{
	    if (!wet_vertex_occupancy->is_Anonymous())
	    {
		register_TranslateIdentifier(*wet_vertex_occupancy);
	    }
	}	

	for (BitIdentifiers_2d_vector::iterator vertex_cardinality_ = m_vertex_water_cardinality_.begin(); vertex_cardinality_ != m_vertex_water_cardinality_.end(); ++vertex_cardinality_)
	{
	    for (BitIdentifiers_vector::iterator vertex_cardinality = vertex_cardinality_->begin(); vertex_cardinality != vertex_cardinality_->end(); ++vertex_cardinality)
	    {
		if (!vertex_cardinality->is_Anonymous())
		{
		    register_TranslateIdentifier(*vertex_cardinality);
		}
	    }
	}

	for (BitIdentifiers_3d_vector::iterator edge_occupancy__ = m_edge_occupancy_by_water__.begin(); edge_occupancy__ != m_edge_occupancy_by_water__.end(); ++edge_occupancy__)
	{
	    for (BitIdentifiers_2d_vector::iterator edge_occupancy_ = edge_occupancy__->begin(); edge_occupancy_ != edge_occupancy__->end(); ++edge_occupancy_)
	    {
		for (BitIdentifiers_vector::iterator edge_occupancy = edge_occupancy_->begin(); edge_occupancy != edge_occupancy_->end(); ++edge_occupancy)
		{
		    if (!edge_occupancy->is_Anonymous())
		    {
			register_TranslateIdentifier(*edge_occupancy);
		    }
		}
	    }
	}

	for (BitIdentifiers_2d_vector::iterator unified_edge_occupancy_ = m_unified_edge_occupancy_by_water__.begin(); unified_edge_occupancy_ != m_unified_edge_occupancy_by_water__.end(); ++unified_edge_occupancy_)
	{
	    for (BitIdentifiers_vector::iterator unified_edge_occupancy = unified_edge_occupancy_->begin(); unified_edge_occupancy != unified_edge_occupancy_->end(); ++unified_edge_occupancy)
	    {
		if (!unified_edge_occupancy->is_Anonymous())
		{
		    register_TranslateIdentifier(*unified_edge_occupancy);
		}
	    }
	}	
    }


    void sMultirobotEncodingContext_CNFsat::register_TranslateIdentifier(sIndexableStateIdentifier &state_identifier)
    {
	if (state_identifier.get_First_CNFVariable() >= 1)
	{
	    m_state_Identifiers[state_identifier.get_First_CNFVariable()] = &state_identifier;
	}
    }


    void sMultirobotEncodingContext_CNFsat::register_TranslateIdentifier(sIndexableBitIdentifier &bit_identifier)
    {
	if (bit_identifier.get_First_CNFVariable() >= 1)
	{
	    m_bit_Identifiers[bit_identifier.get_First_CNFVariable()] = &bit_identifier;
	}
    }


    sSpecifiedIdentifier sMultirobotEncodingContext_CNFsat::translate_CNF_Variable(int cnf_variable) const
    {
	sSpecifiedIdentifier state_translation;
	StateIdentifiers_map::const_iterator state_identifier = m_state_Identifiers.upper_bound(cnf_variable);

	if (state_identifier != m_state_Identifiers.end())
	{
	    if (state_identifier->first > cnf_variable)
	    {
		--state_identifier;
	    }
	    if (state_identifier != m_state_Identifiers.end() && state_identifier->second != NULL)
	    {
		state_translation = state_identifier->second->translate_CNFVariable(cnf_variable);
	    }
	}
	if (!state_translation.is_Null())
	{
	    return state_translation;
	}
	else
	{
	    sSpecifiedIdentifier bit_translation;
	    BitIdentifiers_map::const_iterator bit_identifier = m_bit_Identifiers.upper_bound(cnf_variable);
	    /*
	    printf("bit_var:%d,%ld\n", cnf_variable, m_bit_Identifiers.size());
	    for (BitIdentifiers_map::const_iterator identifier = m_bit_Identifiers.begin(); identifier != m_bit_Identifiers.end(); ++identifier)
	    {
		if (identifier->second != NULL)
		{
		    //		identifier->second->to_Screen();
		    printf("%d\n", identifier->first);
		}
	    }
	    */
	    if (bit_identifier != m_bit_Identifiers.end())
	    {
		if (bit_identifier->first > cnf_variable)
		{
		    --bit_identifier;
		}
		if (bit_identifier != m_bit_Identifiers.end() && bit_identifier->second != NULL)
		{
		    bit_translation = bit_identifier->second->translate_CNFVariable(cnf_variable);
		}
	    }
	    if (!bit_translation.is_Null())
	    {
		return bit_translation;
	    }
	    else
	    {
		return m_clause_generator->translate_AuxiliaryCNFVariable(cnf_variable);
	    }
	}
	return sSpecifiedIdentifier();
    }

    
    void sMultirobotEncodingContext_CNFsat::to_Screen_identifiers(const sString &indent) const
    {
	printf("%sEncoding identifiers {\n", indent.c_str());
	printf("%s%sState identifiers (%ld) [\n", indent.c_str(), sRELOC_INDENT.c_str(), m_state_Identifiers.size());
	for (StateIdentifiers_map::const_iterator state_identifier = m_state_Identifiers.begin(); state_identifier != m_state_Identifiers.end(); ++state_identifier)
	{
	    if (state_identifier->first < sINT_32_MAX)
	    {
		printf("%s%s%d --->\n", indent.c_str(), sRELOC_2_INDENT.c_str(), state_identifier->first);
		state_identifier->second->to_Screen(indent + sRELOC_3_INDENT);
	    }
	}
	printf("%s%s]\n", indent.c_str(), sRELOC_INDENT.c_str());

	printf("%s%sBit identifiers (%ld) [\n", indent.c_str(), sRELOC_INDENT.c_str(), m_bit_Identifiers.size());
	for (BitIdentifiers_map::const_iterator bit_identifier = m_bit_Identifiers.begin(); bit_identifier != m_bit_Identifiers.end(); ++bit_identifier)
	{
	    if (bit_identifier->first < sINT_32_MAX)
	    {
		printf("%s%s%d --->\n", indent.c_str(), sRELOC_2_INDENT.c_str(), bit_identifier->first);
		bit_identifier->second->to_Screen(indent + sRELOC_3_INDENT);
	    }
	}
	printf("%s%s]\n", indent.c_str(), sRELOC_INDENT.c_str());

	printf("%s}\n", indent.c_str());	
    }

    
/*----------------------------------------------------------------------------*/
// sMultirobotInstance

    sMultirobotInstance::sMultirobotInstance()
    {
	// nothing
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotArrangement &goal_arrangement)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
	, m_ratio(-1.0)
	, m_robustness(1)
	, m_range(0) 
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());

	m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), goal_arrangement.get_RobotLocation(robot)));
	}
	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);	
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotArrangement &goal_arrangement)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
	, m_ratio(-1.0)
	, m_robustness(1)
	, m_range(0)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotArrangement &goal_arrangement,
					     double                   ratio,
					     int                      robustness,
					     int                      range)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
	, m_ratio(ratio)
	, m_robustness(robustness)
	, m_range(range)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());

	m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), goal_arrangement.get_RobotLocation(robot)));
	}
	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);	
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotArrangement &goal_arrangement,
					     double                   ratio,
					     int                      robustness,
					     int                      range)
	: m_goal_type(GOAL_TYPE_ARRANGEMENT)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
	, m_ratio(ratio)
	, m_robustness(robustness)
	, m_range(range)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());
    }    


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotGoal &goal_specification)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
	, m_ratio(-1.0)
	, m_robustness(1)
	, m_range(0)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());

//	m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;
 	int N_Robots = m_initial_arrangement.get_RobotCount();

	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    double p = (double)rand() / RAND_MAX;
	    //  printf("%f\n", p);

	    if (p <= 1.0)
	    {
		const sRobotGoal::Vertices_set &robot_goal = goal_specification.get_RobotGoal(robot);
		if (robot_goal.size() == 1)
		{
		    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), *robot_goal.begin()));
		}
		else
		{
		    sASSERT(false);
		}
	    }
	}
//	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);	
//	m_environment.build_SpanningTree(0, m_sparse_environment);
//	printf("%.3f\n", (double)m_sparse_environment.get_EdgeCount() / m_environment.get_EdgeCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotGoal        &goal_specification)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
	, m_ratio(-1.0)
	, m_robustness(1)
	, m_range(0)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph &environment, const sRobotArrangement &initial_arrangement, const sRobotGoal &goal_specification, double ratio, int robustness, int range)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
	, m_ratio(ratio)
	, m_robustness(robustness)
	, m_range(range)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());

//	m_environment.build_SpanningTree(initial_arrangement.get_RobotLocation(1), m_sparse_environment);
	sUndirectedGraph::VertexPairs_vector vertex_Pairs;
 	int N_Robots = m_initial_arrangement.get_RobotCount();

	for (int robot = 1; robot <= N_Robots; ++robot)
	{
	    double p = (double)rand() / RAND_MAX;
	    //  printf("%f\n", p);

	    if (p <= 1.0)
	    {
		const sRobotGoal::Vertices_set &robot_goal = goal_specification.get_RobotGoal(robot);
		if (robot_goal.size() == 1)
		{
		    vertex_Pairs.push_back(sUndirectedGraph::Vertex_pair(initial_arrangement.get_RobotLocation(robot), *robot_goal.begin()));
		}
		else
		{
		    sASSERT(false);
		}
	    }
	}
//	m_environment.build_SparseGraph(vertex_Pairs, m_sparse_environment);	
//	m_environment.build_SpanningTree(0, m_sparse_environment);
//	printf("%.3f\n", (double)m_sparse_environment.get_EdgeCount() / m_environment.get_EdgeCount());
    }


    sMultirobotInstance::sMultirobotInstance(const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     const sRobotArrangement &initial_arrangement,
					     const sRobotGoal        &goal_specification,
					     double                   ratio,
					     int                      robustness,
					     int                      range)
	: m_goal_type(GOAL_TYPE_SPECIFICATION)
	, m_environment(environment)
	, m_sparse_environment(sparse_environment)
	, m_initial_arrangement(initial_arrangement)
	, m_goal_specification(goal_specification)
	, m_ratio(ratio)
	, m_robustness(robustness)
	, m_range(range)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_specification.get_VertexCount());
    }    



/*----------------------------------------------------------------------------*/


    void sMultirobotInstance::collect_Endpoints(VertexIDs_vector &source_IDs, VertexIDs_vector &goal_IDs)
    {
	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    source_IDs.push_back(robot_source_vertex_id);
	    goal_IDs.push_back(robot_sink_vertex_id);
	}
    }


    int sMultirobotInstance::analyze_EdgeHeights(int max_total_cost)
    {
	int min_total_cost = INT_MAX;

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    int min_edge_robot_id = -1;
	    int min_edge_cost = INT_MAX;
	    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    
	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		
		int edge_cost = sMIN(all_pairs_Distances[robot_source_vertex_id][edge_vertex_u_id] + all_pairs_Distances[edge_vertex_v_id][robot_sink_vertex_id],
				     all_pairs_Distances[robot_source_vertex_id][edge_vertex_v_id] + all_pairs_Distances[edge_vertex_u_id][robot_sink_vertex_id]);
		
		if (edge_cost < min_edge_cost)
		{
		    min_edge_cost = edge_cost;
		    min_edge_robot_id = robot_id;
		}
	    }
	    
	    int rest_cost = 0;
	    
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_edge_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    rest_cost += all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		}
	    }
	    int total_cost = min_edge_cost + rest_cost;
	    int edge_height = max_total_cost - total_cost;
	    printf("Edge height: %d\n", edge_height);

	    if (min_total_cost > total_cost)
	    {
		min_total_cost = total_cost;
	    }
	}
	return min_total_cost;
    }


    int sMultirobotInstance::analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height)
    {
	sUndirectedGraph::VertexIDs_set reachable_IDs;

	return analyze_EdgeHeights_(max_total_cost, max_vertex_height, reachable_IDs);
    }


    int sMultirobotInstance::analyze_EdgeHeights_(int max_total_cost, int &max_vertex_height, sUndirectedGraph::VertexIDs_set &reachable_IDs)
    {
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	max_vertex_height = 0;

	int min_total_cost = 0;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    int robot_cost = all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    min_total_cost += robot_cost;
	}

	for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_environment.m_Vertices.begin(); vertex != m_environment.m_Vertices.end(); ++vertex)
	{ 
	    int min_robot_cost = INT_MAX;
	    int min_robot_id = -1;

	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		int cost = all_pairs_Distances[robot_source_vertex_id][vertex->m_id] + all_pairs_Distances[vertex->m_id][robot_sink_vertex_id];

		if (cost < min_robot_cost)
		{
		    min_robot_cost = cost;
		    min_robot_id = robot_id;
		}
	    }
	    printf("Min robot:%d (cost:%d)\n", min_robot_id, min_robot_cost);

	    int rest_cost = 0;
	    int overlaps = 0;

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    int cost = all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		    int overlap_cost = all_pairs_Distances[robot_source_vertex_id][vertex->m_id] + all_pairs_Distances[vertex->m_id][robot_sink_vertex_id];

		    if (cost == overlap_cost)
		    {
			++overlaps;
		    }
		    rest_cost += cost;
		}
	    }
	    int total_cost = min_robot_cost + rest_cost;
	    int remaining_cost = max_total_cost - total_cost;

	    printf("Tc: %d\n", total_cost);
	    int vertex_height = -1;

	    if (remaining_cost >= 0)
	    {
		vertex_height = remaining_cost + overlaps;
		reachable_IDs.insert(vertex->m_id);
	    }
	    if (max_vertex_height < vertex_height)
	    {
		max_vertex_height = vertex_height;
	    }
	    printf("Height: %d\n", vertex_height);
	}
//	printf("Total: %d (%d) Size: %d (%.3f)\n", min_total_cost, max_vertex_height, reachable_IDs.size(), (double)reachable_IDs.size() / m_environment.get_VertexCount());

	return min_total_cost;
    }


    int sMultirobotInstance::build_HeightedEnvironments(int max_total_cost)
    {
	return build_HeightedEnvironments(max_total_cost, m_heighted_Environments);
    }


    int sMultirobotInstance::build_HeightedEnvironments(int max_total_cost, Environments_vector &heighted_Environments)
    {
	int min_total_cost = INT_MAX;
	heighted_Environments.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_AllPairsShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    int min_edge_robot_id = -1;
	    int min_edge_cost = INT_MAX;
	    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    
	    int N_Robots = m_initial_arrangement.get_RobotCount();
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		int robot_sink_vertex_id;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		    sASSERT(goal_IDs.size() == 1);
		    robot_sink_vertex_id = *goal_IDs.begin();
		    break;
		}
		default:
		{
		    break;
		}
		}
		
		int edge_cost = sMIN(all_pairs_Distances[robot_source_vertex_id][edge_vertex_u_id] + all_pairs_Distances[edge_vertex_v_id][robot_sink_vertex_id],
				     all_pairs_Distances[robot_source_vertex_id][edge_vertex_v_id] + all_pairs_Distances[edge_vertex_u_id][robot_sink_vertex_id]);
		
		if (edge_cost < min_edge_cost)
		{
		    min_edge_cost = edge_cost;
		    min_edge_robot_id = robot_id;
		}
	    }
	    
	    int rest_cost = 0;
	    
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		if (robot_id != min_edge_robot_id)
		{
		    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		    int robot_sink_vertex_id;
		    
		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(goal_IDs.size() == 1);
			robot_sink_vertex_id = *goal_IDs.begin();
			break;
		    }
		    default:
		    {
			break;
		    }
		    }
		    rest_cost += all_pairs_Distances[robot_source_vertex_id][robot_sink_vertex_id];
		}
	    }
	    int total_cost = min_edge_cost + rest_cost;
	    int edge_height = max_total_cost - total_cost;

	    printf("eh: %d\n", edge_height);

	    if (edge_height >= 0)
	    {
		int environment_height = heighted_Environments.size();
		while (environment_height++ <= edge_height)
		{
		    heighted_Environments.push_back(sUndirectedGraph());
		    heighted_Environments.back().add_Vertices(m_environment.get_VertexCount());
		}
		for (int eh = 0; eh < edge_height; ++eh)
		{
		    if (!heighted_Environments[eh].is_Adjacent(edge_vertex_u_id, edge_vertex_v_id))
		    {
			heighted_Environments[eh].add_Edge(edge_vertex_u_id, edge_vertex_v_id);
		    }
		}
	    }
	    if (min_total_cost > total_cost)
	    {
		min_total_cost = total_cost;
	    }
	}
	printf("------>\n");
	if (heighted_Environments.empty())
	{
	    heighted_Environments.push_back(sUndirectedGraph());
	    heighted_Environments.back().add_Vertices(m_environment.get_VertexCount());
	}
	return min_total_cost;
    }


    int sMultirobotInstance::build_HeightedEnvironments_(int max_total_cost)
    {
	return build_HeightedEnvironments_(max_total_cost, m_heighted_Environments);
    }


    int sMultirobotInstance::build_HeightedEnvironments_(int max_total_cost, Environments_vector &heighted_Environments)
    {
	heighted_Environments.clear();

	int max_vertex_height = 0;
	sUndirectedGraph::VertexIDs_set reachable_IDs;

	int min_total_cost = analyze_EdgeHeights_(max_total_cost, max_vertex_height, reachable_IDs);

	sUndirectedGraph heighted_environment;
	heighted_environment.add_Vertices(m_environment.get_VertexCount());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{    
	    int edge_vertex_u_id = edge->m_arc_uv.m_source->m_id;
	    int edge_vertex_v_id = edge->m_arc_uv.m_target->m_id;
	    /*
	    sUndirectedGraph::VertexIDs_set::const_iterator reachable_u = reachable_IDs.find(edge_vertex_u_id);
	    sUndirectedGraph::VertexIDs_set::const_iterator reachable_v = reachable_IDs.find(edge_vertex_v_id);
	    */
//	    if (reachable_u != reachable_IDs.end() && reachable_v != reachable_IDs.end())
	    {
		heighted_environment.add_Edge(edge_vertex_u_id, edge_vertex_v_id);
	    }
	}
	for (int height = 0; height <= max_vertex_height; ++height)
	{
	    heighted_Environments.push_back(heighted_environment);
	}
	return min_total_cost;
    }

    
    int sMultirobotInstance::estimate_TotalCost(int &max_individual_cost)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	int min_total_cost = 0;
	max_individual_cost = 0;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
	    min_total_cost += robot_cost;

	    if (robot_cost > max_individual_cost)
	    {
		max_individual_cost = robot_cost;
	    }
	}
	return min_total_cost;
    }


    int sMultirobotInstance::estimate_TotalFuel(int &max_individual_fuel)
    {	
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	collect_Endpoints(source_IDs, goal_IDs);
	m_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = m_environment.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = m_environment.get_GoalShortestPaths();

	int min_total_fuel = 0;
	max_individual_fuel = 0;

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    int robot_sink_vertex_id;
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    int robot_fuel = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
	    min_total_fuel += robot_fuel;

	    if (robot_fuel > max_individual_fuel)
	    {
		max_individual_fuel = robot_fuel;
	    }
	}
	return min_total_fuel;
    }    


    int sMultirobotInstance::construct_MDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }


    int sMultirobotInstance::construct_FuelMDD(int max_total_fuel, int fuel_makespan, MDD_vector &MDD, int &extra_fuel, MDD_vector &extra_MDD)
    {
	return construct_GraphFuelMDD(m_environment, max_total_fuel, fuel_makespan, MDD, extra_fuel, extra_MDD);
    }    


    int sMultirobotInstance::construct_DisplacementMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphDisplacementMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }


    int sMultirobotInstance::construct_LimitedMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphLimitedMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }        

    
    int sMultirobotInstance::construct_SparseMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphMDD(m_sparse_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }
    

    int sMDD_Addition[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//    int sMDD_Addition[] = { 2, 1, 1, 0, 0, 0, 0, 0, 0, 0 };  36.94
//    int sMDD_Addition[] = { 2, 1, 0, 0, 0, 0, 0, 0, 0, 0 }; // 33.97
//    int sMDD_Addition[] = { 2, 2, 1, 1, 1, 0, 0, 0, 0, 0 };  51.21
//    int sMDD_Addition[] = { 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };  54.29
//    int sMDD_Addition[] = { 4, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 43.99
//    int sMDD_Addition[] = { 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //30.00
//    int sMDD_Addition[] = { 4, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 45.0
//    int sMDD_Addition[] = { 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 36.66
//    int sMDD_Addition[] = { 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; 49.0
//    int sMDD_Addition[] = { 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//    int sMDD_Addition[] = { 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 };
//    int sMDD_Addition[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
    
    typedef std::multimap<int, int> RobotIndices_mmap;
    
    int sMultirobotInstance::construct_GraphMDD(sUndirectedGraph &graph, int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {	
	int max_individual_cost;
	int N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	int min_total_cost = estimate_TotalCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
	
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	RobotIndices_mmap sorted_mdd_Robots;

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    sorted_mdd_Robots.insert(RobotIndices_mmap::value_type(robot_cost, mdd_robot_id));
	}

	int sort_index = 0;
	for (RobotIndices_mmap::const_reverse_iterator sort_robot = sorted_mdd_Robots.rbegin(); sort_robot != sorted_mdd_Robots.rend(); ++sort_robot)
	{
	    int add_index = (sort_index++ * (sizeof(sMDD_Addition) / sizeof(int))) / N_Robots;
	    int extra_addition = sMDD_Addition[add_index];

	    int mdd_robot_id = sort_robot->second;
	    
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
/*
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= robot_cost + extra_cost - mdd_level)
		    {
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
			printf("mdd_level:%d\n", mdd_level);
		    }
		}
	    }
*/

	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int mdd_level = source_Distances[robot_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(robot_cost + (extra_cost + extra_addition) - goal_Distances[robot_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
		}
	    }

	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_robot_id][mdd_level].empty())
		{
		    MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
		if (   mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id]
		    && mdd_level < source_Distances[robot_source_vertex_id][robot_sink_vertex_id] + (extra_cost + extra_addition))
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
	    }
	}
/*
	printf("Distance printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    printf("robot:%d (%d,%d)\n", mdd_robot, robot_source_vertex_id, robot_sink_vertex_id);
	    
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int  source_dist = source_Distances[robot_source_vertex_id][vertex_id];
		int  goal_dist = goal_Distances[robot_sink_vertex_id][vertex_id];

		printf("  %d:%d,%d\n", vertex_id, source_dist, goal_dist);
	    }	   
	}
	printf("<----\n");
*/
/*
	printf("MDD printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    printf("robot:%d\n", mdd_robot);
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (!extra_MDD[mdd_robot][mdd_level].empty())
		{
		    printf("* ");
		}		
		for (int i = 0; i < MDD[mdd_robot][mdd_level].size(); ++i)		    
		{		    
		    printf("%d ", MDD[mdd_robot][mdd_level][i]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
	printf("<----\n");
	getchar();
*/
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
	return mdd_depth;
    }


    int sMultirobotInstance::construct_GraphFuelMDD(sUndirectedGraph &graph, int max_total_fuel, int fuel_makespan, MDD_vector &MDD, int &extra_fuel, MDD_vector &extra_MDD)
    {	
	int max_individual_fuel;
	int N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	int min_total_fuel = estimate_TotalFuel(max_individual_fuel);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_fuel = max_total_fuel - min_total_fuel;
	int mdd_depth = fuel_makespan;
	
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	RobotIndices_mmap sorted_mdd_Robots;

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_fuel = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    sorted_mdd_Robots.insert(RobotIndices_mmap::value_type(robot_fuel, mdd_robot_id));
	}

	//int sort_index = 0;
	for (RobotIndices_mmap::const_reverse_iterator sort_robot = sorted_mdd_Robots.rbegin(); sort_robot != sorted_mdd_Robots.rend(); ++sort_robot)
	{
	    //int add_index = (sort_index++ * (sizeof(sMDD_Addition) / sizeof(int))) / N_Robots;
	    //int extra_addition = sMDD_Addition[add_index];

	    int mdd_robot_id = sort_robot->second;
	    
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_fuel = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= mdd_depth - mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= robot_fuel + extra_fuel - source_Distances[robot_source_vertex_id][vertex_id])
		    {
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
		    }
		}
	    }

/*
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int mdd_level = source_Distances[robot_source_vertex_id][vertex_id];		     
		     mdd_level <= mdd_depth - goal_Distances[robot_sink_vertex_id][vertex_id];
		     ++mdd_level)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
		}
	    }
*/
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (MDD[mdd_robot_id][mdd_level].empty())
		{
		    MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
/*		
		if (mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id])
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
*/
	    }
	}
/*
	printf("Distance printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    printf("robot:%d (%d,%d)\n", mdd_robot, robot_source_vertex_id, robot_sink_vertex_id);
	    
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int  source_dist = source_Distances[robot_source_vertex_id][vertex_id];
		int  goal_dist = goal_Distances[robot_sink_vertex_id][vertex_id];

		printf("  %d:%d,%d\n", vertex_id, source_dist, goal_dist);
	    }	   
	}
	printf("<----\n");
*/
/*
	printf("MDD printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    printf("robot:%d\n", mdd_robot);
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (!extra_MDD[mdd_robot][mdd_level].empty())
		{
		    printf("* ");
		}
		for (int i = 0; i < MDD[mdd_robot][mdd_level].size(); ++i)
		{		    
		    printf("%d ", MDD[mdd_robot][mdd_level][i]);
		}
		printf("\n");		
	    }
	    printf("\n");
	}
	printf("<----\n");
	getchar();
*/
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
	return mdd_depth;
    }
    

    int sMultirobotInstance::construct_GraphDisplacementMDD(sUndirectedGraph &graph, int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {	
	int max_individual_cost;
	int N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	int min_total_cost = estimate_TotalCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
	
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	RobotIndices_mmap sorted_mdd_Robots;

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    sorted_mdd_Robots.insert(RobotIndices_mmap::value_type(robot_cost, mdd_robot_id));
	}

	int sort_index = 0;
	for (RobotIndices_mmap::const_reverse_iterator sort_robot = sorted_mdd_Robots.rbegin(); sort_robot != sorted_mdd_Robots.rend(); ++sort_robot)
	{
	    int add_index = (sort_index++ * (sizeof(sMDD_Addition) / sizeof(int))) / N_Robots;
	    int extra_addition = sMDD_Addition[add_index];

	    int mdd_robot_id = sort_robot->second;
	    
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
/*
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= robot_cost + extra_cost - mdd_level)
		    {
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
			printf("mdd_level:%d\n", mdd_level);
		    }
		}
	    }
*/	    

	    RobotMDD_set robot_MDD;
	    robot_MDD.resize(mdd_depth + 1);
		
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int mdd_level = source_Distances[robot_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(robot_cost + (extra_cost + extra_addition) - goal_Distances[robot_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{		    
		    if (m_initial_arrangement.m_robot_Sizes[mdd_robot_id] > 1)
		    {
			for (int dr = 0; dr < m_initial_arrangement.m_robot_Sizes[mdd_robot_id]; ++dr)
			{
			    for (int dc = 0; dc < m_initial_arrangement.m_robot_Sizes[mdd_robot_id]; ++dc)
			    {
				int grid_vertex_id = graph.get_GridNeighborVertexID(vertex_id, dr, dc);
				if (grid_vertex_id >= 0)
				{
				    robot_MDD[mdd_level].insert(grid_vertex_id);
				}
			    }						
			}
		    }
		    else
		    {
			robot_MDD[mdd_level].insert(vertex_id);
		    }
		}
	    }
	    
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (robot_MDD[mdd_level].empty())
		{
		    if (m_initial_arrangement.m_robot_Sizes[mdd_robot_id] > 1)
		    {
			for (int dr = 0; dr < m_initial_arrangement.m_robot_Sizes[mdd_robot_id]; ++dr)
			{
			    for (int dc = 0; dc < m_initial_arrangement.m_robot_Sizes[mdd_robot_id]; ++dc)
			    {
				int grid_vertex_id = graph.get_GridNeighborVertexID(robot_sink_vertex_id, dr, dc);
				if (grid_vertex_id >= 0)
				{
				    robot_MDD[mdd_level].insert(grid_vertex_id);
				}
			    }						
			}						     
		    }
		    else
		    {
			robot_MDD[mdd_level].insert(robot_sink_vertex_id);
		    }		    
		}
		if (   mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id]
		    && mdd_level < source_Distances[robot_source_vertex_id][robot_sink_vertex_id] + (extra_cost + extra_addition))
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
	    }

	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (VertexIDs_set::const_iterator vertex_ID = robot_MDD[mdd_level].begin(); vertex_ID != robot_MDD[mdd_level].end(); ++vertex_ID)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(*vertex_ID);
		}
	    }
	}
/*
	printf("Distance printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    printf("robot:%d (%d,%d)\n", mdd_robot, robot_source_vertex_id, robot_sink_vertex_id);
	    
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int  source_dist = source_Distances[robot_source_vertex_id][vertex_id];
		int  goal_dist = goal_Distances[robot_sink_vertex_id][vertex_id];

		printf("  %d:%d,%d\n", vertex_id, source_dist, goal_dist);
	    }	   
	}
	printf("<----\n");
	
	printf("MDD printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    printf("robot:%d\n", mdd_robot);
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int i = 0; i < MDD[mdd_robot][mdd_level].size(); ++i)
		{		    
		    printf("%d ", MDD[mdd_robot][mdd_level][i]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
	printf("<----\n");		
*/	
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
	return mdd_depth;
    }


    int sMultirobotInstance::construct_GraphLimitedMDD(sUndirectedGraph &graph, int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {	
	int max_individual_cost;
	int N_Vertices = graph.get_VertexCount();	

	MDD.clear();
	extra_MDD.clear();

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);

	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);
	int min_total_cost = estimate_TotalCost(max_individual_cost);
	
	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();	

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
	
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	RobotIndices_mmap sorted_mdd_Robots;

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];

	    sorted_mdd_Robots.insert(RobotIndices_mmap::value_type(robot_cost, mdd_robot_id));
	}

	int sort_index = 0;
	for (RobotIndices_mmap::const_reverse_iterator sort_robot = sorted_mdd_Robots.rbegin(); sort_robot != sorted_mdd_Robots.rend(); ++sort_robot)
	{
	    int add_index = (sort_index++ * (sizeof(sMDD_Addition) / sizeof(int))) / N_Robots;
	    int extra_addition = sMDD_Addition[add_index];

	    int mdd_robot_id = sort_robot->second;
	    
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    
	    int robot_cost = source_Distances[robot_source_vertex_id][robot_sink_vertex_id];
/*
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= robot_cost + extra_cost - mdd_level)
		    {
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
			printf("mdd_level:%d\n", mdd_level);
		    }
		}
	    }
*/	    

	    RobotMDD_set robot_MDD;
	    robot_MDD.resize(mdd_depth + 1);
		
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int mdd_level = source_Distances[robot_source_vertex_id][vertex_id];		     
		     mdd_level <= sMIN(robot_cost + (extra_cost + extra_addition) - goal_Distances[robot_sink_vertex_id][vertex_id], mdd_depth);
		     ++mdd_level)
		{		    
		    if (m_initial_arrangement.m_robot_Sizes[mdd_robot_id] > 1)
		    {
			if (is_Fitting(graph, mdd_robot_id, m_initial_arrangement.m_robot_Sizes[mdd_robot_id], m_initial_arrangement.m_robot_Sizes[mdd_robot_id], vertex_id))
			{
			    robot_MDD[mdd_level].insert(vertex_id);
			}			    
		    }
		    else
		    {
			robot_MDD[mdd_level].insert(vertex_id);
		    }
		}
	    }
	    
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		if (robot_MDD[mdd_level].empty())
		{
		    if (m_initial_arrangement.m_robot_Sizes[mdd_robot_id] > 1)
		    {
			if (is_Fitting(graph, mdd_robot_id, m_initial_arrangement.m_robot_Sizes[mdd_robot_id], m_initial_arrangement.m_robot_Sizes[mdd_robot_id], robot_sink_vertex_id))
			{
			    robot_MDD[mdd_level].insert(robot_sink_vertex_id);
			}
		    }
		    else
		    {
			robot_MDD[mdd_level].insert(robot_sink_vertex_id);
		    }		    
		}
		if (   mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id]
		    && mdd_level < source_Distances[robot_source_vertex_id][robot_sink_vertex_id] + (extra_cost + extra_addition))
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
		}
	    }

	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (VertexIDs_set::const_iterator vertex_ID = robot_MDD[mdd_level].begin(); vertex_ID != robot_MDD[mdd_level].end(); ++vertex_ID)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(*vertex_ID);
		}
	    }
	}
/*
	printf("Distance printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    printf("robot:%d (%d,%d)\n", mdd_robot, robot_source_vertex_id, robot_sink_vertex_id);
	    
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int  source_dist = source_Distances[robot_source_vertex_id][vertex_id];
		int  goal_dist = goal_Distances[robot_sink_vertex_id][vertex_id];

		printf("  %d:%d,%d\n", vertex_id, source_dist, goal_dist);
	    }	   
	}
	printf("<----\n");
	
	printf("MDD printout\n");
	for (int mdd_robot = 1; mdd_robot <= N_Robots; ++mdd_robot)
	{	    
	    printf("robot:%d\n", mdd_robot);
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
		for (int i = 0; i < MDD[mdd_robot][mdd_level].size(); ++i)
		{		    
		    printf("%d ", MDD[mdd_robot][mdd_level][i]);
		}
		printf("\n");
	    }
	    printf("\n");
	}
	printf("<----\n");		
*/	
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
	return mdd_depth;
    }



    bool sMultirobotInstance::is_Fitting(sUndirectedGraph &graph, int sUNUSED(robot_id), int robot_row_size, int robot_column_size, int vertex_id)
    {	
	for (int dr = 0; dr < robot_row_size; ++dr)
	{
	    for (int dc = 0; dc < robot_column_size; ++dc)
	    {
		int grid_vertex_id = graph.get_GridNeighborVertexID(vertex_id, dr, dc);
		
		if (grid_vertex_id < 0)
		{
		    return false;
		}
	    }						
	}
	return true;
    }


    void sMultirobotInstance::reduce_MDD(const Arrangements_vector &unfolded_solution, const MDD_vector &MDD, MDD_vector &reduced_MDD)
    {
	reduced_MDD.clear();
	
	int N_Robots = MDD.size() - 1;
	sASSERT(!MDD.empty() && MDD.size() > 1);
	int mdd_depth = MDD[1].size() - 1;
	sASSERT(mdd_depth >= 1);
	reduced_MDD.resize(MDD.size());

	Arrangements_vector addup_unfolded_solution = unfolded_solution;
	
	if (addup_unfolded_solution.size() <= mdd_depth)
	{
	    int N_addups = mdd_depth - addup_unfolded_solution.size();
	    sRobotArrangement last_arrangement = addup_unfolded_solution[addup_unfolded_solution.size() - 1];

	    do
	    {
		addup_unfolded_solution.push_back(last_arrangement);
	    }
	    while (N_addups-- > 0);
	}

#ifdef sVERBOSE
	{
	    printf("Reducing MDD\n");
	}
#endif
	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
#ifdef sVERBOSE
	    {
		printf("robot:%d (%ld)\n", mdd_robot_id, MDD[mdd_robot_id].size());
	    }
#endif
	    reduced_MDD[mdd_robot_id].resize(MDD[mdd_robot_id].size());
	
	    for (int u = 0; u < MDD[mdd_robot_id][0].size(); ++u)
	    {
		if (   addup_unfolded_solution[0].is_VertexUnforced(MDD[mdd_robot_id][0][u])
		    && addup_unfolded_solution[1].is_VertexUnforced(MDD[mdd_robot_id][0][u]))
		{
#ifdef sVERBOSE
		    {
			printf("+%d ", MDD[mdd_robot_id][0][u]);
		    }
#endif
		    reduced_MDD[mdd_robot_id][0].push_back(MDD[mdd_robot_id][0][u]);
		}
		else
		{
#ifdef sVERBOSE
		    {
			printf("-%d ", MDD[mdd_robot_id][0][u]);
		    }
#endif
		}
	    }
	    
	    for (int mdd_level = 1; mdd_level < mdd_depth; ++mdd_level)
	    {
#ifdef sVERBOSE
		{		   
		    printf("level:%d\n", mdd_level);
		}
#endif
		for (int u = 0; u < MDD[mdd_robot_id][mdd_level].size(); ++u)
		{
		    if (   addup_unfolded_solution[mdd_level + 1].is_VertexUnforced(MDD[mdd_robot_id][mdd_level][u])
			&& addup_unfolded_solution[mdd_level].is_VertexUnforced(MDD[mdd_robot_id][mdd_level][u])
			&& addup_unfolded_solution[mdd_level - 1].is_VertexUnforced(MDD[mdd_robot_id][mdd_level][u]))
		    {
#ifdef sVERBOSE
			{
			    printf("+%d ", MDD[mdd_robot_id][mdd_level][u]);
			}
#endif
			reduced_MDD[mdd_robot_id][mdd_level].push_back(MDD[mdd_robot_id][mdd_level][u]);
		    }
		    else
		    {
#ifdef sVERBOSE
			{
			    printf("-%d ", MDD[mdd_robot_id][mdd_level][u]);
			}
#endif
		    }
		}
	    }
	    
	    for (int u = 0; u < MDD[mdd_robot_id][mdd_depth].size(); ++u)
	    {
		if (   addup_unfolded_solution[mdd_depth - 1].is_VertexUnforced(MDD[mdd_robot_id][mdd_depth][u])
		    && addup_unfolded_solution[mdd_depth].is_VertexUnforced(MDD[mdd_robot_id][mdd_depth][u]))
		{
#ifdef sVERBOSE
		    {
			printf("+%d ", MDD[mdd_robot_id][mdd_depth][u]);
		    }
#endif
		    reduced_MDD[mdd_robot_id][mdd_depth].push_back(MDD[mdd_robot_id][mdd_depth][u]);
		}
		else
		{
#ifdef sVERBOSE
		    {
			printf("-%d ", MDD[mdd_robot_id][mdd_depth][u]);
		    }
#endif
		}
	    }
	}
    }    


    struct MDDPoint
    {
	MDDPoint(int level, int vertex_id)
	{
	    m_level = level;
	    m_vertex_id = vertex_id;
	}
	    
	int m_level;
	int m_vertex_id;
    };

    typedef std::list<MDDPoint> Queue_vector;
    
    
    bool sMultirobotInstance::check_Connectivity(const MDD_vector &MDD) const
    {
	int N_Robots = MDD.size() - 1;
	sASSERT(!MDD.empty() && MDD.size() > 1);
	int mdd_depth = MDD[1].size() - 1;
	sASSERT(mdd_depth >= 1);

	MDD_vector scheduled_MDD = MDD;	

#ifdef sVERBOSE
	{
	    printf("Checking MDD connectivity ...\n");
	}
#endif
	
	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    if (MDD[mdd_robot_id][0].empty() || MDD[mdd_robot_id][mdd_depth].empty())
	    {
#ifdef sVERBOSE
		{
		    printf("  Unaccessible source or destination\n");
		}
#endif		
		return false;
	    }
	    
#ifdef sVERBOSE
	    {
		printf("Goal id: %d (at level %d)\n", MDD[mdd_robot_id][mdd_depth][0], mdd_depth);
	    }
#endif
	    scheduled_MDD[mdd_robot_id][0][0] = -1;
	    Queue_vector mdd_Queue;

	    mdd_Queue.push_back(MDDPoint(0, MDD[mdd_robot_id][0][0]));

	    while (!mdd_Queue.empty())
	    {
		MDDPoint mdd_point = mdd_Queue.front();
		mdd_Queue.pop_front();
		
#ifdef sVERBOSE
		{
		    printf("mdd point:%d,%d\n", mdd_point.m_level,  mdd_point.m_vertex_id);
		}
#endif		
		if (mdd_point.m_level == mdd_depth && mdd_point.m_vertex_id == MDD[mdd_robot_id][mdd_depth][0])
		{
#ifdef sVERBOSE
		    {
			printf(" Connected\n");
		    }
#endif					    
		    return true;
		}
		if (mdd_point.m_level < mdd_depth)
		{
		    for (int v = 0; v < MDD[mdd_robot_id][mdd_point.m_level + 1].size(); ++v)
		    {
			if (scheduled_MDD[mdd_robot_id][mdd_point.m_level + 1][v] >= 0)
			{
			    if (mdd_point.m_vertex_id == MDD[mdd_robot_id][mdd_point.m_level + 1][v] || m_environment.is_Adjacent(mdd_point.m_vertex_id, MDD[mdd_robot_id][mdd_point.m_level + 1][v]))
			    {
				mdd_Queue.push_back(MDDPoint(mdd_point.m_level + 1, MDD[mdd_robot_id][mdd_point.m_level + 1][v]));
				scheduled_MDD[mdd_robot_id][mdd_point.m_level + 1][v] = -1;
			    }
			}
		    }
		}
	    }	    
	}
#ifdef sVERBOSE
	{
	    printf("  Classically disconnected\n");
	}
#endif			
	return false;
    }


    void sMultirobotInstance::unify_MDD(const MDD_vector &MDD, RobotMDD_vector &unified_MDD)
    {
	int N_Robots = MDD.size() - 1;
	sASSERT(!MDD.empty());
	int mdd_depth = MDD[1].size();

	unified_MDD.resize(mdd_depth);

#ifdef sVERBOSE
	{
	    printf("Unifying... (%ld,%ld)\n", MDD.size(), MDD[1].size());
	}
#endif			
	
	for (int mdd_level = 0; mdd_level < mdd_depth; ++mdd_level)
	{	    
	    VertexIDs_set vertex_IDs;
	    
	    for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	    {
		const VertexIDs_vector &mdd_vertex_IDs = MDD[mdd_robot_id][mdd_level];
		for (VertexIDs_vector::const_iterator vertex = mdd_vertex_IDs.begin(); vertex != mdd_vertex_IDs.end(); ++vertex)
		{
		    vertex_IDs.insert(*vertex);
		}		   
	    }

#ifdef sVERBOSE
	    {
		printf("  unify level: %d: ", mdd_level);
	    }
#endif					
	    for (VertexIDs_set::const_iterator vertex = vertex_IDs.begin(); vertex != vertex_IDs.end(); ++vertex)
	    {
		#ifdef sVERBOSE
		{
		    printf("%d ", *vertex);
		}
#endif						    
		unified_MDD[mdd_level].push_back(*vertex);
	    }
#ifdef sVERBOSE
	    {
		printf("\n");
	    }
#endif						    
	}
    }


    void sMultirobotInstance::construct_InverseMDD(int N_Vertices, const MDD_vector &MDD, InverseMDD_vector &inverse_MDD)       
    {
	inverse_MDD.clear();

	int N_Robots = MDD.size() - 1;
	int mdd_depth = MDD[1].size();

	inverse_MDD.resize(N_Robots + 1);
	
	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    inverse_MDD[mdd_robot_id].resize(mdd_depth);

	    for (int mdd_level = 0; mdd_level < mdd_depth; ++mdd_level)
	    {
		inverse_MDD[mdd_robot_id][mdd_level].resize(N_Vertices, -1);
		
		for (int mdd_vertex_index = 0; mdd_vertex_index < MDD[mdd_robot_id][mdd_level].size(); ++mdd_vertex_index)
		{
		    inverse_MDD[mdd_robot_id][mdd_level][MDD[mdd_robot_id][mdd_level][mdd_vertex_index]] = mdd_vertex_index;
		}
	    }
	}
    }

    
    void sMultirobotInstance::construct_MDDIndices(const MDD_vector &MDD, MDDIndices_vector &MDD_Indices)
    {
	int N_Robots = MDD.size() - 1;
	int mdd_depth = MDD[1].size();

	MDD_Indices.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD_Indices[mdd_robot_id].resize(mdd_depth);

	    for (int mdd_level = 0; mdd_level < mdd_depth; ++mdd_level)
	    {
		for (int mdd_vertex_index = 0; mdd_vertex_index < MDD[mdd_robot_id][mdd_level].size(); ++mdd_vertex_index)
		{
		    MDD_Indices[mdd_robot_id][mdd_level][MDD[mdd_robot_id][mdd_level][mdd_vertex_index]] = mdd_vertex_index;
		}
	    }
	}
    }


    void sMultirobotInstance::construct_MDDIndices(const RobotMDD_vector &unified_MDD, RobotMDDIndices_vector &unified_MDD_Indices)
    {
	int mdd_depth = unified_MDD.size();
	unified_MDD_Indices.resize(mdd_depth);

	for (int mdd_level = 0; mdd_level < mdd_depth; ++mdd_level)
	{
	    for (int mdd_vertex_index = 0; mdd_vertex_index < unified_MDD[mdd_level].size(); ++mdd_vertex_index)
	    {
		unified_MDD_Indices[mdd_level][unified_MDD[mdd_level][mdd_vertex_index]] = mdd_vertex_index;
	    }
	}
    }

    
    int sMultirobotInstance::construct_NoMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphMDD(m_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }

    
    int sMultirobotInstance::construct_SparseNoMDD(int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	return construct_GraphMDD(m_sparse_environment, max_total_cost, MDD, extra_cost, extra_MDD);
    }

    
    int sMultirobotInstance::construct_GraphNoMDD(sUndirectedGraph &graph, int max_total_cost, MDD_vector &MDD, int &extra_cost, MDD_vector &extra_MDD)
    {
	int max_individual_cost;
	int min_total_cost = estimate_TotalCost(max_individual_cost);

	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;
	collect_Endpoints(source_IDs, goal_IDs);
	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
//	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();

	extra_cost = max_total_cost - min_total_cost;
	int mdd_depth = max_individual_cost + extra_cost;
/*
	printf("mdd_depth:%d\n", mdd_depth);
	printf("extra_cost:%d\n", extra_cost);
*/
	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.clear();

	MDD.resize(N_Robots + 1);
	extra_MDD.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
//	    printf("mdd_robot:%d\n", mdd_robot_id);
	    MDD[mdd_robot_id].resize(mdd_depth + 1);
	    extra_MDD[mdd_robot_id].resize(mdd_depth + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
  
	    for (int mdd_level = 0; mdd_level <= mdd_depth; ++mdd_level)
	    {
//		printf("level %d (%d):", mdd_level, mdd_depth);
		int N_Vertices = graph.get_VertexCount();
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
//		    printf("%d ", vertex_id);
		}
		if (mdd_level >= source_Distances[robot_source_vertex_id][robot_sink_vertex_id])
		{
		    extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		    printf(" * ");
		}
//		extra_MDD[mdd_robot_id][mdd_level].push_back(robot_sink_vertex_id);
//		printf("\n");
	    }
	}
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}
	
	return mdd_depth;
    }


    void sMultirobotInstance::construct_MakespanMDD(int max_makespan, MDD_vector &MDD)
    {
	construct_GraphMakespanMDD(m_environment, max_makespan, MDD);
    }

    
    void sMultirobotInstance::construct_SparseMakespanMDD(int max_makespan, MDD_vector &MDD)
    {
	construct_GraphMakespanMDD(m_sparse_environment, max_makespan, MDD);
    }
    

    void sMultirobotInstance::construct_GraphMakespanMDD(sUndirectedGraph &graph, int max_makespan, MDD_vector &MDD)
    {
	VertexIDs_vector source_IDs;
	VertexIDs_vector goal_IDs;

	MDD.clear();
	
	collect_Endpoints(source_IDs, goal_IDs);
	graph.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = graph.get_SourceShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &goal_Distances = graph.get_GoalShortestPaths();

	int N_Robots = m_initial_arrangement.get_RobotCount();

	MDD.resize(N_Robots + 1);

	for (int mdd_robot_id = 1; mdd_robot_id <= N_Robots; ++mdd_robot_id)
	{
	    MDD[mdd_robot_id].resize(max_makespan + 1);

	    int robot_source_vertex_id = m_initial_arrangement.get_RobotLocation(mdd_robot_id);
	    int robot_sink_vertex_id;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		robot_sink_vertex_id = m_goal_arrangement.get_RobotLocation(mdd_robot_id);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(mdd_robot_id);
		sASSERT(goal_IDs.size() == 1);
		robot_sink_vertex_id = *goal_IDs.begin();
		break;
	    }
	    default:
	    {
		break;
	    }
	    }

#ifdef sVERBOSE
	    printf("Robot:%d\n", mdd_robot_id);
#endif
	    for (int mdd_level = 0; mdd_level <= max_makespan; ++mdd_level)
	    {
#ifdef sVERBOSE
		printf("mdd level %d,%d: ", mdd_level, max_makespan);
#endif
		int N_Vertices = graph.get_VertexCount();
		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    if (   source_Distances[robot_source_vertex_id][vertex_id] <= mdd_level
			&& goal_Distances[robot_sink_vertex_id][vertex_id] <= max_makespan - mdd_level)
		    {
#ifdef sVERBOSE
			printf("%d ", vertex_id);
#endif
			MDD[mdd_robot_id][mdd_level].push_back(vertex_id);
		    }
		}
#ifdef sVERBOSE
		printf("\n");
#endif
	    }
#ifdef sVERBOSE
	    printf("\n");
#endif
	}
	
	std::vector<int> distribution;
	distribution.resize(N_Robots + 1);
	for (int robot_id = 0; robot_id <= N_Robots; ++robot_id)
	{
	    distribution[robot_id] = 0;
	}	
    }


/*----------------------------------------------------------------------------*/

    void sMultirobotInstance::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_multirobot(const sString &indent) const
    {
	to_Stream_multirobot(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_capacitated_multirobot(const sString &indent) const
    {
	to_Stream_capacitated_multirobot(stdout, indent);
    }    


    void sMultirobotInstance::to_Screen_domainPDDL(const sString &indent) const
    {
	to_Stream_domainPDDL(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_problemPDDL(const sString &indent) const
    {
	to_Stream_problemPDDL(stdout, indent);
    }


    void sMultirobotInstance::to_Screen_bgu(const sString &indent, int instance_id) const
    {
	to_Stream_bgu(stdout, indent, instance_id);
    }


    sResult sMultirobotInstance::to_File(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_capacitated_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream_capacitated_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotInstance::to_File_domainPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_PDDL_OPEN_ERROR;
	}
	to_Stream_domainPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_problemPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_PDDL_OPEN_ERROR;
	}
	to_Stream_problemPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_bgu(const sString &filename, const sString &indent, int instance_id) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_BGU_OPEN_ERROR;
	}
	to_Stream_bgu(fw, indent, instance_id);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotInstance::to_Stream(FILE *fw, const sString &indent) const
    {       
	fprintf(fw, "%sMultirobot instance: [\n", indent.c_str());
	fprintf(fw, "%s%sEnvironment:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_environment.to_Stream_vertices(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	fprintf(fw, "%s%sInitial arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_initial_arrangement.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    fprintf(fw, "%s%sGoal arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	    m_goal_arrangement.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    fprintf(fw, "%s%sGoal specification:\n", indent.c_str(), sRELOC_INDENT.c_str());
	    m_goal_specification.to_Stream(fw, indent + sRELOC_INDENT + sRELOC_INDENT);
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());
	
	int N_Vertices;
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    N_Vertices = m_initial_arrangement.m_vertex_Occups.size();
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    N_Vertices = m_goal_specification.m_goal_Compats.size();
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	for (int i = 0; i < N_Vertices; ++i)
	{
	    
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		fprintf(fw, "(%d:-1)[%d:%d:%d]", i,
			m_initial_arrangement.m_vertex_Occups[i],
			m_goal_arrangement.m_vertex_Occups[i],
			m_goal_arrangement.m_vertex_Occups[i]);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.m_goal_Compats[i].empty())
		{
		    fprintf(fw, "(%d:-1)[%d:0:0]", i,
			    m_initial_arrangement.m_vertex_Occups[i]);
		}
		else if (m_goal_specification.m_goal_Compats[i].size() == 1)
		{
		    fprintf(fw, "(%d:-1)[%d:%d:%d]", i,
			    m_initial_arrangement.m_vertex_Occups[i],
			    *m_goal_specification.m_goal_Compats[i].begin(),
			    *m_goal_specification.m_goal_Compats[i].begin());
		}		
		else
		{
		    fprintf(fw, "(%d:-1)[%d:", i, m_initial_arrangement.m_vertex_Occups[i]);
		    m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		    fprintf(fw, ":");
		    m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		    fprintf(fw, "]");
		}		
		break;
	    }	    
	    default:
	    {
		break;
	    }
	    }
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (int c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_multirobot(fw, indent);
    }


    void sMultirobotInstance::to_Stream_capacitated_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sV =\n", indent.c_str());

	// TODO
	int N_Vertices;
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    N_Vertices = m_initial_arrangement.m_vertex_Occups.size();
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    N_Vertices = m_goal_specification.m_goal_Compats.size();
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	for (int i = 0; i < N_Vertices; ++i)
	{
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		fprintf(fw, "(%d:-1:%d)[%d:%d:%d]", i,
			m_environment.m_Vertices[i].m_capacity,
			m_initial_arrangement.m_vertex_Occups[i],
			m_goal_arrangement.m_vertex_Occups[i],
			m_goal_arrangement.m_vertex_Occups[i]);
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.m_goal_Compats[i].empty())
		{
		    fprintf(fw, "(%d:-1:%d)[%d:0:0]", i,
			    m_environment.m_Vertices[i].m_capacity,
			    m_initial_arrangement.m_vertex_Occups[i]);
		}
		else if (m_goal_specification.m_goal_Compats[i].size() == 1)
		{
		    fprintf(fw, "(%d:-1:%d)[%d:%d:%d]", i,
			    m_environment.m_Vertices[i].m_capacity,
			    m_initial_arrangement.m_vertex_Occups[i],
			    *m_goal_specification.m_goal_Compats[i].begin(),
			    *m_goal_specification.m_goal_Compats[i].begin());
		}		
		else
		{
		    fprintf(fw, "(%d:-1:%d)[%d:", i, m_environment.m_Vertices[i].m_capacity, m_initial_arrangement.m_vertex_Occups[i]);
		    m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		    fprintf(fw, ":");
		    m_goal_specification.to_Stream_multirobot(fw, m_goal_specification.m_goal_Compats[i]);
		    fprintf(fw, "]");
		}		
		break;
	    }
	    default:
	    {
		break;
	    }
	    }
	    if (m_environment.m_Vertices[i].m_Conflicts.empty())
	    {
		fprintf(fw, "\n");
	    }
	    else
	    {
		fprintf(fw, "< ");
		for (int c = 0; c < m_environment.m_Vertices[i].m_Conflicts.size(); ++c)
		{
		    fprintf(fw, "%d ", m_environment.m_Vertices[i].m_Conflicts[c]);
		}			 
		fprintf(fw, ">\n");
	    }
	}
	m_environment.to_Stream_capacitated_multirobot(fw, indent);
    }    


    void sMultirobotInstance::to_Stream_domainPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (domain multirobot)\n", indent.c_str());
	fprintf(fw, "%s%s(:predicates\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(robot_location ?r ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(no_robot ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:action move\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:parameters (?r ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:precondition (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(robot_location ?r ?u) (no_robot ?v) (adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:effect (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(robot_location ?r ?v) (no_robot ?u) (not (no_robot ?v))\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_problemPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (problem multirobot_instance)\n", indent.c_str());
	fprintf(fw, "%s%s(:domain multirobot)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:objects\n", indent.c_str(), sRELOC_INDENT.c_str());

	int N_Vertices = m_environment.get_VertexCount();
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    fprintf(fw, "%s%s%sv%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	}

	int N_Robots = m_initial_arrangement.get_RobotCount();
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    fprintf(fw, "%s%s%sr%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id);
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:init\n", indent.c_str(), sRELOC_INDENT.c_str());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_vu.m_target->m_id, edge->m_arc_uv.m_target->m_id);
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_uv.m_target->m_id, edge->m_arc_vu.m_target->m_id);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    fprintf(fw, "%s%s%s(robot_location r%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id, m_initial_arrangement.get_RobotLocation(robot_id));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (robot_id == sRobotArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%s%s(robot_location r%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), robot_id, m_goal_arrangement.get_RobotLocation(robot_id));
	    }
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		
		if (robot_id == sRobotArrangement::VACANT_VERTEX)
		{
		    fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
		}
	    }
	    fprintf(fw, "%s%s))\n", indent.c_str(), sRELOC_INDENT.c_str());
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
		fprintf(fw, "%s%s%s(or ", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
		for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
		{
		    fprintf(fw, "(robot_location r%d v%d) ", robot_id, *goal_id);
		}
		fprintf(fw, ")\n");
	    }
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (robot_IDs.empty())
		{
		    fprintf(fw, "%s%s%s(no_robot v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
		}
	    }
	    fprintf(fw, "%s%s))\n", indent.c_str(), sRELOC_INDENT.c_str());
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sMultirobotInstance::to_Stream_bgu(FILE *fw, const sString &indent, int instance_id) const
    {
	sASSERT(m_environment.m_Matrix != NULL);

	fprintf(fw, "%s%d\n", indent.c_str(), instance_id);
	fprintf(fw, "%sGrid:\n", indent.c_str());
	fprintf(fw, "%s%d,%d\n", indent.c_str(), m_environment.m_y_size, m_environment.m_x_size);

	for (int j = 0; j < m_environment.m_y_size; ++j)
	{
	    for (int i = 0; i < m_environment.m_x_size; ++i)
	    {
		if (m_environment.m_Matrix[j * m_environment.m_x_size + i] >= 0)
		{
		    fprintf(fw, ".");
		}
		else
		{
		    fprintf(fw, "@");
		}
	    }
	    fprintf(fw, "\n");
	}
	int N_Robots = m_initial_arrangement.get_RobotCount();

	fprintf(fw, "%sAgents:\n", indent.c_str());
	fprintf(fw, "%s%d\n", indent.c_str(), N_Robots);

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%d,", indent.c_str(), robot_id - 1);

		int goal_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));

		int init_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d\n", m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		fprintf(fw, "%s%d,", indent.c_str(), robot_id - 1);

		sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		int goal_vertex_id = *m_goal_specification.get_RobotGoal(robot_id).begin();
		fprintf(fw, "%d,%d,",  m_environment.calc_GridRow(goal_vertex_id), m_environment.calc_GridColumn(goal_vertex_id));

		int init_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "%d,%d\n",  m_environment.calc_GridRow(init_vertex_id), m_environment.calc_GridColumn(init_vertex_id));
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


    sResult sMultirobotInstance::from_File_bgu(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_BGU_OPEN_ERROR;
	}
	
	result = from_Stream_bgu(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_Stream_bgu(FILE *fr)
    {
	sResult result;
	int N_Robots;
	
	if (sFAILED(result = m_environment.from_Stream_bgu(fr)))
	{
	    return result;
	}

	fscanf(fr, "Agents:\n");
	fscanf(fr, "%d\n", &N_Robots);

	m_goal_type = GOAL_TYPE_ARRANGEMENT;

	m_initial_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_specification = sRobotGoal(m_environment.get_VertexCount(), N_Robots);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int r_id;
	    fscanf(fr, "%d,", &r_id);
	    
	    int goal_row, goal_column;
	    fscanf(fr, "%d,%d,", &goal_row, &goal_column);

	    int goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);
	    
	    int init_row, init_column;
	    fscanf(fr, "%d,%d\n", &init_row, &init_column);
	    int init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    m_initial_arrangement.place_Robot(robot_id, init_vertex_id);
	    m_goal_arrangement.place_Robot(robot_id, goal_vertex_id);
	    m_goal_specification.charge_Robot(robot_id, goal_vertex_id);
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_File_usc(const sString &map_filename, const sString &agents_filename)
    {
	sResult result;
	FILE *fr_map, *fr_agents;

	if ((fr_map = fopen(map_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_USC_MAP_OPEN_ERROR;
	}
	if ((fr_agents = fopen(agents_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_USC_AGNT_OPEN_ERROR;
	}
	
	result = from_Stream_usc(fr_map, fr_agents);
	
	if (sFAILED(result))
	{
	    fclose(fr_map);
	    fclose(fr_agents);	    
	    return result;
	}
	fclose(fr_map);
	fclose(fr_agents);	

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_Stream_usc(FILE *fr_map, FILE *fr_agents)
    {
	sResult result;
	int N_Robots;

	if (sFAILED(result = m_environment.from_Stream_usc(fr_map)))
	{
	    return result;
	}
	fscanf(fr_agents, "%d\n", &N_Robots);

	m_goal_type = GOAL_TYPE_ARRANGEMENT;

	m_initial_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_specification = sRobotGoal(m_environment.get_VertexCount(), N_Robots);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{	    
	    int goal_row, goal_column;
	    fscanf(fr_agents, "%d,%d,", &goal_row, &goal_column);

	    int goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);

	    int init_row, init_column;
	    fscanf(fr_agents, "%d,%d,", &init_row, &init_column);
	    int init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    float delay_ignore;
	    fscanf(fr_agents, "%f\n", &delay_ignore);	    

	    m_initial_arrangement.place_Robot(robot_id, init_vertex_id);
	    m_goal_arrangement.place_Robot(robot_id, goal_vertex_id);
	    m_goal_specification.charge_Robot(robot_id, goal_vertex_id);
	}	
	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_File_lusc(const sString &map_filename, const sString &agents_filename)
    {
	sResult result;
	FILE *fr_map, *fr_agents;

	if ((fr_map = fopen(map_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_USC_MAP_OPEN_ERROR;
	}
	if ((fr_agents = fopen(agents_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_USC_AGNT_OPEN_ERROR;
	}
	
	result = from_Stream_lusc(fr_map, fr_agents);
	
	if (sFAILED(result))
	{
	    fclose(fr_map);
	    fclose(fr_agents);	    
	    return result;
	}
	fclose(fr_map);
	fclose(fr_agents);	

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_Stream_lusc(FILE *fr_map, FILE *fr_agents)
    {
	sResult result;
	int N_Robots;

	if (sFAILED(result = m_environment.from_Stream_lusc(fr_map)))
	{
	    return result;
	}
	fscanf(fr_agents, "%d\n", &N_Robots);

	m_goal_type = GOAL_TYPE_ARRANGEMENT;

	m_initial_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_specification = sRobotGoal(m_environment.get_VertexCount(), N_Robots);

	m_initial_arrangement.m_robot_Sizes.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{	    
	    int init_row, init_column;
	    fscanf(fr_agents, "%d,%d,", &init_row, &init_column);
	    int init_vertex_id =  m_environment.calc_GridVertexID(init_row, init_column);

	    int goal_row, goal_column;
	    fscanf(fr_agents, "%d,%d,", &goal_row, &goal_column);
	    int goal_vertex_id = m_environment.calc_GridVertexID(goal_row, goal_column);	    

	    int robot_size;
	    fscanf(fr_agents, "%d\n", &robot_size);
	    m_initial_arrangement.m_robot_Sizes[robot_id] = robot_size;

	    m_initial_arrangement.place_Robot(robot_id, init_vertex_id);
	    m_goal_arrangement.place_Robot(robot_id, goal_vertex_id);
	    m_goal_specification.charge_Robot(robot_id, goal_vertex_id);
	}	
	
	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotInstance::to_File_dibox(const sString &filename)
    {
	sResult result;
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_DIBOX_OPEN_ERROR;
	}
	
	result = to_Stream_dibox(fw);
	if (sFAILED(result))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Stream_dibox(FILE *fw)
    {
	int N_Robots = m_initial_arrangement.get_RobotCount();

	m_environment.to_Stream_dibox(fw);
	fprintf(fw, "LIST OF AGENTS:%d\n", N_Robots);
	
	m_goal_type = GOAL_TYPE_ARRANGEMENT;

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int init_vertex_id, goal_vertex_id;

	    init_vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    goal_vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
	    
	    fprintf(fw, "(q%d,%d,%d)\n", robot_id, init_vertex_id + 1, goal_vertex_id + 1);
	}

	return sRESULT_SUCCESS;
    }
    
    
    sResult sMultirobotInstance::from_File_dibox(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_DIBOX_OPEN_ERROR;
	}
	
	result = from_Stream_dibox(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::from_Stream_dibox(FILE *fr)
    {
	sResult result;
	int N_Robots;

	if (sFAILED(result = m_environment.from_Stream_dibox(fr)))
	{
	    return result;
	}
	fscanf(fr, "LIST OF AGENTS:%d\n", &N_Robots);
	
	m_goal_type = GOAL_TYPE_ARRANGEMENT;

	m_initial_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_arrangement = sRobotArrangement(m_environment.get_VertexCount(), N_Robots);
	m_goal_specification = sRobotGoal(m_environment.get_VertexCount(), N_Robots);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int init_vertex_id, goal_vertex_id;

	    while(fgetc(fr) != ',');
	    fscanf(fr, "%d,%d)\n", &init_vertex_id, &goal_vertex_id);
	    
	    m_initial_arrangement.place_Robot(robot_id, init_vertex_id - 1);
	    m_goal_arrangement.place_Robot(robot_id, goal_vertex_id - 1);
	    m_goal_specification.charge_Robot(robot_id, goal_vertex_id - 1);
	}

	return sRESULT_SUCCESS;
    }    

    
/*----------------------------------------------------------------------------*/
// sMultirobotSolution

    const sMultirobotSolution::Move sMultirobotSolution::UNDEFINED_MOVE = sMultirobotSolution::Move(0, 0, 0);


/*----------------------------------------------------------------------------*/

    sMultirobotSolution::Move::Move(int robot_id, int src_vrtx_id, int dest_vrtx_id)
	: m_robot_id(robot_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(0)
    {
	// nothing
    }


    sMultirobotSolution::Move::Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, int crt_time)
	: m_robot_id(robot_id)
	, m_src_vrtx_id(src_vrtx_id)
	, m_dest_vrtx_id(dest_vrtx_id)
	, m_crt_time(crt_time)
    {
	// nothing
    }


    bool sMultirobotSolution::Move::is_Undefined(void) const
    {
	return (m_robot_id <= 0);
    }


    bool sMultirobotSolution::Move::is_Dependent(const Move &move) const
    {
	return (   m_src_vrtx_id == move.m_src_vrtx_id
		|| m_src_vrtx_id == move.m_dest_vrtx_id
		|| m_dest_vrtx_id == move.m_src_vrtx_id
		|| m_dest_vrtx_id == move.m_dest_vrtx_id);
    }



/*----------------------------------------------------------------------------*/

    sMultirobotSolution::Step::Step(int time)
	: m_time(time)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolution::sMultirobotSolution()
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	// nothing
    }


    sMultirobotSolution::sMultirobotSolution(int start_step, const sMultirobotSolution &sub_solution)
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	int N_Steps = sub_solution.m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = sub_solution.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i + start_step, *move);
	    }
	}	
    }


    sMultirobotSolution::sMultirobotSolution(const sMultirobotSolution &sub_solution_1, const sMultirobotSolution sub_solution_2)
	: m_Moves_cnt(0)
	, m_optimality_ratio(-1.0)
    {
	int N_Steps_1 = sub_solution_1.m_Steps.size();
	for (int i = 0; i < N_Steps_1; ++i)
	{
	    const Step &step = sub_solution_1.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}

	int N_Steps_2 = sub_solution_2.m_Steps.size();
	for (int i = 0; i < N_Steps_2; ++i)
	{
	    const Step &step = sub_solution_2.m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		add_Move(i, *move);
	    }
	}
    }

    bool sMultirobotSolution::is_Null(void) const
    {
	return (m_Steps.empty());
    }


    int sMultirobotSolution::get_MoveCount(void) const
    {
	return m_Moves_cnt;
    }


    int sMultirobotSolution::get_StepCount(void) const
    {
	return m_Steps.size();
    }


    void sMultirobotSolution::add_Move(int time, const Move &move)
    {
	int push_cnt = time - m_Steps.size();

	while (push_cnt-- >= 0)
	{
	    m_Steps.push_back(Step(m_Steps.size()));
	}
	m_Steps[time].m_Moves.push_back(move);
	++m_Moves_cnt;
    }


    int sMultirobotSolution::calc_EmptySteps(void) const
    {
	int N_empty_Steps = 0;
	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    if (step.m_Moves.empty())
	    {
		++N_empty_Steps;
	    }
	}
	return N_empty_Steps;
    }


    void sMultirobotSolution::remove_EmptySteps(void)
    {
	int time = 0;
	Steps_vector clean_Steps;

	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    Step &step = m_Steps[i];

	    if (!step.m_Moves.empty())
	    {
		step.m_time = time++;
		clean_Steps.push_back(step);
	    }
	}
	m_Steps = clean_Steps;
    }


    sMultirobotSolution sMultirobotSolution::extract_Subsolution(int start_step, int final_step) const
    {
	sMultirobotSolution subsolution;

	for (int i = start_step; i <= final_step; ++i)
	{
	    const Step &step = m_Steps[i];

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		subsolution.add_Move(i, *move);
	    }
	}	

	return subsolution;
    }


    void sMultirobotSolution::execute_Solution(const sRobotArrangement &initial_arrangement,
					       sRobotArrangement       &final_arrangement,
					       int                      N_Steps) const
    {
	final_arrangement = initial_arrangement;

	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sMultirobotSolution::execute_Step(const sRobotArrangement &current_arrangement,
					   sRobotArrangement       &final_arrangement,
					   int                      step_idx) const
    {
	final_arrangement = current_arrangement;

	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    final_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	}	    
    }


    void sMultirobotSolution::execute_Solution(sRobotArrangement &arrangement,
					       int                N_Steps) const
    {
	if (N_Steps == N_STEPS_UNDEFINED)
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
	    }
	}
	else
	{
	    for (sMultirobotSolution::Steps_vector::const_iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		}	    
		if (--N_Steps <= 0)
		{
		    return;
		}
	    }
	}
    }


    void sMultirobotSolution::execute_Step(sRobotArrangement &arrangement,
					   int                step_idx) const
    {
	 const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	}	    
    }


    bool sMultirobotSolution::verify_Step(const sRobotArrangement &arrangement,
					  int                      step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!arrangement.verify_Move(move->m_robot_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    bool sMultirobotSolution::check_Step(const sRobotArrangement &arrangement,
					 int                      step_idx) const
    {
	const Step &step = m_Steps[step_idx];

	for (sMultirobotSolution::Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	{
	    if (!arrangement.check_Move(move->m_robot_id, move->m_dest_vrtx_id))
	    {
		return false;
	    }
	}
	return true;
    }


    void sMultirobotSolution::filter_Solution(const sRobotArrangement &initial_arrangement,
					      const sRobotGoal        &goal_arrangement,
					      sMultirobotSolution     &filter_solution) const
    {
	sRobotArrangement robot_arrangement = initial_arrangement;

	for (sMultirobotSolution::Steps_vector::const_iterator step = m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		robot_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
		filter_solution.add_Move(step->m_time, *move);

		if (goal_arrangement.is_Satisfied(robot_arrangement))
		{
		    break;
		}
	    }	    
	}
    }


    int sMultirobotSolution::calc_CriticalTimes(void)
    {
	int max_crt_time = 0;

	for (sMultirobotSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		move->m_crt_time = 0;

		for (sMultirobotSolution::Steps_vector::const_iterator prev_step =  m_Steps.begin(); prev_step != step; ++prev_step)
		{
		    for (sMultirobotSolution::Moves_list::const_iterator prev_move = prev_step->m_Moves.begin(); prev_move != prev_step->m_Moves.end(); ++prev_move)
		    {
			if (move->is_Dependent(*prev_move))
			{
			    if (prev_move->m_crt_time >= move->m_crt_time)
			    {
				move->m_crt_time = prev_move->m_crt_time + 1;
				max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			    }
			}
		    }
		}
		for (sMultirobotSolution::Moves_list::const_iterator prev_move = step->m_Moves.begin(); prev_move != move; ++prev_move)
		{
		    if (move->is_Dependent(*prev_move))
		    {
			if (prev_move->m_crt_time >= move->m_crt_time)
			{
			    move->m_crt_time = prev_move->m_crt_time + 1;
			    max_crt_time = sMAX(max_crt_time, move->m_crt_time);
			}
		    }
		}
	    }
	}

	return max_crt_time;
    }


    void sMultirobotSolution::criticalize_Solution(sMultirobotSolution &critical_solution)
    {
	calc_CriticalTimes();

	for (sMultirobotSolution::Steps_vector::iterator step =  m_Steps.begin(); step != m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		critical_solution.add_Move(move->m_crt_time, *move);
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    void sMultirobotSolution::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sMultirobotSolution::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMulirobot solution: (|moves| = %d, paralellism = %.3f) [\n", indent.c_str(), m_Moves_cnt, (double)m_Moves_cnt / m_Steps.size());

	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    fprintf(fw, "%s%sStep %d: ", indent.c_str(), sRELOC_INDENT.c_str(), step.m_time);

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%d#%d->%d ", move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id);
	    }
	    fprintf(fw, "\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sMultirobotSolution::to_Screen(const sUndirectedGraph &grid, const sString &indent) const
    {
	to_Stream(grid, stdout, indent);
    }


    void sMultirobotSolution::to_Stream(const sUndirectedGraph &grid, FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sMulirobot solution: (|moves| = %d, paralellism = %.3f) [\n", indent.c_str(), m_Moves_cnt, (double)m_Moves_cnt / m_Steps.size());

	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    fprintf(fw, "%s%sStep %d: ", indent.c_str(), sRELOC_INDENT.c_str(), step.m_time);

	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		int src_row = grid.calc_GridRow(move->m_src_vrtx_id);
		int src_column = grid.calc_GridColumn(move->m_src_vrtx_id);
		int dest_row = grid.calc_GridRow(move->m_dest_vrtx_id);
		int dest_column = grid.calc_GridColumn(move->m_dest_vrtx_id);
		    
		fprintf(fw, "%d#%d[%d,%d] --> %d[%d,%d] ", move->m_robot_id, move->m_src_vrtx_id, src_row, src_column, move->m_dest_vrtx_id, dest_row, dest_column);
	    }
	    fprintf(fw, "\n");
	}

	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sMultirobotSolution::to_File(const sUndirectedGraph &grid, const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_OPEN_ERROR;
	}
	to_Stream(grid, fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolution::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotSolution::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sFine solution\n", indent.c_str());
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%s%d # %d ---> %d (%d)\n", indent.c_str(), move->m_robot_id, move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
    }


    sResult sMultirobotSolution::to_File_graphrec(const sString &filename, const sMultirobotInstance &instance, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_OPEN_ERROR;
	}
	
	to_Stream_graphrec(fw, instance, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotSolution::to_Stream_graphrec(FILE *fw, const sMultirobotInstance &instance, const sString &indent) const
    {
	fprintf(fw, "id:-\n");
	instance.to_Stream_multirobot(fw, indent);
	
	fprintf(fw, "Solution\n");
	int N_Steps = m_Steps.size();
	for (int i = 0; i < N_Steps; ++i)
	{
	    const Step &step = m_Steps[i];
	    for (Moves_list::const_iterator move = step.m_Moves.begin(); move != step.m_Moves.end(); ++move)
	    {
		fprintf(fw, "%d ---> %d (%d)\n", move->m_src_vrtx_id, move->m_dest_vrtx_id, i);
	    }
	}
	fprintf(fw, "%sLength:%d\n", indent.c_str(), get_MoveCount());
    }    


    sResult sMultirobotSolution::from_File_multirobot(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_OPEN_ERROR;
	}
	
	result = from_Stream_multirobot(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolution::from_Stream_multirobot(FILE *fr)
    {
	int N_Moves;
	int c = fgetc(fr);

	while (c != 'F')
	{
	    if (c != '\n')
	    {
		while(fgetc(fr) != '\n');
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, "ine solution\nLength:%d\n", &N_Moves);
        #ifdef sDEBUG
	printf("Length:%d\n", N_Moves);
	#endif

	for (int i = 0; i < N_Moves; ++i)
	{
	    int robot_id, src_vertex_id, dest_vertex_id, step;
	    fscanf(fr, "%d # %d ---> %d (%d)\n", &robot_id, &src_vertex_id, &dest_vertex_id, &step);
	    #ifdef sDEBUG
	    printf("%d # %d ---> %d (%d)\n", robot_id, src_vertex_id, dest_vertex_id, step);
	    #endif
	    add_Move(step, Move(robot_id, src_vertex_id, dest_vertex_id));
	}

	return sRESULT_SUCCESS;
    }




/*----------------------------------------------------------------------------*/
// sMultirobotFlowModel

    sMultirobotFlowModel::sMultirobotFlowModel(const sMultirobotInstance &instance)
	: m_instance(instance)
    {
	// nothing
    }


    void sMultirobotFlowModel::build_Network(int N_steps)
    {
	for (int step = 0; step < N_steps; ++step)
	{
	    sString step_label = sInt_32_to_String(step);
	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
		m_flow_network.add_Vertex(vertex_label + "_" + step_label);
	    }
	}
	for (int step = 0; step < N_steps - 1; ++step)
	{
	    sString step_label = sInt_32_to_String(step);	    
	    sString next_step_label = sInt_32_to_String(step + 1);

	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
	
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		{
		    sString next_vertex_label = sInt_32_to_String((*neighbor)->m_target->m_id);		    
		    m_flow_network.add_Arc(vertex_label + "_" + step_label, next_vertex_label + "_" + next_step_label, 1, 0);
		}
		m_flow_network.add_Arc(vertex_label + "_" + step_label, vertex_label + "_" + next_step_label, 1, 0);
	    }	    
	}

	m_network_source = m_flow_network.add_Vertex("source");
	m_network_sink = m_flow_network.add_Vertex("sink");

	sString first_step_label = sInt_32_to_String(0);
	sString last_step_label = sInt_32_to_String(N_steps - 1);

	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	for (int r = 1; r <= N_robots; ++r)
	{
	    sString init_vertex_label = sInt_32_to_String(m_instance.m_initial_arrangement.get_RobotLocation(r));
	    sString goal_vertex_label = sInt_32_to_String(m_instance.m_goal_arrangement.get_RobotLocation(r));

	    m_flow_network.add_Arc("source", init_vertex_label + "_" + first_step_label, 1, 0);
	    m_flow_network.add_Arc(goal_vertex_label + "_" + last_step_label, "sink", 1, 0);
	}
    }


    void sMultirobotFlowModel::build_Network(const Robots_vector &robot_selection, int N_steps)
    {	
	for (int step = 0; step < N_steps; ++step)
	{
	    sString step_label = sInt_32_to_String(step);
	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
		m_flow_network.add_Vertex(vertex_label + "_" + step_label);
	    }
	}
	for (int step = 0; step < N_steps - 1; ++step)
	{
	    sString step_label = sInt_32_to_String(step);	    
	    sString next_step_label = sInt_32_to_String(step + 1);

	    for (sUndirectedGraph::Vertices_vector::const_iterator vertex = m_instance.m_environment.m_Vertices.begin(); vertex != m_instance.m_environment.m_Vertices.end(); ++vertex)
	    {
		sString vertex_label = sInt_32_to_String(vertex->m_id);
	
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
		{
		    sString next_vertex_label = sInt_32_to_String((*neighbor)->m_target->m_id);		    
		    m_flow_network.add_Arc(vertex_label + "_" + step_label, next_vertex_label + "_" + next_step_label, 1, 0);
		}
		m_flow_network.add_Arc(vertex_label + "_" + step_label, vertex_label + "_" + next_step_label, 1, 0);
	    }	    
	}

	m_network_source = m_flow_network.add_Vertex("source");
	m_network_sink = m_flow_network.add_Vertex("sink");

	sString first_step_label = sInt_32_to_String(0);
	sString last_step_label = sInt_32_to_String(N_steps - 1);

#ifdef sDEBUG
	{
	    int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	    sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());
	}
#endif
	for (Robots_vector::const_iterator r = robot_selection.begin(); r != robot_selection.end(); ++r)
	{
	    sString init_vertex_label = sInt_32_to_String(m_instance.m_initial_arrangement.get_RobotLocation(*r));
	    sString goal_vertex_label = sInt_32_to_String(m_instance.m_goal_arrangement.get_RobotLocation(*r));

	    m_flow_network.add_Arc("source", init_vertex_label + "_" + first_step_label, 1, 0);
	    m_flow_network.add_Arc(goal_vertex_label + "_" + last_step_label, "sink", 1, 0);
	}
    }


    void sMultirobotFlowModel::destroy_Network(void)
    {
	m_flow_network.clean_Graph();
    }


    int sMultirobotFlowModel::compute_Relocation(void)
    {
	sGoldberg goldberg;
	return goldberg.compute_Flow(&m_flow_network, &m_network_source->second, &m_network_sink->second);
    }


    int sMultirobotFlowModel::compute_Distance(void)
    {
	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	int N_steps = 1;

	while (true)
	{
	    destroy_Network();
	    build_Network(N_steps);

	    int relocation = compute_Relocation();
	    if (relocation == N_robots)
	    {
		return N_steps;
	    }
	    ++N_steps;
	}
	return -1;
    }


    int sMultirobotFlowModel::compute_Distance(const Robots_vector &robot_selection)
    {
	int N_robots = robot_selection.size();
	int N_steps = 1;

	while (true)
	{
	    destroy_Network();
	    build_Network(robot_selection, N_steps);

	    int relocation = compute_Relocation();
	    if (relocation == N_robots)
	    {
		return N_steps;
	    }
	    ++N_steps;
	}
	return -1;
    }


    int sMultirobotFlowModel::compute_Distance(int *N_tries)
    {
	int N_robots = m_instance.m_initial_arrangement.get_RobotCount();
	sASSERT(N_robots == m_instance.m_goal_arrangement.get_RobotCount());

	int max_N_steps = 1;

	for (int N_r = 1; N_r <= N_robots; ++N_r)
	{
	    for (int t = 0; t < N_tries[N_r-1]; ++t)
	    {
		Robots_vector robot_selection;
		for (int r = 1; r <= N_robots; ++r)
		{
		    robot_selection.push_back(r);
		}
		int selection_size = N_robots;
		while (selection_size > N_r)
		{
		    int rnd_r = rand() % selection_size;
		    robot_selection[rnd_r] = robot_selection[selection_size - 1];

		    --selection_size;
		}
		robot_selection.resize(N_r);

		#ifdef sVERBOSE
		for (Robots_vector::const_iterator sr = robot_selection.begin(); sr != robot_selection.end(); ++sr)
		{
		    printf("%d,", *sr);
		}
		printf("\n");
		#endif

		int N_steps = 1;
		while (true)
		{
		    destroy_Network();
		    build_Network(robot_selection, N_steps);
	    
		    int relocation = compute_Relocation();
		    if (relocation == N_r)
		    {
			break;
		    }
		    ++N_steps;
		}
		if (N_steps > max_N_steps)
		{
		    max_N_steps = N_steps;
		}
		printf("----> %d\n", N_steps);
	    }
	}
	return max_N_steps;
    }



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

} // namespace sReloc
