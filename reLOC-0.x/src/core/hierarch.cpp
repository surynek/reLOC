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
/* hierarch.cpp / 0.21-robik_041                                              */
/*----------------------------------------------------------------------------*/
//
// Hierarchical version of relocation problem.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "result.h"
#include "reloc.h"
#include "hierarch.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sEntityArrangement

    sEntityArrangement::sEntityArrangement()
    {
	// nothing
    }
    

    sEntityArrangement::sEntityArrangement(int N_Vertices, int N_Entities)
	: m_entity_Locs(N_Entities + 1),
	  m_vertex_Occups(N_Vertices)
    {
	for (int i = 0; i <= N_Entities; ++i)
	{
	    m_entity_Locs[i] = UNDEFINED_LOCATION;
	}

	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_vertex_Occups[i] = VACANT_VERTEX;
	}
    }


    sEntityArrangement::sEntityArrangement(const sEntityArrangement &entity_arrangement)
	: m_entity_Locs(entity_arrangement.m_entity_Locs),
	  m_vertex_Occups(entity_arrangement.m_vertex_Occups)
    {
	// nothing
    }


    const sEntityArrangement& sEntityArrangement::operator=(const sEntityArrangement &entity_arrangement)
    {
	m_entity_Locs = entity_arrangement.m_entity_Locs;
	m_vertex_Occups = entity_arrangement.m_vertex_Occups;

	return *this;
    }


    bool sEntityArrangement::operator==(const sEntityArrangement &entity_arrangement) const
    {
	sASSERT(m_entity_Locs.size() == entity_arrangement.m_entity_Locs.size());

	Entities_vector::const_iterator entity_A, entity_B;
	
	for (entity_A = m_entity_Locs.begin(), entity_B = entity_arrangement.m_entity_Locs.begin();
	     entity_A != m_entity_Locs.end();
	     ++entity_A, ++entity_B)
	{
	    if (*entity_A != *entity_B)
	    {
		return false;
	    }
	}
	return true;
    }


    bool sEntityArrangement::operator<(const sEntityArrangement &entity_arrangement) const
    {
	sASSERT(m_entity_Locs.size() == entity_arrangement.m_entity_Locs.size());

	Entities_vector::const_iterator entity_A, entity_B;
	
	for (entity_A = m_entity_Locs.begin(), entity_B = entity_arrangement.m_entity_Locs.begin();
	     entity_A != m_entity_Locs.end();
	     ++entity_A, ++entity_B)
	{
	    if (*entity_A < *entity_B)
	    {
		return true;
	    }
	    else
	    {
		if (*entity_A > *entity_B)
		{
		    return false;
		}
	    }
	}

	return false;
    }


    int sEntityArrangement::get_EntityCount(void) const
    {
	return (m_entity_Locs.size() - 1);
    }

    
    int sEntityArrangement::get_VertexCount(void) const
    {
	return m_vertex_Occups.size();
    }
	

    int sEntityArrangement::get_EntityLocation(int entity_id) const
    {
	sASSERT(entity_id > 0 && entity_id < m_entity_Locs.size());

	return m_entity_Locs[entity_id];
    }


    int sEntityArrangement::get_VertexOccupancy(int vertex_id) const
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	return m_vertex_Occups[vertex_id];
    }


    void sEntityArrangement::place_Entity(int entity_id, int vertex_id)
    {
	sASSERT(entity_id > 0 && entity_id < m_entity_Locs.size());
	sASSERT(vertex_id < m_vertex_Occups.size());
	sASSERT(m_vertex_Occups[vertex_id] == VACANT_VERTEX);
	
	m_entity_Locs[entity_id] = vertex_id;
	m_vertex_Occups[vertex_id] = entity_id;
    }


    void sEntityArrangement::remove_Entity(int entity_id)
    {
	sASSERT(entity_id > 0 && entity_id < m_entity_Locs.size());

	m_vertex_Occups[m_entity_Locs[entity_id]] = VACANT_VERTEX;
	m_entity_Locs[entity_id] = UNDEFINED_LOCATION;
    }


    void sEntityArrangement::clean_Vertex(int vertex_id)
    {
	sASSERT(vertex_id < m_vertex_Occups.size());

	m_entity_Locs[m_vertex_Occups[vertex_id]] = UNDEFINED_LOCATION;
	m_vertex_Occups[vertex_id] = VACANT_VERTEX;
    }


    void sEntityArrangement::move_Entity(int entity_id, int dest_vertex_id)
    {
	sASSERT(entity_id > 0 && entity_id < m_entity_Locs.size());
	sASSERT(dest_vertex_id < m_vertex_Occups.size());
	sASSERT(m_vertex_Occups[dest_vertex_id] == VACANT_VERTEX);

	m_vertex_Occups[m_entity_Locs[entity_id]] = VACANT_VERTEX;
	m_entity_Locs[entity_id] = dest_vertex_id;
	m_vertex_Occups[dest_vertex_id] = entity_id;
    }


    void sEntityArrangement::to_Screen(const sString &indent) const
    {
	printf("%sEntity arrangement: (|R| = %ld, |V| = %ld) [\n", indent.c_str(), m_entity_Locs.size() - 1, m_vertex_Occups.size());
	printf("%s%s entity locations: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Entitys_1 = m_entity_Locs.size();
	for (int i = 1; i < N_Entitys_1; ++i)
	{
	    printf("%d#%d ", i, m_entity_Locs[i]);
	}
	printf("}\n");

	printf("%s%s vertex occupancy: {", indent.c_str(), sRELOC_INDENT.c_str());
	
	int N_Vertices = m_vertex_Occups.size();
	for (int i = 0; i < N_Vertices; ++i)
	{
	    printf("%d#%d ", m_vertex_Occups[i], i);
	}
	printf("}\n");

	printf("%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
// sHierarchicalArrangement

    sHierarchicalArrangement::sHierarchicalArrangement()
    {
	// nothing
    }
    

    sHierarchicalArrangement::sHierarchicalArrangement(int N_Vertices, int N_Objects, int N_Agents)
	: m_object_arrangement(N_Vertices, N_Objects),
	  m_agent_arrangement(N_Vertices, N_Agents)
    {
	// nothing
    }


    sHierarchicalArrangement::sHierarchicalArrangement(const sEntityArrangement &object_arrangement, const sEntityArrangement &agent_arrangement)
	: m_object_arrangement(object_arrangement),
	  m_agent_arrangement(agent_arrangement)
    {
	sASSERT(object_arrangement.get_VertexCount() == agent_arrangement.get_VertexCount());
    }


    sHierarchicalArrangement::sHierarchicalArrangement(const sHierarchicalArrangement &hierarchical_arrangement)
	: m_object_arrangement(hierarchical_arrangement.m_object_arrangement),
	  m_agent_arrangement(hierarchical_arrangement.m_agent_arrangement)
    {
	// nothing
    }


    const sHierarchicalArrangement& sHierarchicalArrangement::operator=(const sHierarchicalArrangement &resticted_arrangement)
    {
	m_object_arrangement = resticted_arrangement.m_object_arrangement;
	m_agent_arrangement = resticted_arrangement.m_agent_arrangement;

	return *this;
    }


    int sHierarchicalArrangement::get_VertexCount(void) const
    {
	return m_object_arrangement.get_VertexCount();
    }


    int sHierarchicalArrangement::get_ObjectCount(void) const
    {
	return m_object_arrangement.get_EntityCount();
    }
    
    
    int sHierarchicalArrangement::get_AgentCount(void) const
    {
	return m_agent_arrangement.get_EntityCount();
    }


    int sHierarchicalArrangement::get_ObjectLocation(int object_id) const
    {
	return m_object_arrangement.get_EntityLocation(object_id);
    }


    int sHierarchicalArrangement::get_AgentLocation(int agent_id) const
    {
	return m_agent_arrangement.get_EntityLocation(agent_id);
    }


    void sHierarchicalArrangement::get_VertexOccupancy(int vertex_id, int &object_id, int &agent_id) const
    {
	object_id = m_object_arrangement.get_VertexOccupancy(vertex_id);
	agent_id = m_agent_arrangement.get_VertexOccupancy(vertex_id);
    }


    void sHierarchicalArrangement::place_Object(int object_id, int vertex_id)
    {
	m_object_arrangement.place_Entity(object_id, vertex_id);
    }


    void sHierarchicalArrangement::remove_Object(int object_id)
    {
	m_object_arrangement.remove_Entity(object_id);
    }


    void sHierarchicalArrangement::place_Agent(int agent_id, int vertex_id)
    {
	m_agent_arrangement.place_Entity(agent_id, vertex_id);
    }


    void sHierarchicalArrangement::remove_Agent(int agent_id)
    {
	m_agent_arrangement.remove_Entity(agent_id);
    }


    void sHierarchicalArrangement::clean_Vertex(int vertex_id)
    {
	m_object_arrangement.clean_Vertex(vertex_id);
	m_agent_arrangement.clean_Vertex(vertex_id);
    }

    
    void sHierarchicalArrangement::move_Agent(int agent_id, int dest_vertex_id)
    {
	m_agent_arrangement.move_Entity(agent_id, dest_vertex_id);
    }


    void sHierarchicalArrangement::move_AgentCombined(int agent_id, int dest_vertex_id)
    {
	int agent_vertex_id = m_agent_arrangement.get_EntityLocation(agent_id);
	int object_id = m_object_arrangement.get_VertexOccupancy(agent_vertex_id);
	sASSERT(object_id != sEntityArrangement::VACANT_VERTEX);
	m_object_arrangement.move_Entity(object_id, dest_vertex_id);
	m_agent_arrangement.move_Entity(agent_id, dest_vertex_id);
    }


    void sHierarchicalArrangement::move_ObjectCombined(int object_id, int dest_vertex_id)
    {
	int object_vertex_id = m_object_arrangement.get_EntityLocation(object_id);
	int agent_id = m_agent_arrangement.get_VertexOccupancy(object_vertex_id);
	sASSERT(agent_id != sEntityArrangement::VACANT_VERTEX);
	m_object_arrangement.move_Entity(object_id, dest_vertex_id);
	m_agent_arrangement.move_Entity(agent_id, dest_vertex_id);
    }


    void sHierarchicalArrangement::to_Screen(const sString &indent) const
    {
	printf("%sHierarchical arrangement: [\n", indent.c_str());
	printf("%s%sObjects:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_object_arrangement.to_Screen(indent + sRELOC_INDENT + sRELOC_INDENT);
	printf("%s%sAgents:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_agent_arrangement.to_Screen(indent + sRELOC_INDENT + sRELOC_INDENT);
	printf("%s]\n", indent.c_str());
    }


/*----------------------------------------------------------------------------*/
// sHierarchicalRelocationInstance

    sHierarchicalRelocationInstance::sHierarchicalRelocationInstance()
    {
	// nothing
    }


    sHierarchicalRelocationInstance::sHierarchicalRelocationInstance(const sUndirectedGraph &environment, const sHierarchicalArrangement &initial_arrangement, const sHierarchicalArrangement &goal_arrangement)
	: m_environment(environment),
	  m_initial_arrangement(initial_arrangement),
	  m_goal_arrangement(goal_arrangement)
    {
	sASSERT(environment.get_VertexCount() == initial_arrangement.get_VertexCount() && environment.get_VertexCount() == goal_arrangement.get_VertexCount());
    }


    sHierarchicalRelocationInstance::sHierarchicalRelocationInstance(const sHierarchicalRelocationInstance &multirobot_instance)
	: m_environment(multirobot_instance.m_environment),
	  m_initial_arrangement(multirobot_instance.m_initial_arrangement),
	  m_goal_arrangement(multirobot_instance.m_goal_arrangement)
    {
	// nothing
    }


    const sHierarchicalRelocationInstance& sHierarchicalRelocationInstance::operator=(const sHierarchicalRelocationInstance &multirobot_instance)
    {
	m_environment = multirobot_instance.m_environment;
	m_initial_arrangement = multirobot_instance.m_initial_arrangement;
	m_goal_arrangement = multirobot_instance.m_goal_arrangement;

	return *this;
    }


    void sHierarchicalRelocationInstance::to_Screen(const sString &indent) const
    {
	printf("%sHierarchical relocation instance: [\n", indent.c_str());
	printf("%s%sEnvironment:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_environment.to_Screen_vertices(indent + sRELOC_INDENT + sRELOC_INDENT);
	printf("%s%sInitial arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_initial_arrangement.to_Screen(indent + sRELOC_INDENT + sRELOC_INDENT);
	printf("%s%sGoal arrangement:\n", indent.c_str(), sRELOC_INDENT.c_str());
	m_goal_arrangement.to_Screen(indent + sRELOC_INDENT + sRELOC_INDENT);
	printf("%s]\n", indent.c_str());
    }


    void sHierarchicalRelocationInstance::to_Screen_domainPDDL(const sString &indent) const
    {
	to_Stream_domainPDDL(stdout, indent);
    }


    void sHierarchicalRelocationInstance::to_Screen_problemPDDL(const sString &indent) const
    {
	to_Stream_problemPDDL(stdout, indent);
    }


    void sHierarchicalRelocationInstance::to_Screen_CNFsat(int N_Layers, const sString &indent) const
    {
	to_Stream_CNFsat(stdout, N_Layers, indent);
    }


    sResult sHierarchicalRelocationInstance::to_File_domainPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sHIERARCHICAL_RELOCATION_PDDL_OPEN_ERROR;
	}
	to_Stream_domainPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sHierarchicalRelocationInstance::to_File_problemPDDL(const sString &filename, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sHIERARCHICAL_RELOCATION_PDDL_OPEN_ERROR;
	}
	to_Stream_problemPDDL(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sHierarchicalRelocationInstance::to_File_CNFsat(const sString &filename, int N_Layers, const sString &indent) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sHIERARCHICAL_RELOCATION_CNF_OPEN_ERROR;
	}
	to_Stream_CNFsat(fw, N_Layers, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


/*----------------------------------------------------------------------------*/

    void sHierarchicalRelocationInstance::to_Stream_domainPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (domain hierarchical_relocation)\n", indent.c_str());
	fprintf(fw, "%s%s(:predicates\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(object_location ?o ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(agent_location ?a ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(no_object ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s(no_agent ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:action move_agent\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:parameters (?a ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:precondition (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?a ?u) (no_agent ?v) (adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:effect (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?a ?v) (no_agent ?u) (not (no_agent ?v))\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:action move_combined\n", indent.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:parameters (?a ?o ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:precondition (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?a ?u) (object_location ?o ?u) (no_agent ?v) (no_object ?v) (adjacent ?u ?v)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s:effect (and\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s%s(agent_location ?a ?v) (object_location ?o ?v) (no_agent ?u) (no_object ?u) (not (no_agent ?v)) (not (no_object ?v))\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sHierarchicalRelocationInstance::to_Stream_problemPDDL(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(define (problem hierarchical_relocation_instance)\n", indent.c_str());
	fprintf(fw, "%s%s(:domain hierarchical_relocation)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:objects\n", indent.c_str(), sRELOC_INDENT.c_str());

	int N_Vertices = m_environment.get_VertexCount();
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    fprintf(fw, "%s%s%sv%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	}

	int N_Objects = m_initial_arrangement.get_ObjectCount();
	for (int object_id = 1; object_id <= N_Objects; ++object_id)
	{
	    fprintf(fw, "%s%s%so%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), object_id);
	}

	int N_Agents = m_initial_arrangement.get_AgentCount();
	for (int agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%sa%d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), agent_id);
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:init\n", indent.c_str(), sRELOC_INDENT.c_str());

	for (sUndirectedGraph::Edges_list::const_iterator edge = m_environment.m_Edges.begin(); edge != m_environment.m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_vu.m_target->m_id, edge->m_arc_uv.m_target->m_id);
	    fprintf(fw, "%s%s%s(adjacent v%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), edge->m_arc_uv.m_target->m_id, edge->m_arc_vu.m_target->m_id);
	}

	for (int object_id = 1; object_id <= N_Objects; ++object_id)
	{
	    fprintf(fw, "%s%s%s(object_location o%d, v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), object_id, m_initial_arrangement.get_ObjectLocation(object_id));
	}

	for (int agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%s(agent_location a%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), agent_id, m_initial_arrangement.get_AgentLocation(agent_id));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int object_id, agent_id;
	    m_initial_arrangement.get_VertexOccupancy(vertex_id, object_id, agent_id);

	    if (object_id == sEntityArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_object v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	    if (agent_id == sEntityArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_agent v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s)\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s%s(:goal (and \n", indent.c_str(), sRELOC_INDENT.c_str());

	for (int object_id = 1; object_id <= N_Objects; ++object_id)
	{
	    fprintf(fw, "%s%s%s(object_location o%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), object_id, m_goal_arrangement.get_ObjectLocation(object_id));
	}
	for (int agent_id = 1; agent_id <= N_Agents; ++agent_id)
	{
	    fprintf(fw, "%s%s%s(agent_location a%d v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), agent_id, m_goal_arrangement.get_AgentLocation(agent_id));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int object_id, agent_id;
	    m_goal_arrangement.get_VertexOccupancy(vertex_id, object_id, agent_id);

	    if (object_id == sEntityArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_object v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	    if (agent_id == sEntityArrangement::VACANT_VERTEX)
	    {
		fprintf(fw, "%s%s%s(no_agent v%d)\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), vertex_id);
	    }
	}
	fprintf(fw, "%s%s))\n", indent.c_str(), sRELOC_INDENT.c_str());

	fprintf(fw, "%s)\n", indent.c_str());
    }


    void sHierarchicalRelocationInstance::to_Stream_CNFsat(FILE *sUNUSED(fw), int sUNUSED(N_Layers), const sString &sUNUSED(indent)) const
    {
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
