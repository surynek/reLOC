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
/* hierarch.h / 0.21-robik_054                                                */
/*----------------------------------------------------------------------------*/
//
// Hierarchical version of relocation problem.
//
/*----------------------------------------------------------------------------*/


#ifndef __HIERARCH_H__
#define __HIERARCH_H__

#include <vector>
#include <list>
#include <set>
#include <map>

#include "reloc.h"
#include "result.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{

  
/*----------------------------------------------------------------------------*/
// sEntityArrangement

    class sEntityArrangement
    {
    public:
	static const int VACANT_VERTEX = 0; /* Entities are numbered starting with 1 */
	static const int UNDEFINED_LOCATION = -1;

    public:
	typedef std::vector<int> Entities_vector;
	typedef std::vector<int> Vertices_vector;

    public:
	sEntityArrangement();
	sEntityArrangement(int N_Vertices, int N_Entities);
	sEntityArrangement(const sEntityArrangement &entity_arrangement);
	const sEntityArrangement& operator=(const sEntityArrangement &entity_arrangement);

	bool operator==(const sEntityArrangement &entity_arrangement) const;
	bool operator<(const sEntityArrangement &entity_arrangement) const;

	int get_EntityCount(void) const;
	int get_VertexCount(void) const;

	int get_EntityLocation(int entity_id) const;
	int get_VertexOccupancy(int vertex_id) const;

	void place_Entity(int entity_id, int vertex_id);
	void remove_Entity(int entity_id);
	void clean_Vertex(int vertex_id);
	void move_Entity(int entity_id, int dest_vertex_id);

	virtual void to_Screen(const sString &indent = "") const;

    public:
	Entities_vector m_entity_Locs;
	Vertices_vector m_vertex_Occups;
    };


/*----------------------------------------------------------------------------*/
// sHierarchicalArrangement

    class sHierarchicalArrangement
    {
    public:
	sHierarchicalArrangement();
	sHierarchicalArrangement(int N_Vertices, int N_Objects, int N_Agents);
	sHierarchicalArrangement(const sEntityArrangement &object_arrangement, const sEntityArrangement &agent_arrangement);
	sHierarchicalArrangement(const sHierarchicalArrangement &hierarchical_arrangement);
	const sHierarchicalArrangement& operator=(const sHierarchicalArrangement &hierarchical_arrangement);	

	int get_VertexCount(void) const;
	int get_ObjectCount(void) const;
	int get_AgentCount(void) const;

	int get_ObjectLocation(int object_id) const;
	int get_AgentLocation(int agent_id) const;
	void get_VertexOccupancy(int vertex_id, int &object_id, int &agent_id) const;

	void place_Object(int object_id, int vertex_id);
	void remove_Object(int object_id);

	void place_Agent(int agent_id, int vertex_id);
	void remove_Agent(int agent_id);
	void clean_Vertex(int vertex_id);

	void move_Agent(int agent_id, int dest_vertex_id);
	void move_AgentCombined(int agent_id, int dest_vertex_id);
	void move_ObjectCombined(int object_id, int dest_vertex_id);

	virtual void to_Screen(const sString &indent = "") const;

    public:
	sEntityArrangement m_object_arrangement;
	sEntityArrangement m_agent_arrangement;
    };
    
/*----------------------------------------------------------------------------*/
// sHierarchicalRelocationInstance

    class sHierarchicalRelocationInstance
    {
    public:
	sHierarchicalRelocationInstance();
	sHierarchicalRelocationInstance(const sUndirectedGraph &environment, const sHierarchicalArrangement &initial_arrangement, const sHierarchicalArrangement &goal_arrangement);
	sHierarchicalRelocationInstance(const sHierarchicalRelocationInstance &hierarchical_instance);
	const sHierarchicalRelocationInstance& operator=(const sHierarchicalRelocationInstance &hierarchical_instance);

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Screen_domainPDDL(const sString &indent = "") const;
	virtual void to_Screen_problemPDDL(const sString &indent = "") const;
	virtual void to_Screen_CNFsat(int N_Layers, const sString &indent = "") const;

	virtual sResult to_File_domainPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_problemPDDL(const sString &filename, const sString &indent = "") const;
	virtual sResult to_File_CNFsat(const sString &filename, int N_Layers, const sString &indent = "") const;

    private:
	virtual void to_Stream_domainPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_problemPDDL(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_CNFsat(FILE *fw, int N_Layers, const sString &indent = "") const;

    public:
	sUndirectedGraph m_environment;
	sHierarchicalArrangement m_initial_arrangement;
	sHierarchicalArrangement m_goal_arrangement;
    };


    
/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __HIERARCH_H__ */
