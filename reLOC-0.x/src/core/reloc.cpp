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
/* reloc.cpp / 0.20-kruh_055                                                  */
/*----------------------------------------------------------------------------*/
//
// Relocation problem solving package - original development source.
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

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// Global constants

    const sString sRELOC_INDENT = "    ";
    const sString sRELOC_1_INDENT = sRELOC_INDENT;
    const sString sRELOC_2_INDENT = sRELOC_INDENT + sRELOC_INDENT;
    const sString sRELOC_3_INDENT = sRELOC_INDENT + sRELOC_INDENT + sRELOC_INDENT;
    const sString sRELOC_4_INDENT = sRELOC_INDENT + sRELOC_INDENT + sRELOC_INDENT + sRELOC_INDENT;

//    const sString sRELOC_SAT_SOLVER_PATH = "../../sat/glucose3 -verb=0 2>/dev/null";
    const sString sRELOC_SAT_SOLVER_PATH = "../../sat/glucose3 -verb=0 -var-decay=0.4 -cla-decay=0.3 -K=0.6 2>/dev/null";

/*----------------------------------------------------------------------------*/
// sVertex

    sVertex::sVertex()
	: m_id(0)
    {
	// nothing
    }


    sVertex::sVertex(int id)
	: m_id(id)
    {
	// nothing
    }


    sVertex::sVertex(const sVertex &vertex)
	: m_id(vertex.m_id)
	, m_Conflicts(vertex.m_Conflicts)
    {	
	sASSERT(m_Neighbors.empty() && vertex.m_Neighbors.empty());
    }


    const sVertex& sVertex::operator=(const sVertex &vertex)
    {
	m_id = vertex.m_id;
	m_Conflicts = vertex.m_Conflicts;
	sASSERT(m_Neighbors.empty() && vertex.m_Neighbors.empty());

	return *this;
    }


/*----------------------------------------------------------------------------*/

    int sVertex::calc_NeighborCount(void) const
    {
	return m_Neighbors.size();
    }


    int sVertex::calc_NeighborOrder(int vertex_id) const
    {
	int order = 0;
	for (Neighbors_list::const_iterator neighbor = m_Neighbors.begin(); neighbor != m_Neighbors.end(); ++neighbor)
	{
	    if ((*neighbor)->m_target->m_id == vertex_id)
	    {
		return order; 
	    }
	    ++order;
	}

	return ORDER_UNDEFINED;
    }


    int sVertex::calc_NeighborID(int order) const
    {
	Neighbors_list::const_iterator neighbor = m_Neighbors.begin();

	while (--order >= 0)
	{
	    ++neighbor;
	}

	return (*neighbor)->m_target->m_id;
    }


    void sVertex::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sVertex::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sVertex: (id = %d, distance = %d, prev_id = %d)", indent.c_str(), m_id, m_distance, m_prev_id);
	
	if (!m_Neighbors.empty())
	{
	    fprintf(fw, " {");
	    for (Neighbors_list::const_iterator neighbor = m_Neighbors.begin(); neighbor != m_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d->%d ", (*neighbor)->m_source->m_id, (*neighbor)->m_target->m_id);
	    }
	    fprintf(fw, "}");

	    fprintf(fw, " i[");
	    for (Neighbors_list::const_iterator neighbor = m_in_Neighbors.begin(); neighbor != m_in_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d ", (*neighbor)->m_source->m_id);
	    }
	    fprintf(fw, "]");

	    fprintf(fw, " o[");
	    for (Neighbors_list::const_iterator neighbor = m_out_Neighbors.begin(); neighbor != m_out_Neighbors.end(); ++neighbor)
	    {
		fprintf(fw, "%d ", (*neighbor)->m_target->m_id);
	    }
	    fprintf(fw, "]");
	}
	if (!m_Conflicts.empty())
	{
	    fprintf(fw, " < ");
	    for (VertexIDs_vector::const_iterator conflict = m_Conflicts.begin(); conflict != m_Conflicts.end(); ++conflict)
	    {
		fprintf(fw, "%d ", *conflict);
	    }
	    fprintf(fw, ">");	    
	}
	fprintf(fw, "\n");
    }


/*----------------------------------------------------------------------------*/
// sArc

    sArc::sArc()
	: m_edge(NULL)
	, m_source(NULL)
	, m_target(NULL)
    {
	// nothing
    }


    sArc::sArc(sEdge *edge, sVertex *source, sVertex *target)
	: m_edge(edge)
	, m_source(source)
	, m_target(target)
    {
	// nothing
    }


    sArc::sArc(const sArc &arc)
	: m_edge(arc.m_edge)
	, m_source(arc.m_source)
	, m_target(arc.m_target)
    {
	// nothing
    }


    const sArc& sArc::operator=(const sArc &arc)
    {
	m_edge = arc.m_edge;
	m_source = arc.m_source;
	m_target = arc.m_target;

	return *this;
    }


    void sArc::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

	 
    void sArc::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sArc: (edge = %p, source = %p, target = %p)\n", indent.c_str(), (void*)m_edge, (void*)m_source, (void*)m_target);
    }


/*----------------------------------------------------------------------------*/
// sEdge

    sEdge::sEdge(int id, sVertex *vertex_u, sVertex *vertex_v, bool directed)
	: m_id(id)
	, m_directed(directed)
	, m_arc_uv(this, vertex_u, vertex_v)
	, m_arc_vu(this, vertex_v, vertex_u)
    {
	// nothing
    }


    sEdge::sEdge(const sEdge &edge)
	: m_id(edge.m_id)
	, m_directed(edge.m_directed)
	, m_arc_uv(this, edge.m_arc_uv.m_source, edge.m_arc_uv.m_target)
	, m_arc_vu(this, edge.m_arc_vu.m_source, edge.m_arc_vu.m_target)
	, m_Conflicts(edge.m_Conflicts)
    {
	// nothing
    }


    const sEdge& sEdge::operator=(const sEdge &edge)
    {
	m_id = edge.m_id;
	m_directed = edge.m_directed;

	m_arc_uv = sArc(this, edge.m_arc_uv.m_source, edge.m_arc_uv.m_target);
	m_arc_vu = sArc(this, edge.m_arc_vu.m_source, edge.m_arc_vu.m_target);

	m_Conflicts = edge.m_Conflicts;

	return *this;
    }


    void sEdge::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sEdge::to_Stream(FILE *fw, const sString &indent) const
    {
	if (m_directed)
	{
	    fprintf(fw, "%sEdge  %d: %d --> %d", indent.c_str(), m_id, m_arc_uv.m_source->m_id, m_arc_uv.m_target->m_id);
//	    fprintf(fw, "%s[Edge  %d: %d --> %d]\n", indent.c_str(), m_id, m_arc_vu.m_source->m_id, m_arc_vu.m_target->m_id);
	}
	else
	{
	    fprintf(fw, "%sEdge  %d: %d <-> %d", indent.c_str(), m_id, m_arc_uv.m_source->m_id, m_arc_uv.m_target->m_id);
//	    fprintf(fw, "%s[Edge  %d: %d <-> %d]\n", indent.c_str(), m_id, m_arc_vu.m_source->m_id, m_arc_vu.m_target->m_id);
	}

	if (!m_Conflicts.empty())
	{
	    fprintf(fw, " < ");
	    for (Conflicts_vector::const_iterator conflict = m_Conflicts.begin(); conflict != m_Conflicts.end(); ++conflict)
	    {
		fprintf(fw, "{%d,%d} ", conflict->m_x_ID, conflict->m_y_ID);
	    }
	    fprintf(fw, ">");	    
	}
	fprintf(fw, "\n");
	
/*
	fprintf(fw, "%s]\n", indent.c_str());
	m_arc_uv.to_Stream(fw, indent + sRELOC_INDENT);
	m_arc_vu.to_Stream(fw, indent + sRELOC_INDENT);
	fprintf(fw, "%s]\n", indent.c_str());
*/
    }


/*----------------------------------------------------------------------------*/
// sUndirectedGraph

    sUndirectedGraph::sUndirectedGraph()
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	// nothing
    }


    sUndirectedGraph::sUndirectedGraph(bool directed)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	// nothing
    }    


    sUndirectedGraph::sUndirectedGraph(int x_size, int y_size)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	int grid_size = x_size * y_size;
	m_Matrix = new int[grid_size];

	for (int i = 0; i < grid_size; ++i)
	{
	    m_Matrix[i] = i + 1;
	}
	add_Vertices(x_size * y_size);
	
	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		add_Edge(u_id, v_id);
		add_Edge(u_id, w_id);		
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    add_Edge(u_id, v_id);
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    add_Edge(u_id, v_id);
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(int x_size, int y_size, double obstacle_prob)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	int rnd_limit = RAND_MAX * obstacle_prob;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		int rnd = rand();

		if (rnd > rnd_limit)
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
	    }
	}

	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(int x_size, int y_size, int N_obstacles)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	int matrix_size = x_size * y_size;
	m_Matrix = new int[matrix_size];
	int *selection_Matrix = new int[matrix_size];

	for (int i = 0; i < matrix_size; ++i)	
	{
	    m_Matrix[i] = 0;
	    selection_Matrix[i] = i;
	}

	int remaining = matrix_size;
	for (int i = 0; i < N_obstacles; ++i)
	{
	    int rnd = rand() % remaining;
	    m_Matrix[selection_Matrix[rnd]] = -1;

	    selection_Matrix[rnd] = selection_Matrix[remaining - 1];
	    --remaining;
	}
	delete selection_Matrix;

	int cnt = 0;
	for (int i = 0; i < matrix_size; ++i)	
	{
	    if (m_Matrix[i] != -1)
	    {
		m_Matrix[i] = cnt++;
	    }
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, int x_size, int y_size)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	int grid_size = x_size * y_size;
	m_Matrix = new int[grid_size];

	for (int i = 0; i < grid_size; ++i)
	{
	    m_Matrix[i] = i + 1;
	}
	add_Vertices(x_size * y_size);
	
	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (directed)
		{		    
		    add_RandomArrow(u_id, v_id);
		    add_RandomArrow(u_id, w_id);
		}
		else
		{
		    add_Edge(u_id, v_id);
		    add_Edge(u_id, w_id);		    
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (directed)
	    {
		add_RandomArrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (directed)
	    {
		add_RandomArrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, int x_size, int y_size, double obstacle_prob)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	int rnd_limit = RAND_MAX * obstacle_prob;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		int rnd = rand();

		if (rnd > rnd_limit)
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
	    }
	}

	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			if (directed)
			{			    
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
			}
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[w_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
			}
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, int x_size, int y_size, int N_obstacles)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_x_size(x_size)
	, m_y_size(y_size)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	int matrix_size = x_size * y_size;
	m_Matrix = new int[matrix_size];
	int *selection_Matrix = new int[matrix_size];

	for (int i = 0; i < matrix_size; ++i)	
	{
	    m_Matrix[i] = 0;
	    selection_Matrix[i] = i;
	}

	int remaining = matrix_size;
	for (int i = 0; i < N_obstacles; ++i)
	{
	    int rnd = rand() % remaining;
	    m_Matrix[selection_Matrix[rnd]] = -1;

	    selection_Matrix[rnd] = selection_Matrix[remaining - 1];
	    --remaining;
	}
	delete selection_Matrix;

	int cnt = 0;
	for (int i = 0; i < matrix_size; ++i)	
	{
	    if (m_Matrix[i] != -1)
	    {
		m_Matrix[i] = cnt++;
	    }
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
			}
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			if (directed)
			{
			    add_RandomArrow(m_Matrix[u_id], m_Matrix[w_id]);
			}
			else
			{
			    add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
			}
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}		    
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		if (directed)
		{
		    add_RandomArrow(m_Matrix[u_id], m_Matrix[v_id]);
		}
		else
		{
		    add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		}
	    }
	}
	initialize_InverseMatrix();
    }    


    sUndirectedGraph::sUndirectedGraph(int base_cycle_size, int ear_min_size, int ear_max_size, int graph_size)
	: m_directed(false)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	Edges_vector Edges;

	int vertex_id = 0;
	add_Vertex();
	++vertex_id;

	for (int i = 1; i < base_cycle_size; ++i)
	{
	    vertex_id++;
	    add_Vertex();
	    Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	}
	Edges.push_back(Edge(vertex_id - 1, 0));
	
	while (vertex_id < graph_size)
	{
	    int ear_size = ear_min_size + rand() % (ear_max_size - ear_min_size);
	    int first_connection_id = rand() % vertex_id;
	    int last_connection_id = rand() % vertex_id;

	    if (last_connection_id == first_connection_id)
	    {
		last_connection_id = (last_connection_id + 1) % vertex_id;
	    }

	    int first_vertex_id = ++vertex_id;
	    add_Vertex();

	    for (int i = 1; i < ear_size; ++i)
	    {
		vertex_id++;
		add_Vertex();
		Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	    }
	    int last_vertex_id = vertex_id - 1;

	    Edges.push_back(Edge(first_vertex_id, first_connection_id));
	    Edges.push_back(Edge(last_vertex_id, last_connection_id));
	}
	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Edge(edge->m_u_id, edge->m_v_id);
	}
	initialize_InverseMatrix();
    }


    sUndirectedGraph::sUndirectedGraph(bool directed, int base_cycle_size, int ear_min_size, int ear_max_size, int graph_size)
	: m_directed(directed)
	, m_Edge_cnt(0)
	, m_Matrix(NULL)
	, m_all_pairs_distances_calculated(false)
	, m_source_goal_distances_calculated(false)
	, m_all_pairs_coop_distances_calculated(false)
    {
	Edges_vector Edges;

	int vertex_id = 0;
	add_Vertex();
	++vertex_id;

	for (int i = 1; i < base_cycle_size; ++i)
	{
	    add_Vertex();
	    ++vertex_id;
	    Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	}
	Edges.push_back(Edge(vertex_id - 1, 0));
	
	while (vertex_id < graph_size)
	{
	    int ear_size = ear_min_size + rand() % (ear_max_size - ear_min_size);
	    int first_connection_id = rand() % vertex_id;
	    int last_connection_id = rand() % vertex_id;

	    if (last_connection_id == first_connection_id)
	    {
		last_connection_id = (last_connection_id + 1) % vertex_id;
	    }

	    int first_vertex_id = vertex_id;
	    add_Vertex();
	    ++vertex_id;

	    for (int i = 1; i < ear_size; ++i)
	    {
		add_Vertex();
		++vertex_id;
		Edges.push_back(Edge(vertex_id - 2, vertex_id - 1));
	    }
	    int last_vertex_id = vertex_id - 1;

	    Edges.push_back(Edge(first_connection_id, first_vertex_id));
	    Edges.push_back(Edge(last_vertex_id, last_connection_id));
	}
	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Arrow(edge->m_u_id, edge->m_v_id);
	}
	initialize_InverseMatrix();
    }    


    sUndirectedGraph::sUndirectedGraph(const sUndirectedGraph &undirected_graph)
	: m_directed(undirected_graph.m_directed)
	, m_Edge_cnt(0)
	, m_x_size(undirected_graph.m_x_size)
	, m_y_size(undirected_graph.m_y_size)
    {
	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    int grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (int i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
	
	add_Vertices(undirected_graph.m_Vertices.size());
	for (int i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	}

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)	
	{
	    if (edge->m_directed)
	    {
		add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    m_Edges.back().m_Conflicts = edge->m_Conflicts;
	}

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;
    }


    sUndirectedGraph::sUndirectedGraph(const sUndirectedGraph &undirected_graph, bool opposite)
	: m_directed(undirected_graph.m_directed)
	, m_Edge_cnt(0)
	, m_x_size(undirected_graph.m_x_size)
	, m_y_size(undirected_graph.m_y_size)
    {
	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    int grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (int i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
		
	add_Vertices(undirected_graph.m_Vertices.size());
	for (int i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	}

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)	
	{
	    if (edge->m_directed)
	    {
		if (opposite)
		{
		    add_Arrow(edge->m_arc_uv.m_target->m_id, edge->m_arc_uv.m_source->m_id);
		}
		else
		{
		    add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
		}
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    m_Edges.back().m_Conflicts = edge->m_Conflicts;
	}

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;
    }    


    const sUndirectedGraph& sUndirectedGraph::operator=(const sUndirectedGraph &undirected_graph)
    {
	m_directed = undirected_graph.m_directed;
	m_Edge_cnt = 0;

	m_x_size = undirected_graph.m_x_size;
	m_y_size = undirected_graph.m_y_size;

	if (undirected_graph.m_Matrix != NULL)
	{
	    m_Matrix = new int[undirected_graph.m_x_size * undirected_graph.m_y_size];
	    
	    int grid_size = undirected_graph.m_x_size * undirected_graph.m_y_size;
	    for (int i = 0; i < grid_size; ++i)
	    {
		m_Matrix[i] = undirected_graph.m_Matrix[i];
	    }
	}
	else
	{
	    m_Matrix = NULL;
	}
	m_inverse_Matrix = undirected_graph.m_inverse_Matrix;
	
	m_Vertices.clear();
	m_Edges.clear();

	add_Vertices(undirected_graph.m_Vertices.size());
	for (int i = 0; i < undirected_graph.m_Vertices.size(); ++i)
	{
	    m_Vertices[i].m_Conflicts = undirected_graph.m_Vertices[i].m_Conflicts;
	}	

	for (Edges_list::const_iterator edge = undirected_graph.m_Edges.begin(); edge != undirected_graph.m_Edges.end(); ++edge)
	{
	    if (edge->m_directed)
	    {
		add_Arrow(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    else
	    {
		add_Edge(edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	    }
	    m_Edges.back().m_Conflicts = edge->m_Conflicts;
	}	

	m_all_pairs_distances_calculated = undirected_graph.m_all_pairs_distances_calculated;
	m_all_pairs_Distances = undirected_graph.m_all_pairs_Distances;

	m_source_goal_distances_calculated = undirected_graph.m_source_goal_distances_calculated;
	m_source_Distances = undirected_graph.m_source_Distances;
	m_goal_Distances = undirected_graph.m_goal_Distances;

	return *this;
    }


    sUndirectedGraph::~sUndirectedGraph()
    {
	if (m_Matrix != NULL)
	{
	    delete m_Matrix;
	}
    }
    

/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::initialize_InverseMatrix(void)
    {	
	if (m_Matrix != NULL)
	{
	    m_inverse_Matrix.resize(get_VertexCount());
		
	    for (int j = 0; j < m_y_size; ++j)
	    {
		for (int i = 0; i < m_x_size; ++i)
		{
		    int vertex_ID = m_Matrix[j * m_x_size + i];
		    
		    if (vertex_ID > -1)
		    {
			m_inverse_Matrix[vertex_ID].m_row = j;
			m_inverse_Matrix[vertex_ID].m_column = i;
		    }
		}
	    }
	}
    }

    
    void sUndirectedGraph::generate_Network(int x_size, int y_size, int aisle_length, double obstacle_prob)
    {
	generate_Network(x_size, y_size, aisle_length, aisle_length, obstacle_prob);
    }


    void sUndirectedGraph::generate_Network(int x_size, int y_size, int aisle_length, int N_obstacles)
    {
	generate_Network(x_size, y_size, aisle_length, aisle_length, N_obstacles);
    }


    void sUndirectedGraph::generate_Network(int x_size, int y_size, int min_aisle_length, int max_aisle_length, double obstacle_prob)
    {
	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	int rnd_limit = RAND_MAX * obstacle_prob;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		int rnd = rand();

		if (rnd > rnd_limit)
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
	    }
	}

	generate_Network(m_Matrix, cnt, x_size, y_size, min_aisle_length, max_aisle_length);
    }


    void sUndirectedGraph::generate_Network(int x_size, int y_size, int min_aisle_length, int max_aisle_length, int N_obstacles)
    {
	int matrix_size = x_size * y_size;
	m_Matrix = new int[matrix_size];
	int *selection_Matrix = new int[matrix_size];

	for (int i = 0; i < matrix_size; ++i)	
	{
	    m_Matrix[i] = 0;
	    selection_Matrix[i] = i;
	}

	int remaining = matrix_size;
	for (int i = 0; i < N_obstacles; ++i)
	{
	    int rnd = rand() % remaining;
	    m_Matrix[selection_Matrix[rnd]] = -1;

	    selection_Matrix[rnd] = selection_Matrix[remaining - 1];
	    --remaining;
	}
	delete selection_Matrix;

	int cnt = 0;
	for (int i = 0; i < matrix_size; ++i)	
	{
	    if (m_Matrix[i] != -1)
	    {
		m_Matrix[i] = cnt++;
	    }
	}

	generate_Network(m_Matrix, cnt, x_size, y_size, min_aisle_length, max_aisle_length);
    }


    void sUndirectedGraph::generate_Network(int *sUNUSED(Matrix), int size, int x_size, int y_size, int min_aisle_length, int max_aisle_length)
    {
	Edges_vector Edges;

	add_Vertices(size);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Aisle(m_Matrix[u_id], m_Matrix[v_id], min_aisle_length, max_aisle_length, Edges);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Aisle(m_Matrix[u_id], m_Matrix[w_id], min_aisle_length, max_aisle_length, Edges);
		    }
		}
	    }
	}

	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Aisle(m_Matrix[u_id], m_Matrix[v_id], min_aisle_length, max_aisle_length, Edges);
	    }
	}

	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Aisle(m_Matrix[u_id], m_Matrix[v_id], min_aisle_length, max_aisle_length, Edges);
	    }
	}

	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Edge(edge->m_u_id, edge->m_v_id);
	}
    }


    void sUndirectedGraph::generate_Hypercube(int dimmension, int aisle_length)
    {
	generate_Hypercube(dimmension, aisle_length, aisle_length);
    }


    void sUndirectedGraph::generate_Hypercube(int dimmension, int min_aisle_length, int max_aisle_length)
    {
	Edges_vector hyper_Edges, Edges;
	VertexIDs_vector hyper_Vertices;

	if (dimmension > 0)
	{
	    add_Vertices(2);
	    hyper_Vertices.push_back(0);
	    hyper_Vertices.push_back(1);
	    hyper_Edges.push_back(Edge(0, 1));

	    for (int d = 1; d < dimmension; ++d)
	    {
		int hyper_copy_id = m_Vertices.size();
		add_Vertices(m_Vertices.size());

		Edges_vector hyper_copy_Edges;

		for (Edges_vector::const_iterator hyper_edge = hyper_Edges.begin(); hyper_edge != hyper_Edges.end(); ++hyper_edge)
		{
		    hyper_copy_Edges.push_back(Edge(hyper_edge->m_u_id + hyper_copy_id, hyper_edge->m_v_id + hyper_copy_id));
		}
		for (int i = 0; i < hyper_copy_id; ++i)
		{
		    hyper_Vertices.push_back(i + hyper_copy_id);
		    hyper_Edges.push_back(Edge(i, i + hyper_copy_id));
		}
		hyper_Edges.insert(hyper_Edges.end(), hyper_copy_Edges.begin(), hyper_copy_Edges.end());
	    }
	}
	for (Edges_vector::const_iterator hyper_edge = hyper_Edges.begin(); hyper_edge != hyper_Edges.end(); ++hyper_edge)
	{
	    add_Aisle(hyper_edge->m_u_id, hyper_edge->m_v_id, min_aisle_length, max_aisle_length, Edges);
	}

	for (Edges_vector::const_iterator edge = Edges.begin(); edge != Edges.end(); ++edge)
	{
	    add_Edge(edge->m_u_id, edge->m_v_id);
	}
    }


    void sUndirectedGraph::add_Aisle(int u_id, int v_id, int min_aisle_length, int max_aisle_length, Edges_vector &Edges)
    {
	int aisle_length;

	if (max_aisle_length > min_aisle_length)
	{
	    aisle_length = min_aisle_length + rand() % (max_aisle_length - min_aisle_length);
	}
	else
	{
	    aisle_length = min_aisle_length;
	}

	if (aisle_length > 0)
	{
	    int first_vertex = m_Vertices.size();
	    m_Vertices.push_back(sVertex(m_Vertices.size()));
	    int last_vertex = first_vertex;
	    
	    int prev_vertex = first_vertex;
	    
	    for (int i = 1; i < aisle_length; ++i)
	    {
		last_vertex = m_Vertices.size();
		m_Vertices.push_back(sVertex(m_Vertices.size()));
		Edges.push_back(Edge(prev_vertex, last_vertex));
	    }

	    Edges.push_back(Edge(u_id, first_vertex));
	    Edges.push_back(Edge(last_vertex, v_id));
	}
	else
	{
	    Edges.push_back(Edge(u_id, v_id));
	}
    }


    void sUndirectedGraph::explicate_Conflicts(int range)
    {
	Distances_2d_vector all_pairs_Distances;
	calc_AllPairsShortestPaths(all_pairs_Distances);

	for (int u_id = 0; u_id < m_Vertices.size(); ++u_id)
	{
	    for (int v_id = 0; v_id < u_id; ++v_id)
	    {
		if (all_pairs_Distances[u_id][v_id] <= range)
		{
		    m_Vertices[u_id].m_Conflicts.push_back(v_id);
		}      
	    }
	    for (int v_id = u_id + 1; v_id < m_Vertices.size(); ++v_id)
	    {
		if (all_pairs_Distances[u_id][v_id] <= range)
		{
		    m_Vertices[u_id].m_Conflicts.push_back(v_id);
		}		
	    }	    
	}
    }


    void sUndirectedGraph::add_Vertex(void)
    {
	m_Vertices.push_back(sVertex(m_Vertices.size()));
    }


    void sUndirectedGraph::add_Vertices(int Vertex_cnt)
    {
	while (Vertex_cnt-- > 0)
	{
	    m_Vertices.push_back(sVertex(m_Vertices.size()));
	}
    }


    int sUndirectedGraph::get_VertexCount(void) const
    {
	return m_Vertices.size();
    }


    sVertex* sUndirectedGraph::get_Vertex(int id)
    {
	return &m_Vertices[id];
    }


    const sVertex* sUndirectedGraph::get_Vertex(int id) const
    {
	return &m_Vertices[id];
    }


    bool sUndirectedGraph::is_Grid(void) const
    {
	return (m_Matrix != NULL);
    }


    int sUndirectedGraph::get_GridHeight(void) const
    {
	sASSERT(m_Matrix != NULL);
	return m_y_size;
    }

    
    int sUndirectedGraph::get_GridWidth(void) const
    {
	sASSERT(m_Matrix != NULL);
	return m_x_size;
    }


    int sUndirectedGraph::get_GridCell(int x, int y) const
    {
	sASSERT(m_Matrix != NULL);
	return m_Matrix[y * m_x_size + x];
    }


    int sUndirectedGraph::calc_GridRow(int vertex_id) const
    {
	sASSERT(m_Matrix != NULL);

	for (int j = 0; j < m_y_size; ++j)
	{
	    for (int i = 0; i < m_x_size; ++i)
	    {
		if (m_Matrix[j * m_x_size + i] == vertex_id)
		{
		    return j;
		}
	    }
	}
	sASSERT(false);
	return -1;
    }


    int sUndirectedGraph::calc_GridColumn(int vertex_id) const
    {
	sASSERT(m_Matrix != NULL);

	for (int j = 0; j < m_y_size; ++j)
	{
	    for (int i = 0; i < m_x_size; ++i)
	    {
		if (m_Matrix[j * m_x_size + i] == vertex_id)
		{
		    return i;
		}
	    }
	}
	sASSERT(false);
	return -1;
    }


    int sUndirectedGraph::calc_GridVertexID(int grid_row, int grid_column) const
    {
	sASSERT(m_Matrix != NULL);
	return m_Matrix[grid_row * m_x_size + grid_column];
    }


    int sUndirectedGraph::calc_GridNeighborVertexID(int vertex_id, int delta_row, int delta_column) const
    {
	int row = calc_GridRow(vertex_id);
	int column = calc_GridColumn(vertex_id);

	int next_row = row + delta_row;
	int next_column = column + delta_column;

	if (next_row >= 0 && next_column < m_y_size && next_column >= 0 && next_column < m_x_size)
	{
	    return calc_GridVertexID(next_row, next_column);
	}
	else
	{
	    return -1;
	}
    }


    int sUndirectedGraph::get_GridNeighborVertexID(int vertex_id, int delta_row, int delta_column) const
    {
	sASSERT(m_Matrix != NULL);
		
	int row = m_inverse_Matrix[vertex_id].m_row;
	int column = m_inverse_Matrix[vertex_id].m_column;

	int next_row = row + delta_row;
	int next_column = column + delta_column;	

	if (next_row >= 0 && next_column < m_y_size && next_column >= 0 && next_column < m_x_size)
	{
	    return calc_GridVertexID(next_row, next_column);
	}
	else
	{
	    return -1;
	}	
    }
    


    int sUndirectedGraph::get_EdgeCount(void) const
    {
	return m_Edge_cnt;
	//	return m_Edges.size();
    }


    void sUndirectedGraph::add_Arrow(int u_id, int v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	int id = m_Edge_cnt++;
	m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id], true));
	sEdge &inserted_edge = m_Edges.back();

	m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
    }


    void sUndirectedGraph::add_Edge(int u_id, int v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	int id = m_Edge_cnt++;
	m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id]));
	sEdge &inserted_edge = m_Edges.back();	

	m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
	
	m_Vertices[v_id].m_Neighbors.push_back(&inserted_edge.m_arc_vu);
	m_Vertices[u_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_vu);
	m_Vertices[v_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_vu);
    }


    void sUndirectedGraph::add_RandomArrow(int u_id, int v_id)
    {
	sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);
	sASSERT(u_id != v_id);

	int rnd = rand() % 2;

	if (rnd == 1)
	{
	    int id = m_Edge_cnt++;
	    m_Edges.push_back(sEdge(id, &m_Vertices[u_id], &m_Vertices[v_id], true));
	    sEdge &inserted_edge = m_Edges.back();

	    m_Vertices[u_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[v_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[u_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);
	}
	else
	{
	    int id = m_Edge_cnt++;
	    m_Edges.push_back(sEdge(id, &m_Vertices[v_id], &m_Vertices[u_id], true));
	    sEdge &inserted_edge = m_Edges.back();

	    m_Vertices[v_id].m_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[u_id].m_in_Neighbors.push_back(&inserted_edge.m_arc_uv);
	    m_Vertices[v_id].m_out_Neighbors.push_back(&inserted_edge.m_arc_uv);	    
	}
    }    


    bool sUndirectedGraph::is_Adjacent(int u_id, int v_id) const
    {
	if (m_directed)
	{
	    for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_Neighbors.end(); ++u_neighbor)
	    {
		if ((*u_neighbor)->m_target->m_id == v_id)
		{
		    return true;
		}
	    }	    
	}
	else
	{
	    sASSERT(m_Vertices.size() > u_id && m_Vertices.size() > v_id);

	    if (m_Vertices[u_id].m_Neighbors.size() < m_Vertices[v_id].m_Neighbors.size())
	    {
		for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_Neighbors.end(); ++u_neighbor)
		{
		    if ((*u_neighbor)->m_target->m_id == v_id)
		    {
			return true;
		    }
		}
	    }
	    else
	    {
		for (sVertex::Neighbors_list::const_iterator v_neighbor = m_Vertices[v_id].m_Neighbors.begin(); v_neighbor != m_Vertices[v_id].m_Neighbors.end(); ++v_neighbor)
		{
		    if ((*v_neighbor)->m_target->m_id == u_id)
		    {
			return true;
		    }
		}
	    }
	}
	return false;
    }


    bool sUndirectedGraph::is_LinkedTo(int u_id, int v_id) const
    {
	for (sVertex::Neighbors_list::const_iterator u_neighbor = m_Vertices[u_id].m_out_Neighbors.begin(); u_neighbor != m_Vertices[u_id].m_out_Neighbors.end(); ++u_neighbor)
	{
	    if ((*u_neighbor)->m_target->m_id == v_id)
	    {
		return true;
	    }
	}
	return false;
    }


    int sUndirectedGraph::calc_ShortestPath(int u_id, int v_id) const
    {
	Distances_vector Distances;
	calc_SingleSourceShortestPathsBreadth(u_id, Distances);

	return Distances[v_id];
    }


    void sUndirectedGraph::calc_SingleSourceShortestPaths(int s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);

	Distances[s_id] = 0;
	VertexQueue_multimap vertex_Queue;
	vertex_Queue.insert(VertexQueue_multimap::value_type(0, s_id));

	while (!vertex_Queue.empty())
	{
	    VertexQueue_multimap::value_type front_record = *vertex_Queue.begin();

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_record.second].m_Neighbors.begin(); neighbor != m_Vertices[front_record.second].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;
		int distance_update = front_record.first + 1;
		if (Distances[neighbor_id] == sINT_32_MAX || Distances[neighbor_id] > distance_update)
		{
		    Distances[neighbor_id] = distance_update;
		    vertex_Queue.insert(VertexQueue_multimap::value_type(distance_update, neighbor_id));
		}
	    }
	    vertex_Queue.erase(vertex_Queue.begin());
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth(int s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	Distances[s_id] = 0;
/*
	VertexIDs_list vertex_Queue;
	vertex_Queue.push_back(s_id);
*/

	VertexIDs_vector vertex_Queue_;
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);
	int queue_bottom = 0;
	int queue_top = 0;
	vertex_Queue_[queue_top++] = s_id;

//	while (!vertex_Queue.empty())
	while (queue_top > queue_bottom)
	{
//	    int front_id = *vertex_Queue.begin();
	    int front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
//		    vertex_Queue.push_back(neighbor_id);
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
//	    vertex_Queue.pop_front();
	    ++queue_bottom;
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth_opposite(int s_id, Distances_vector &Distances) const
    {
	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	Distances[s_id] = 0;
/*
	VertexIDs_list vertex_Queue;
	vertex_Queue.push_back(s_id);
*/

	VertexIDs_vector vertex_Queue_;
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);
	int queue_bottom = 0;
	int queue_top = 0;
	vertex_Queue_[queue_top++] = s_id;

	sUndirectedGraph oppo_graph(*this, true);

//	while (!vertex_Queue.empty())
	while (queue_top > queue_bottom)
	{
//	    int front_id = *vertex_Queue.begin();
	    int front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = oppo_graph.m_Vertices[front_id].m_Neighbors.begin(); neighbor != oppo_graph.m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
//		    vertex_Queue.push_back(neighbor_id);
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
//	    vertex_Queue.pop_front();
	    ++queue_bottom;
	}
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth_opposite(int s_id)
    {
	int N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	int queue_bottom = 0;
	int queue_top = 0;
	m_Queue[queue_top++] = s_id;

	sUndirectedGraph oppo_graph(*this, true);	

	while (queue_top > queue_bottom)
	{
	    int front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = oppo_graph.m_Vertices[front_id].m_Neighbors.begin(); neighbor != oppo_graph.m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }    


    void sUndirectedGraph::find_ShortestPathBreadth(int source_id, int dest_id, VertexIDs_vector &shortest_Path)
    {
	Distances_vector Distances;
	VertexIDs_vector vertex_Queue_;

	Distances.resize(m_Vertices.size(), sINT_32_MAX);
	vertex_Queue_.resize(m_Vertices.size(), sINT_32_MAX);

	int queue_bottom = 0;
	int queue_top = 0;
	vertex_Queue_[queue_top++] = source_id;
	m_Vertices[source_id].m_prev_id = -1;
	Distances[source_id] = 0;

	while (queue_top > queue_bottom)
	{
	    int front_id = vertex_Queue_[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (Distances[neighbor_id] == sINT_32_MAX)
		{
		    Distances[neighbor_id] = Distances[front_id] + 1;
		    m_Vertices[neighbor_id].m_prev_id = front_id;
		    vertex_Queue_[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}

	int vertex_id = dest_id;

	do
	{
	    shortest_Path.push_back(vertex_id);
	    vertex_id = m_Vertices[vertex_id].m_prev_id;
	} while (vertex_id != -1);
    }


    void sUndirectedGraph::calc_SingleSourceShortestPathsBreadth(int s_id)
    {
	int N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	int queue_bottom = 0;
	int queue_top = 0;
	m_Queue[queue_top++] = s_id;

	while (queue_top > queue_bottom)
	{
	    int front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;
		}
	    }
	    ++queue_bottom;
	}
    }


    int sUndirectedGraph::calc_ShortestCoopPath(int u1_id, int u2_id, int v1_id, int v2_id) const
    {
	Distances_2d_vector Distances;
	calc_SingleSourceShortestCoopPaths(u1_id, u2_id, Distances);

	return Distances[v1_id][v2_id];
    }


    void sUndirectedGraph::calc_SingleSourceShortestCoopPaths(int s1_id, int s2_id, Distances_2d_vector &Distances) const
    {
	sASSERT(s1_id != s2_id);

	int N_Vertices = m_Vertices.size();
	Distances.resize(N_Vertices);
	for (int i = 0; i < N_Vertices; ++i)
	{
	    Distances[i].resize(N_Vertices, sINT_32_MAX);
	}

	Distances[s1_id][s2_id] = 0;
	VertexPairQueue_multimap vertex_Queue;
	vertex_Queue.insert(VertexPairQueue_multimap::value_type(0, Vertex_pair(s1_id, s2_id)));

	while (!vertex_Queue.empty())
	{
	    VertexPairQueue_multimap::value_type front_record = *vertex_Queue.begin();
	    
	    VertexIDs_vector Neighbors_1;
	    Neighbors_1.push_back(front_record.second.first);
	    for (sVertex::Neighbors_list::const_iterator neighbor_1 = m_Vertices[front_record.second.first].m_Neighbors.begin(); neighbor_1 != m_Vertices[front_record.second.first].m_Neighbors.end(); ++neighbor_1)
	    {
		Neighbors_1.push_back((*neighbor_1)->m_target->m_id);
	    }

	    VertexIDs_vector Neighbors_2;
	    Neighbors_2.push_back(front_record.second.second);
	    for (sVertex::Neighbors_list::const_iterator neighbor_2 = m_Vertices[front_record.second.second].m_Neighbors.begin(); neighbor_2 != m_Vertices[front_record.second.second].m_Neighbors.end(); ++neighbor_2)
	    {
		Neighbors_2.push_back((*neighbor_2)->m_target->m_id);
	    }

	    for (VertexIDs_vector::const_iterator neighbor_1 = Neighbors_1.begin(); neighbor_1 != Neighbors_1.end(); ++neighbor_1)
	    {
		for (VertexIDs_vector::const_iterator neighbor_2 = Neighbors_2.begin(); neighbor_2 != Neighbors_2.end(); ++neighbor_2)
		{
		    int distance_update = front_record.first + 1;
		    bool update = false;

		    if (front_record.second.first != *neighbor_1)
		    {
			if (front_record.second.second != *neighbor_2)
			{
			    std::set<int, std::less<int> > context;
			    context.insert(front_record.second.first);
			    context.insert(front_record.second.second);
			    context.insert(*neighbor_1);
			    context.insert(*neighbor_2);

			    if (context.size() == 4)
			    {
				update = true;
			    }
			}
			else
			{
			    std::set<int, std::less<int> > context;
			    context.insert(front_record.second.first);
			    context.insert(front_record.second.second);
			    context.insert(*neighbor_1);

			    if (context.size() == 3)
			    {
				update = true;
			    }
			}
		    }
		    else
		    {
			if (front_record.second.second != *neighbor_2)
			{
			    std::set<int, std::less<int> > context;
			    context.insert(front_record.second.first);
			    context.insert(front_record.second.second);
			    context.insert(*neighbor_2);

			    if (context.size() == 3)
			    {
				update = true;
			    }
			}
			else
			{
			    std::set<int, std::less<int> > context;
			    context.insert(front_record.second.first);
			    context.insert(front_record.second.second);

			    if (context.size() == 2)
			    {
				update = true;
			    }
			}
		    }
		    if (update)
		    {
			if (Distances[*neighbor_1][*neighbor_2] == sINT_32_MAX || Distances[*neighbor_1][*neighbor_2] > distance_update)
			{
			    Distances[*neighbor_1][*neighbor_2] = distance_update;
			    vertex_Queue.insert(VertexPairQueue_multimap::value_type(distance_update, Vertex_pair(*neighbor_1, *neighbor_2)));
			}
		    }
		}
	    }
	    vertex_Queue.erase(vertex_Queue.begin());
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances)
    {
	all_pairs_Distances.clear();
	int N_Vertices = m_Vertices.size();

	for (int source_id = 0; source_id < N_Vertices; ++source_id)
	{
/*
	    Distances_vector source_Distances;
	    calc_SingleSourceShortestPathsBreadth(source_id, source_Distances);
*/
	    calc_SingleSourceShortestPathsBreadth(source_id);
	    all_pairs_Distances.push_back(m_Distances);
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	all_pairs_Distances.clear();
	int N_Vertices = m_Vertices.size();

	all_pairs_Distances.resize(N_Vertices);
	for (int i = 0; i < N_Vertices; ++i)
	{
	    all_pairs_Distances[i].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
/*
	    Distances_vector source_Distances;
	    calc_SingleSourceShortestPathsBreadth(*source, source_Distances);
*/
	    calc_SingleSourceShortestPathsBreadth(*source);

	    int vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		all_pairs_Distances[*source][vertex_id] = *distance;
		all_pairs_Distances[vertex_id][*source] = *distance;
		++vertex_id;
	    }
	}

	for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	{
/*
	    Distances_vector goal_Distances;
	    calc_SingleSourceShortestPathsBreadth(*goal, goal_Distances);
*/
	    calc_SingleSourceShortestPathsBreadth(*goal);

	    int vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		all_pairs_Distances[*goal][vertex_id] = *distance;
		all_pairs_Distances[vertex_id][*goal] = *distance;
		++vertex_id;
	    }
	}
    }


    void sUndirectedGraph::calc_SourceGoalShortestPaths(Distances_2d_vector &source_Distances, Distances_2d_vector &goal_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	source_Distances.clear();
	goal_Distances.clear();
	int N_Vertices = m_Vertices.size();

	source_Distances.resize(N_Vertices);
	goal_Distances.resize(N_Vertices);

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
	    source_Distances[*source].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	{
	    goal_Distances[*goal].resize(N_Vertices);
	}

	for (VertexIDs_vector::const_iterator source = source_IDs.begin(); source != source_IDs.end(); ++source)
	{
/*
	    Distances_vector source_Distances;
	    calc_SingleSourceShortestPathsBreadth(*source, source_Distances);
*/
	    calc_SingleSourceShortestPathsBreadth(*source);

	    int vertex_id = 0;

	    for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
	    {
		source_Distances[*source][vertex_id] = *distance;
		++vertex_id;
	    }
	}

	if (m_directed)
	{
	    for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	    {
/*
	    Distances_vector goal_Distances;
	    calc_SingleSourceShortestPathsBreadth(*goal, goal_Distances);
*/
		calc_SingleSourceShortestPathsBreadth_opposite(*goal);

		int vertex_id = 0;
		
		for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
		{
		    goal_Distances[*goal][vertex_id] = *distance;
		    ++vertex_id;
		}
	    }	    
	}
	else
	{
	    for (VertexIDs_vector::const_iterator goal = goal_IDs.begin(); goal != goal_IDs.end(); ++goal)
	    {
/*
	    Distances_vector goal_Distances;
	    calc_SingleSourceShortestPathsBreadth(*goal, goal_Distances);
*/
		calc_SingleSourceShortestPathsBreadth(*goal);

		int vertex_id = 0;
		
		for (Distances_vector::const_iterator distance = m_Distances.begin(); distance != m_Distances.end(); ++distance)
		{
		    goal_Distances[*goal][vertex_id] = *distance;
		    ++vertex_id;
		}
	    }
	}
    }


  void sUndirectedGraph::collect_EquidistantVertices(int s_id, int distance, VertexIDs_vector &equidistant_IDs)
  {
    	int N_Vertices = m_Vertices.size();
	m_Distances.resize(N_Vertices, sINT_32_MAX);
	m_Queue.resize(m_Vertices.size(), sINT_32_MAX);

	for (int i = 0; i < N_Vertices; ++i)
	{
	    m_Distances[i] = sINT_32_MAX;
	    m_Queue[i] = sINT_32_MAX;
	}

	m_Distances[s_id] = 0;

	int queue_bottom = 0;
	int queue_top = 0;
	m_Queue[queue_top++] = s_id;

	if (distance == 0)
	  {
	    equidistant_IDs.push_back(s_id);
	  }

	while (queue_top > queue_bottom)
	{
	    int front_id = m_Queue[queue_bottom];

	    for (sVertex::Neighbors_list::const_iterator neighbor = m_Vertices[front_id].m_Neighbors.begin(); neighbor != m_Vertices[front_id].m_Neighbors.end(); ++neighbor)
	    {
		int neighbor_id = (*neighbor)->m_target->m_id;

		if (m_Distances[neighbor_id] == sINT_32_MAX)
		{
		    m_Distances[neighbor_id] = m_Distances[front_id] + 1;
		    m_Queue[queue_top++] = neighbor_id;

		    if (m_Distances[neighbor_id] == distance)
		    {
			equidistant_IDs.push_back(neighbor_id);
		    }
		}
	    }
	    ++queue_bottom;
	}
	
  }

    
    void sUndirectedGraph::calc_AllPairsShortestPaths(void)
    {
	if (!m_all_pairs_distances_calculated)
	{
	    calc_AllPairsShortestPaths(m_all_pairs_Distances);
	    m_all_pairs_distances_calculated = true;
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	if (!m_all_pairs_distances_calculated)
	{
	    calc_AllPairsShortestPaths(m_all_pairs_Distances, source_IDs, goal_IDs);
	    m_all_pairs_distances_calculated = true;
	}
    }


    void sUndirectedGraph::calc_SourceGoalShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs)
    {
	if (!m_source_goal_distances_calculated)
	{
	    calc_SourceGoalShortestPaths(m_source_Distances, m_goal_Distances, source_IDs, goal_IDs);
	    m_source_goal_distances_calculated = true;
	}
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_AllPairsShortestPaths(void) const
    {
	sASSERT(m_all_pairs_distances_calculated);
	return m_all_pairs_Distances;
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_SourceShortestPaths(void) const
    {
	sASSERT(m_source_goal_distances_calculated);
	return m_source_Distances;
    }


    const sUndirectedGraph::Distances_2d_vector& sUndirectedGraph::get_GoalShortestPaths(void) const
    {
	sASSERT(m_source_goal_distances_calculated);
	return m_goal_Distances;
    }


    void sUndirectedGraph::calc_AllPairsShortestCoopPaths(Distances_4d_vector &all_pairs_Distances) const
    {
	all_pairs_Distances.clear();
	int N_Vertices = m_Vertices.size();

	for (int source_1_id = 0; source_1_id < N_Vertices; ++source_1_id)
	{
	    all_pairs_Distances.push_back(Distances_3d_vector());

	    for (int source_2_id = 0; source_2_id < N_Vertices; ++source_2_id)
	    {
		if (source_1_id != source_2_id)
		{
		    Distances_2d_vector source_Distances;
		    calc_SingleSourceShortestCoopPaths(source_1_id, source_2_id, source_Distances);
		    
		    all_pairs_Distances[source_1_id].push_back(source_Distances);
		}
		else
		{
		    Distances_2d_vector source_Distances;

		    source_Distances.resize(N_Vertices);
		    for (int i = 0; i < N_Vertices; ++i)
		    {
			source_Distances[i].resize(N_Vertices, sINT_32_MAX);
		    }
		    all_pairs_Distances[source_1_id].push_back(source_Distances);
		}
	    }
	}
    }


    void sUndirectedGraph::calc_AllPairsShortestCoopPaths(void)
    {
	if (!m_all_pairs_coop_distances_calculated)
	{
	    calc_AllPairsShortestCoopPaths(m_all_pairs_coop_Distances);
	    m_all_pairs_coop_distances_calculated = true;
	}
    }


    const sUndirectedGraph::Distances_4d_vector& sUndirectedGraph::get_AllPairsShortestCoopPaths(void) const
    {
	sASSERT(m_all_pairs_coop_distances_calculated);
	return m_all_pairs_coop_Distances;
    }


    void sUndirectedGraph::build_SpanningTree(int root_id, sUndirectedGraph &spanning_tree)
    {
	spanning_tree = sUndirectedGraph();
	spanning_tree.add_Vertices(get_VertexCount());

	extend_SpanningTree(root_id, spanning_tree);
    }

    
    void sUndirectedGraph::extend_SpanningTree(int root_id, sUndirectedGraph &spanning_tree)
    {
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->m_visited = false;
	}

	build_SpanningTreeDFS(root_id, spanning_tree);
	/*
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		if (!spanning_tree.is_Adjacent(vertex->m_id, (*neighbor)->m_target->m_id))
		{
		    double p = (double)rand() / RAND_MAX;		
		    if (p < 0.3)
		    {
			spanning_tree.add_Edge(vertex->m_id, (*neighbor)->m_target->m_id);
		    }
		}
		
	    }	
	}
	*/
	/*
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->m_visited = false;
	}
	build_SpanningTreeDFS(root_id, spanning_tree);
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->m_visited = false;
	}
	build_SpanningTreeDFS(root_id, spanning_tree);
	*/
    }


    void sUndirectedGraph::build_SpanningTreeDFS(int root_id, sUndirectedGraph &spanning_tree)
    {
	sVertex &vertex = m_Vertices[root_id];
	vertex.m_visited = true;

	int N_unvisited = 0;
	for (sVertex::Neighbors_list::iterator neighbor = vertex.m_Neighbors.begin(); neighbor != vertex.m_Neighbors.end(); ++neighbor)
	{
	    if (!(*neighbor)->m_target->m_visited)
	    {
		++N_unvisited;
	    }
	}
	while (N_unvisited > 0)
	{
	    int selected = rand() % N_unvisited;

	    for (sVertex::Neighbors_list::iterator neighbor = vertex.m_Neighbors.begin(); neighbor != vertex.m_Neighbors.end(); ++neighbor)
	    {
		if (!(*neighbor)->m_target->m_visited)
		{
		    if (selected == 0)
		    {		    
			if (!spanning_tree.is_Adjacent(root_id, (*neighbor)->m_target->m_id))
			{
			    spanning_tree.add_Edge(root_id, (*neighbor)->m_target->m_id);
//			    spanning_tree.add_Arrow(root_id, (*neighbor)->m_target->m_id);
			}
			build_SpanningTreeDFS((*neighbor)->m_target->m_id, spanning_tree);
			break;
		    }
		    else
		    {
			--selected;
		    }
		}
	    }
	    N_unvisited = 0;

	    for (sVertex::Neighbors_list::iterator neighbor = vertex.m_Neighbors.begin(); neighbor != vertex.m_Neighbors.end(); ++neighbor)
	    {
		if (!(*neighbor)->m_target->m_visited)
		{
		    ++N_unvisited;
		}
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::find_ShortestPath(int u_id, int v_id, VertexIDs_list &path)
    {
	for (Vertices_vector::iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->m_distance = INT_MAX;
	    vertex->m_prev_id = -1;
	}
	find_ShortestPathBFS(u_id, v_id);

	int path_id = get_Vertex(v_id)->m_id;
	while (path_id != -1)
	{
	    path.push_front(path_id);
	    path_id = get_Vertex(path_id)->m_prev_id;
	}
    }


    void sUndirectedGraph::find_ShortestPathBFS(int u_id, int v_id)
    {
	Vertices_list queue;

	sVertex *first_vertex = get_Vertex(u_id);
	first_vertex->m_distance = 0;
	first_vertex->m_prev_id = -1;
	queue.push_back(first_vertex);

	while (!queue.empty())
	{
	    sVertex *vertex = queue.front();
	    queue.pop_front();

	    for (sVertex::Neighbors_list::iterator neighbor = vertex->m_Neighbors.begin(); neighbor != vertex->m_Neighbors.end(); ++neighbor)
	    {
		if ((*neighbor)->m_target->m_distance > vertex->m_distance + 1)
		{
		    (*neighbor)->m_target->m_distance = vertex->m_distance + 1;
		    (*neighbor)->m_target->m_prev_id = vertex->m_id;

		    queue.push_back((*neighbor)->m_target);

		    if ((*neighbor)->m_target->m_id == v_id)
		    {
			return;
		    }
		}
	    }	    
	}
    }


/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::build_SparseGraph(const VertexPairs_vector &vertex_Pairs, sUndirectedGraph &sparse_graph)
    {
	sparse_graph = sUndirectedGraph();
	sparse_graph.add_Vertices(get_VertexCount());

	extend_SparseGraph(vertex_Pairs, sparse_graph);
    }

    
    void sUndirectedGraph::extend_SparseGraph(const VertexPairs_vector &vertex_Pairs, sUndirectedGraph &sparse_graph)
    {
	sparse_graph = sUndirectedGraph();
	sparse_graph.add_Vertices(get_VertexCount());

	for (VertexPairs_vector::const_iterator vertex_pair = vertex_Pairs.begin(); vertex_pair != vertex_Pairs.end(); ++vertex_pair)
	{
	    VertexIDs_list path;
	    find_ShortestPath(vertex_pair->first, vertex_pair->second, path);

	    VertexIDs_list::const_iterator path_vertex_1 = path.begin();
	    VertexIDs_list::const_iterator path_vertex_2 = path_vertex_1;
	    ++path_vertex_2;

	    while (path_vertex_2 != path.end())
	    {
		if (!sparse_graph.is_LinkedTo(*path_vertex_1, *path_vertex_2) && !sparse_graph.is_LinkedTo(*path_vertex_2, *path_vertex_1))
		{
		    sparse_graph.add_Edge((*path_vertex_1), (*path_vertex_2));
//		    sparse_graph.add_Arrow((*path_vertex_1), (*path_vertex_2));
		}
		++path_vertex_1;
		++path_vertex_2;
	    }
	}
    }


/*----------------------------------------------------------------------------*/

    void sUndirectedGraph::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sUndirectedGraph::to_Screen_vertices(const sString &indent) const
    {
	to_Stream_vertices(stdout, indent);
    }


    void sUndirectedGraph::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sUndirected graph: (|V|=%ld |E|=%ld) [\n", indent.c_str(), m_Vertices.size(), m_Edges.size());

	for (Vertices_vector::const_iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->to_Stream(fw, indent + sRELOC_INDENT);
	}

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    edge->to_Stream(fw, indent + sRELOC_INDENT);
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }


    void sUndirectedGraph::to_Stream_vertices(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sUndirected graph: (|V|=%ld, |E|=%ld) [\n", indent.c_str(), m_Vertices.size(), m_Edges.size());

	for (Vertices_vector::const_iterator vertex = m_Vertices.begin(); vertex != m_Vertices.end(); ++vertex)
	{
	    vertex->to_Stream(fw, indent + sRELOC_INDENT);
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }


    sResult sUndirectedGraph::from_File_multirobot(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_multirobot(FILE *fr)
    {
	m_Vertices.clear();
	m_Edges.clear();

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
	c = fgetc(fr);

	while (c == '(')
	{
	    add_Vertex();
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    int x_id;

		    while (c != '>')
		    {
			fscanf(fr, "%d", &x_id);
			m_Vertices.back().m_Conflicts.push_back(x_id);
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}
	    }
	    c = fgetc(fr);
	}

	fscanf(fr, " =\n");
	c = fgetc(fr);

	while (c == (int)'{')
	{
	    int u_id, v_id;
	    fscanf(fr, "%d,%d", &u_id, &v_id);

	    if (m_directed)
	    {
		add_Arrow(u_id, v_id);
	    }
	    else
	    {
		add_Edge(u_id, v_id);
	    }
	    if (c != '\n' && c != '<')
	    {
		while((c = fgetc(fr)) != '\n' && c != '<');
		if (c == '<')
		{
		    int x_id, y_id;

		    while (c != '>')
		    {
			fscanf(fr, "{%d,%d}", &x_id, &y_id);
			m_Edges.back().m_Conflicts.push_back(sEdge::Conflict(x_id, y_id));
			
			while((c = fgetc(fr)) == ' ');
			ungetc(c, fr);
		    }
		    if (c == '>')
		    {
			fgetc(fr);
		    }
		    fscanf(fr, "\n");
		}		
	    }
	    c = fgetc(fr);
	}

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::to_File_multirobot(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_multirobot(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sUndirectedGraph::to_Stream_multirobot(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sE =\n", indent.c_str());

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    fprintf(fw, "%s{%d,%d} (-1)\n", indent.c_str(), edge->m_arc_uv.m_source->m_id, edge->m_arc_uv.m_target->m_id);
	}
    }


    sResult sUndirectedGraph::from_File_map(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_map(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_map(FILE *fr)
    {
	char type_ignore[128];
	int x_size, y_size;
	fscanf(fr, "type %s\n", type_ignore);
	fscanf(fr, "height %d\n", &y_size);
	fscanf(fr, "width %d\n", &x_size);

	fscanf(fr, "map\n");

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_bgu(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_bgu(FILE *fr)
    {
	char type_ignore[256];
	int x_size, y_size, id;
	fscanf(fr, "%d,%s\n", &id, type_ignore);
	fscanf(fr, "Grid:\n");
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_usc(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_usc(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_usc(FILE *fr)
    {
	int x_size, y_size;
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
	
	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_File_lusc(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	result = from_Stream_lusc(fr);
	if (sFAILED(result))
	{
	    fclose(fr);
	    return result;
	}
	fclose(fr);

	return sRESULT_SUCCESS;
    }


    sResult sUndirectedGraph::from_Stream_lusc(FILE *fr)
    {
	int x_size, y_size;
	fscanf(fr, "%d,%d\n", &y_size, &x_size);

	m_x_size = x_size;
	m_y_size = y_size;

	m_Matrix = new int[x_size * y_size];
	int cnt = 0;

	for (int j = 0; j < y_size; ++j)
	{
	    for (int i = 0; i < x_size; ++i)
	    {
		char ch;

		fscanf(fr, "%c", &ch);
		if (ch == '.')
		{
		    m_Matrix[j * x_size + i] = cnt++;
		}
		else
		{
		    m_Matrix[j * x_size + i] = -1;
		}
		printf("%c", ch);
	    }
	    printf("\n");
	    fscanf(fr, "\n");
	}
	add_Vertices(cnt);

	for (int j = 0; j < y_size - 1; ++j)
	{
	    for (int i = 0; i < x_size - 1; ++i)
	    {
		int u_id = j * x_size + i;
		int v_id = (j + 1) * x_size + i;
		int w_id = j * x_size + i + 1;

		if (m_Matrix[u_id] != -1)
		{
		    if (m_Matrix[v_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
		    }
		    if (m_Matrix[w_id] != -1)
		    {
			add_Edge(m_Matrix[u_id], m_Matrix[w_id]);
		    }
		}
	    }
	}
	for (int j = 0; j < y_size - 1; ++j)
	{
	    int u_id = (j + 1) * x_size - 1;
	    int v_id = (j + 2) * x_size - 1;

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	for (int i = 0; i < x_size - 1; ++i)
	{
	    int u_id = (y_size - 1) * x_size + i;
	    int v_id = (y_size - 1) * x_size + (i + 1);

	    if (m_Matrix[u_id] != -1 && m_Matrix[v_id] != -1)
	    {
		add_Edge(m_Matrix[u_id], m_Matrix[v_id]);
	    }
	}
	initialize_InverseMatrix();
		
	return sRESULT_SUCCESS;
    }        


    sResult sUndirectedGraph::to_File_dibox(const sString &filename, const sString &indent) const
    {
	FILE *fw;

	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
	}
	
	to_Stream_dibox(fw, indent);
	fclose(fw);

	return sRESULT_SUCCESS;	
    }

    
    void sUndirectedGraph::to_Stream_dibox(FILE *fw, const sString &sUNUSED(indent)) const
    {
	char name_ignore[] = "anonymous";
	fprintf(fw, "%s\n", name_ignore);

	int N_Vertices = m_Vertices.size();
	int N_Edges = m_Edges.size();
	
	fprintf(fw, "NUMBER OF VERTICES:%d\n", N_Vertices);
	fprintf(fw, "NUMBER OF EDGES:%d\n", N_Edges);
	fprintf(fw, "LIST OF EDGES:\n");

	for (Edges_list::const_iterator edge = m_Edges.begin(); edge != m_Edges.end(); ++edge)
	{
	    int u_id, v_id;

	    u_id = edge->m_arc_uv.m_source->m_id;
	    v_id = edge->m_arc_uv.m_target->m_id;
	    
	    fprintf(fw, "(%d,%d)\n", u_id + 1, v_id + 1);

	}
    }
    

    sResult sUndirectedGraph::from_File_dibox(const sString &filename)
    {
	sResult result;
	FILE *fr;

	if ((fr = fopen(filename.c_str(), "r")) == NULL)
	{
	    return sUNDIRECTED_GRAPH_OPEN_ERROR;
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


    sResult sUndirectedGraph::from_Stream_dibox(FILE *fr)
    {
	char name_ignore[256];
	fscanf(fr, "%s\n", name_ignore);

	int N_Vertices, N_Edges;
	
	fscanf(fr, "NUMBER OF VERTICES:%d\n", &N_Vertices);
	fscanf(fr, "NUMBER OF EDGES:%d\n", &N_Edges);
	fscanf(fr, "LIST OF EDGES:\n");

	add_Vertices(N_Vertices);
	
	for (int i = 0; i < N_Edges; ++i)
	{
	    int u_id, v_id;
	    fscanf(fr, "(%d,%d)\n", &u_id, &v_id);

	    add_Arrow(u_id - 1, v_id - 1);
	}

	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/
// sVectorGraph

    sVectorGraph::sVectorGraph()
    {
	// nothing
    }


    sVectorGraph::sVectorGraph(const sUndirectedGraph &undirected_graph)
    {
	if (undirected_graph.is_Grid())
	{
	    int grid_height = undirected_graph.get_GridHeight();
	    int grid_width = undirected_graph.get_GridWidth();
	    m_Connections.resize(grid_height);

	    for (int i = 0; i < grid_height; ++i)
	    {
		m_Connections[i].resize(grid_width);
		
		for (int j = 0; j < grid_width; ++j)
		{
		    m_Connections[i][j] = (undirected_graph.get_GridCell(i, j) >= 0) ? 1 : 0;
		}
	    }

	}
	else
	{
	    int N_vertices = undirected_graph.get_VertexCount();
	    m_Connections.resize(N_vertices);

	    for (int i = 0; i < N_vertices; ++i)
	    {
		m_Connections[i].resize(N_vertices);
		
		for (int j = 0; j < N_vertices; ++j)
		{
		    m_Connections[i][j] = undirected_graph.is_Adjacent(i, j) ? 1 : 0;
		}
	    }
	}
    }


    sVectorGraph::~sVectorGraph()
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    void sVectorGraph::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sVectorGraph::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sVector Graph: (height=%ld width=%ld) [\n", indent.c_str(), m_Connections.size(), m_Connections[0].size());

	int N_vertices = m_Connections.size();
	for (int i = 0; i < N_vertices; ++i)
	{
	    fprintf(fw, "%s", indent.c_str());
	    for (int j = 0; j < N_vertices; ++j)
	    {
		fprintf(fw, "%d ", m_Connections[i][j]);
	    }
	    fprintf(fw, "\n");
	}
	fprintf(fw, "%s]\n", indent.c_str());
    }

/*----------------------------------------------------------------------------*/

} // namespace sReloc
