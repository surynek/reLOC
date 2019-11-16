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
/* graph.cpp / 0.21-robik_013                                                 */
/*----------------------------------------------------------------------------*/
//
// Algorithmic graph theory.
//
/*----------------------------------------------------------------------------*/

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "reloc.h"
#include "types.h"
#include "statistics.h"
#include "reloc.h"
#include "graph.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{




/*============================================================================*/
// sSerializable class
/*----------------------------------------------------------------------------*/
// public methods of the sSerializable class
/*----------------------------------------------------------------------------*/

    sString sSerializable::to_String_tag(const sString &begin_tag,
					 const sString &end_tag,
					 int           &int_content,
					 const sString &indent)
    {
	sString output;
	output += indent + begin_tag + sInt_32_to_String(int_content) + end_tag;
	
	return output;
    }
    
    
    sString sSerializable::to_String_tag(const sString &begin_tag,
					 const sString &end_tag,
					 const sString &string_content,
					 const sString &indent)
    {
	sString output;
	output += indent + begin_tag + string_content + end_tag;
	
	return output;
    }
    
    
    sString sSerializable::to_String_tag(const sString       &begin_tag,
					 const sString       &end_tag,
					 const sSerializable &serial_content,
					 const sString       &indent)
    {
	sString output;
	output += indent + begin_tag + "\n";
	output += serial_content.to_String(indent + sRELOC_INDENT) + "\n";
	output += indent +  end_tag;

	return output;
    }

    
    sString sSerializable::to_String_attribute(const sString &attribute, int int_value)
    {
	sString output;
	output += attribute + "='" + sInt_32_to_String(int_value) + "'";
	
	return output;
    }
    

    sString sSerializable::to_String_attribute(const sString &attribute, double double_value)
    {
	sString output;
	output += attribute + "='" + sDouble_to_String(double_value) + "'";

	return output;
    }


    sString sSerializable::to_String_attribute(const sString &attribute, const sString &string_value)
    {
	sString output;
	output += attribute + "='" + string_value + "'";

	return output;
    }


/*----------------------------------------------------------------------------*/

    void sSerializable::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }


    void sSerializable::to_Stream(FILE *fw, const sString &indent) const
    {
	sString output = to_String(indent);
	fprintf(fw, "%s", output.c_str());
    }





/*============================================================================*/
// sVertexKey class
/*----------------------------------------------------------------------------*/

    sVertexKey::sVertexKey(sDigraph *graph, const sString &identifier)
	: m_graph(graph)
	, m_identifier(identifier)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
// public methods of the sVertexKey class
/*----------------------------------------------------------------------------*/

    int sVertexKey::compare(const sVertexKey &vertex) const
    {
	return (m_identifier < vertex.m_identifier ? -1 : (m_identifier > vertex.m_identifier ? 1 : 0));
    }
    

    int sVertexKey::compare(const sVertexKey &vertex_A, const sVertexKey &vertex_B) const
    {
	return (vertex_A.m_identifier < vertex_B.m_identifier ? -1 : (vertex_A.m_identifier > vertex_B.m_identifier ? 1 : 0));
    }




/*============================================================================*/
// sVertex_ class
/*----------------------------------------------------------------------------*/

    sVertex_::sVertex_(sVertexKey *vertex_key)
	: m_vertex_key(vertex_key)
	, m_excess(0)
	, m_height(0)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
// public methods of the sVertex_ class
/*----------------------------------------------------------------------------*/

    sString sVertex_::to_String(const sString &indent) const
    {
	sString output;
	
	output +=   indent + "<vertex " + to_String_attribute("identifier", m_vertex_key->m_identifier) + " "
		  + to_String_attribute("excess", m_excess) + " " + to_String_attribute("height", m_height) + ">" + "\n";
	output += to_String_container<Arcs_list>("<in_arcs>", "</in_arcs>", &m_in_arcs, indent + sRELOC_INDENT) + "\n";
	output += to_String_container<Arcs_list>("<out_arcs>", "</out_arcs>", &m_out_arcs, indent + sRELOC_INDENT) + "\n";
	output += indent + "</vertex>";

	return output;
    }




/*============================================================================*/
// sArcKey class
/*----------------------------------------------------------------------------*/

    sArcKey::sArcKey(sDigraph *graph, sVertex_ *first_vertex, sVertex_ *second_vertex)
	: m_graph(graph)
	, m_first_vertex(first_vertex)
	, m_second_vertex(second_vertex)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
// public methods of the sArcKey class
/*----------------------------------------------------------------------------*/

    int sArcKey::compare(const sArcKey &arc) const
    {
	return (m_first_vertex < arc.m_first_vertex ? -1 :
		(m_first_vertex > arc.m_first_vertex ? 1 : 
		 (m_second_vertex < arc.m_second_vertex ? -1 :
		  (m_second_vertex > arc.m_second_vertex ? 1 : 0))));
    }


    int sArcKey::compare(const sArcKey &arc_A, const sArcKey &arc_B) const
    {
	return (arc_A.m_first_vertex < arc_B.m_first_vertex ? -1 :
		(arc_A.m_first_vertex > arc_B.m_first_vertex ? 1 : 
		 (arc_A.m_second_vertex < arc_B.m_second_vertex ? -1 :
		  (arc_A.m_second_vertex > arc_B.m_second_vertex ? 1 : 0))));
    }




/*============================================================================*/
// sArc_ class
/*----------------------------------------------------------------------------*/

    sArc_::sArc_(sArcKey *arc_key, int capacity, int flow)
	: m_arc_key(arc_key)
	, m_capacity(capacity)
	, m_flow(flow)
    {
	// nothing
    }
   
 
/*----------------------------------------------------------------------------*/
// public methods of the sArc_ class
/*----------------------------------------------------------------------------*/

    sString sArc_::to_String(const sString &indent) const
    {
	sString output;

	output += indent + "<arc " + to_String_attribute("capacity", m_capacity) + " " + to_String_attribute("flow", m_flow) + ">\n";
	output += to_String_tag("<first_vertex>", "</first_vertex>", m_arc_key->m_first_vertex->m_vertex_key->m_identifier, indent + sRELOC_INDENT) + "\n";
	output += to_String_tag("<second_vertex>", "</second_vertex>", m_arc_key->m_second_vertex->m_vertex_key->m_identifier, indent + sRELOC_INDENT) + "\n";
	output += indent + "</arc>";

	return output;
    }




/*============================================================================*/
// sDigraph class
/*----------------------------------------------------------------------------*/

    sDigraph::sDigraph()
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
// public methods of the sDigraph class
/*----------------------------------------------------------------------------*/

    sDigraph::Vertices_map::iterator sDigraph::add_Vertex(const sString &identifier)
    {
	sVertexKey vertex_key(this, identifier);
	
	std::pair<Vertices_map::iterator, bool> insert_result = m_vertices.insert(Vertices_map::value_type(vertex_key, sVertex_(NULL)));
	insert_result.first->second.m_vertex_key = const_cast<sVertexKey*>(&insert_result.first->first);
	
	return insert_result.first;
    }


    sDigraph::Vertices_map::iterator sDigraph::add_Vertex(const sVertex_ &vertex)
    {
	std::pair<Vertices_map::iterator, bool> insert_result = m_vertices.insert(Vertices_map::value_type(*vertex.m_vertex_key, vertex));
	insert_result.first->second.m_vertex_key = const_cast<sVertexKey*>(&insert_result.first->first);
	
	return insert_result.first;
    }


    void sDigraph::remove_Vertex(const sString &identifier)
    {
	sVertexKey vertex_key(this, identifier);
	
	Vertices_map::iterator vertex_erase = m_vertices.find(vertex_key);
	sASSERT(vertex_erase->second.m_in_arcs.empty() && vertex_erase->second.m_out_arcs.empty());
	m_vertices.erase(vertex_erase);
    }


    void sDigraph::remove_Vertex(Vertices_map::iterator vertex)
    {
	sASSERT(vertex->second.m_in_arcs.empty() && vertex->second.m_out_arcs.empty());
	m_vertices.erase(vertex);
    }


    sDigraph::Arcs_map::iterator sDigraph::add_Arc(const sString &first_vertex_id, const sString &second_vertex_id, int capacity, int flow)
    {
	Vertices_map::iterator first_vertex_iter = m_vertices.find(sVertexKey(this, first_vertex_id));
	sASSERT(first_vertex_iter != m_vertices.end());
	Vertices_map::iterator second_vertex_iter = m_vertices.find(sVertexKey(this, second_vertex_id));
	sASSERT(second_vertex_iter != m_vertices.end());

	sArcKey arc_key(this, &first_vertex_iter->second, &second_vertex_iter->second);

	std::pair<Arcs_map::iterator, bool> insert_result = m_arcs.insert(Arcs_map::value_type(arc_key, sArc_(NULL, capacity, flow)));
	insert_result.first->second.m_arc_key = const_cast<sArcKey*>(&insert_result.first->first);

	first_vertex_iter->second.m_out_arcs.push_back(&insert_result.first->second);
	second_vertex_iter->second.m_in_arcs.push_back(&insert_result.first->second);

	return insert_result.first;
    }


    sDigraph::Arcs_map::iterator sDigraph::add_Arc(const sArc_ &arc)
    {
	std::pair<Arcs_map::iterator, bool> insert_result = m_arcs.insert(Arcs_map::value_type(*arc.m_arc_key, arc));
	insert_result.first->second.m_arc_key = const_cast<sArcKey*>(&insert_result.first->first);

	arc.m_arc_key->m_first_vertex->m_out_arcs.push_back(&insert_result.first->second);
	arc.m_arc_key->m_second_vertex->m_in_arcs.push_back(&insert_result.first->second);

	return insert_result.first;
    }


    void sDigraph::remove_Arc(const sString &first_vertex_id, const sString &second_vertex_id)
    {
	Vertices_map::iterator first_vertex_iter = m_vertices.find(sVertexKey(this, first_vertex_id));
	sASSERT(first_vertex_iter != m_vertices.end());
	Vertices_map::iterator second_vertex_iter = m_vertices.find(sVertexKey(this, second_vertex_id));
	sASSERT(second_vertex_iter != m_vertices.end());

	sArcKey arc_key(this, &first_vertex_iter->second, &second_vertex_iter->second);
	Arcs_map::iterator arc_erase = m_arcs.find(sArcKey(this, &first_vertex_iter->second, &second_vertex_iter->second));
	sASSERT(arc_erase != m_arcs.end());

	first_vertex_iter->second.m_out_arcs.push_back(&arc_erase->second);
	second_vertex_iter->second.m_in_arcs.push_back(&arc_erase->second);

	m_arcs.erase(arc_erase);
    }


    void sDigraph::remove_Arc(Arcs_map::iterator arc)
    {
	arc->first.m_first_vertex->m_out_arcs.push_back(&arc->second);
	arc->first.m_second_vertex->m_in_arcs.push_back(&arc->second);

	m_arcs.erase(arc);
    }


    void sDigraph::clean_Graph(void)
    {
	m_vertices.clear();
	m_arcs.clear();
    }


/*----------------------------------------------------------------------------*/

    sString sDigraph::to_String(const sString &indent) const
    {
	sString output;

	output += indent + "<graph>\n";
	output += to_String_associative<Vertices_map>("<vertices>", "</vertices>", m_vertices, indent + sRELOC_INDENT) + "\n";
	output += to_String_associative<Arcs_map>("<arcs>", "</arcs>", m_arcs, indent + sRELOC_INDENT) + "\n";
	output += indent + "</graph>\n";

	return output;
    }




/*============================================================================*/
// sGoldberg class
/*----------------------------------------------------------------------------*/

    sGoldberg::sGoldberg()
	: m_source(NULL)
	, m_sink(NULL)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/
// public methods of the sGoldberg class
/*----------------------------------------------------------------------------*/

    void sGoldberg::init(sDigraph *graph, sVertex_ *source, sVertex_ *sink, Vertices_list &rlb)
    {
	m_source = source;
	m_sink = sink;
	
	for (sDigraph::Vertices_map::iterator vertex = graph->m_vertices.begin(); vertex != graph->m_vertices.end(); ++vertex)
	{
	    vertex->second.m_height = 0;
	    vertex->second.m_excess = 0;
	    
	    vertex->second.m_out_farcs.clear();
	    vertex->second.m_out_garcs.clear();
	}
	for (sDigraph::Arcs_map::iterator arc = graph->m_arcs.begin(); arc != graph->m_arcs.end(); ++arc)
	{
	    arc->second.m_flow = 0;
	    arc->second.m_arc_key->m_first_vertex->m_out_farcs.insert(&arc->second);
	}
	source->m_height = graph->m_vertices.size();
	
	for (sVertex_::Arcs_list::iterator arc = source->m_out_arcs.begin(); arc != source->m_out_arcs.end(); ++arc)
	{
	    (*arc)->m_flow = (*arc)->m_capacity;
	    (*arc)->m_arc_key->m_second_vertex->m_excess = (*arc)->m_capacity;
	    source->m_excess -= (*arc)->m_capacity;
	    
	    source->m_out_farcs.erase(*arc);
	    (*arc)->m_arc_key->m_second_vertex->m_out_garcs.insert(*arc);
	    
	    if ((*arc)->m_arc_key->m_second_vertex != m_sink)
	    {
		rlb.push_back((*arc)->m_arc_key->m_second_vertex);
	    }
	}
    }


    void sGoldberg::push_In(sArc_ *arc, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const
    {
	if (/*arc->m_arc_key->m_first_vertex != m_source && arc->m_arc_key->m_second_vertex != m_sink &&*/ arc->m_arc_key->m_second_vertex->m_excess > 0)
	{
	    bool exceeding;
	    bool satur = true;
	    bool empty = true;

	    if (arc->m_capacity - arc->m_flow > 0)
	    {
		satur = false;
	    }
	    if (arc->m_flow > 0)
	    {
		empty = false;
	    }
	    
	    int delta = sMIN(arc->m_arc_key->m_second_vertex->m_excess, arc->m_flow);
	    
	    if (arc->m_arc_key->m_first_vertex->m_excess == 0)
	    {
		exceeding = false;
	    }
	    else
	    {
		exceeding = true;
	    }
	    
	    if (delta > 0)
	    {
		arc->m_arc_key->m_first_vertex->m_excess += delta;
		arc->m_arc_key->m_second_vertex->m_excess -= delta;
		arc->m_flow -= delta;
		
		if (arc->m_capacity - arc->m_flow > 0)
		{
		    if (satur)
		    {
			arc->m_arc_key->m_first_vertex->m_out_farcs.insert(arc);
		    }
		}
		else
		{
		    if (!satur)
		    {
			arc->m_arc_key->m_first_vertex->m_out_farcs.erase(arc);
		    }
		}
		
		if (arc->m_flow > 0)
		{
		    if (empty)
		    {
			arc->m_arc_key->m_second_vertex->m_out_garcs.insert(arc);
		    }
		}
		else
		{
		    if (!empty)
		    {
			arc->m_arc_key->m_second_vertex->m_out_garcs.erase(arc);
		    }
		}
		if (arc->m_arc_key->m_first_vertex != m_source && arc->m_arc_key->m_first_vertex != m_sink)
		{
		    rlb.push_back(arc->m_arc_key->m_first_vertex);
		}
		if (!exceeding)
		{
		    for (sVertex_::Arcs_set::const_iterator out_farc = arc->m_arc_key->m_first_vertex->m_out_farcs.begin();
			 out_farc != arc->m_arc_key->m_first_vertex->m_out_farcs.end(); ++out_farc)
		    {
			if ((*out_farc)->m_arc_key->m_first_vertex->m_height > (*out_farc)->m_arc_key->m_second_vertex->m_height)
			{
			    push_out.push_back(*out_farc);
			}
		    }
		    for (sVertex_::Arcs_set::const_iterator out_garc = arc->m_arc_key->m_first_vertex->m_out_garcs.begin();
			 out_garc != arc->m_arc_key->m_first_vertex->m_out_garcs.end(); ++out_garc)
		    {
			if ((*out_garc)->m_arc_key->m_first_vertex->m_height < (*out_garc)->m_arc_key->m_second_vertex->m_height)
			{
			    push_in.push_back(*out_garc);
			}
		    }
		}
	    }
	}
    }
    

    void sGoldberg::push_Out(sArc_ *arc, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const
    {
	if (/*arc->m_arc_key->m_first_vertex != m_sink && arc->m_arc_key->m_second_vertex != m_source &&*/ arc->m_arc_key->m_first_vertex->m_excess > 0)
	{
	    bool exceeding;
	    bool satur = true;
	    bool empty = true;
	    
	    if (arc->m_capacity - arc->m_flow > 0)
	    {
		satur = false;
	    }
	    if (arc->m_flow > 0)
	    {
		empty = false;
	    }
	    
	    int delta = sMIN(arc->m_arc_key->m_first_vertex->m_excess, arc->m_capacity - arc->m_flow);
	    
	    if (arc->m_arc_key->m_second_vertex->m_excess == 0)
	    {
		exceeding = false;
	    }
	    else
	    {
		exceeding = true;
	    }
	    
	    if (delta > 0)
	    {
		arc->m_arc_key->m_first_vertex->m_excess -= delta;
		arc->m_arc_key->m_second_vertex->m_excess += delta;
		arc->m_flow += delta;
		
		if (arc->m_capacity - arc->m_flow > 0)
		{
		    if (satur)
		    {
			arc->m_arc_key->m_first_vertex->m_out_farcs.insert(arc);
		    }
		}
		else
		{
		    if (!satur)
		    {
			arc->m_arc_key->m_first_vertex->m_out_farcs.erase(arc);
		    }
		}
		
		if (arc->m_flow > 0)
		{
		    if (empty)
		    {
			arc->m_arc_key->m_second_vertex->m_out_garcs.insert(arc);
		    }
		}
		else
		{
		    if (!empty)
		    {
			arc->m_arc_key->m_second_vertex->m_out_garcs.insert(arc);
		    }
		}
		
		if (arc->m_arc_key->m_second_vertex != m_source && arc->m_arc_key->m_second_vertex != m_sink)
		{
		    rlb.push_back(arc->m_arc_key->m_second_vertex);
		}
		
		if (!exceeding)
		{
		    for (sVertex_::Arcs_set::const_iterator out_farc = arc->m_arc_key->m_second_vertex->m_out_farcs.begin(); out_farc != arc->m_arc_key->m_second_vertex->m_out_farcs.end(); ++out_farc)
		    {
			if ((*out_farc)->m_arc_key->m_first_vertex->m_height > (*out_farc)->m_arc_key->m_second_vertex->m_height)
			{
			    push_out.push_back(*out_farc);
			}
		    }
		    for (sVertex_::Arcs_set::const_iterator out_garc = arc->m_arc_key->m_second_vertex->m_out_garcs.begin(); out_garc != arc->m_arc_key->m_second_vertex->m_out_garcs.end(); ++out_garc)
		    {
			if ((*out_garc)->m_arc_key->m_first_vertex->m_height < (*out_garc)->m_arc_key->m_second_vertex->m_height)
			{
			    push_in.push_back(*out_garc);
			}
		    }
		}
	    }
	}
    }
    
    
    void sGoldberg::relabel(sVertex_ *vertex, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const
    {
	if (vertex != m_source && vertex != m_sink && vertex->m_excess > 0)
	{
	    int height = INT_MAX;
	    Arcs_list push_in_temp, push_out_temp;
	    
	    for (sVertex_::Arcs_set::const_iterator out_farc = vertex->m_out_farcs.begin();  out_farc != vertex->m_out_farcs.end(); ++out_farc)
	    {
		if (vertex->m_height <= (*out_farc)->m_arc_key->m_second_vertex->m_height)
		{
		    if (height > (*out_farc)->m_arc_key->m_second_vertex->m_height)
		    {
			height = (*out_farc)->m_arc_key->m_second_vertex->m_height;
			push_out_temp.clear();
			push_out_temp.push_back(*out_farc);
		    }
		    else
		    {
			if (height == (*out_farc)->m_arc_key->m_second_vertex->m_height)
			{
			    push_out_temp.push_back(*out_farc);
			}
		    }
		}
		else
		{
		    return;
		}
	    }
	    
	    for (sVertex_::Arcs_set::const_iterator out_garc = vertex->m_out_garcs.begin(); out_garc != vertex->m_out_garcs.end(); ++out_garc)
	    {
		if (vertex->m_height <= (*out_garc)->m_arc_key->m_first_vertex->m_height)
		{
		    if (height > (*out_garc)->m_arc_key->m_first_vertex->m_height)
		    {
			height = (*out_garc)->m_arc_key->m_first_vertex->m_height;
			push_in_temp.clear();
			push_out_temp.clear();
			push_in_temp.push_back(*out_garc);
		    }
		    else
		    {
			if (height == (*out_garc)->m_arc_key->m_first_vertex->m_height)
			{
			    push_in_temp.push_back(*out_garc);
			}
		    }
		}
		else
		{
		    return;
		}
	    }
	    
	    if (height < INT_MAX)
	    {
		vertex->m_height = height + 1;
		
		for (Arcs_list::const_iterator in_arc = push_in_temp.begin();
		     in_arc != push_in_temp.end(); ++in_arc)
		{
		    rlb.push_back((*in_arc)->m_arc_key->m_first_vertex);
		    push_in.push_back(*in_arc);
		    
		}
		
		for (Arcs_list::const_iterator out_arc = push_out_temp.begin();
		     out_arc != push_out_temp.end(); ++out_arc)
		{
		    rlb.push_back((*out_arc)->m_arc_key->m_second_vertex);
		    push_out.push_back(*out_arc);
		    
		}
	    }
	}
    }
    
    
    int sGoldberg::compute_Flow(sDigraph *graph, sVertex_ *source, sVertex_ *sink)
    {
	Vertices_list rlb;
	Arcs_list push_in, push_out;
	
	init(graph, source, sink, rlb);
	
	while (!rlb.empty())
	{
	    while (push_in.empty() && push_out.empty() && !rlb.empty())
	    {
		sVertex_ *vertex = rlb.front();
		rlb.pop_front();
		relabel(vertex, rlb, push_in, push_out);
	    }
	    while (!push_in.empty())
	    {
		sArc_ *arc = push_in.front();
		push_in.pop_front();
		push_In(arc, rlb, push_in, push_out);
	    }
	    while (!push_out.empty())
	    {
		sArc_ *arc = push_out.front();
		push_out.pop_front();
		push_Out(arc, rlb, push_in, push_out);
	    }
	}
	
	int flow_size = 0;
	
	for (sVertex_::Arcs_list::const_iterator sink_arc = sink->m_in_arcs.begin(); sink_arc != sink->m_in_arcs.end(); ++sink_arc)
	{
	    flow_size += (*sink_arc)->m_flow;
	}
	
	return flow_size;
}



/*----------------------------------------------------------------------------*/

} // namespace sReloc
