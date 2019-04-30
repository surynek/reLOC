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
<<<<<<< HEAD
/* graph.h / 0.20-kruh_055                                                    */
=======
/* graph.h / 0.20-kruh_056                                                    */
>>>>>>> f57c68398eae7b055f31a50698a3a79978214a2b
/*----------------------------------------------------------------------------*/
//
// Algorithmic graph theory.
//
/*----------------------------------------------------------------------------*/


#ifndef __GRAPH_H__
#define __GRAPH_H__


#include "types.h"
#include "defs.h"
#include "result.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// Forward declarations

    class sVertex_;
    class sArc_;
    class sDigraph;




/*============================================================================*/
// class sSerializable
/*----------------------------------------------------------------------------*/

    class sSerializable
    {
    public:
	static sString to_String_tag(const sString &begin_tag,
				     const sString &end_tag,
				     int           &int_content,
				     const sString &indent = "");

	static sString to_String_tag(const sString &begin_tag,
				     const sString &end_tag,
				     const sString &string_content,
				     const sString &indent = "");

	static sString to_String_tag(const sString       &begin_tag,
				     const sString       &end_tag,
				     const sSerializable &serial_content,
				     const sString       &indent = "");

	template<class TContainer>
	static sString to_String_container(const sString    &begin_tag,
					   const sString    &end_tag,
					   const TContainer &container_content,
					   const sString    &indent = "");

	template<class TContainer>
	static sString to_String_container(const sString    &begin_tag,
					   const sString    &end_tag,
					   const TContainer *ptrcntr_content,
					   const sString    &indent = "");

	template<class TAssociative>
	static sString to_String_associative(const sString      &begin_tag,
					     const sString      &end_tag,
					     const TAssociative &associative_content,
					     const sString      &indent = "");

	template<class TAssociative>
	static sString to_String_associative(const sString      &begin_tag,
					     const sString      &end_tag,
					     const TAssociative *ptrassc_content,
					     const sString      &indent = "");

	static sString to_String_attribute(const sString &attribute, int int_value);
	static sString to_String_attribute(const sString &attribute, double double_value);
	static sString to_String_attribute(const sString &attribute, const sString &string_value);

	/*----------------------------------------------------------------------------*/

	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual sString to_String(const sString &indent = "") const = 0;
};




/*============================================================================*/
// class sWeakComparable
/*----------------------------------------------------------------------------*/

    template<class TWeakComparable>
    class sWeakComparable
    {
    public:
	virtual bool operator==(const TWeakComparable &weak_cmpr) const;
	virtual bool operator!=(const TWeakComparable &weak_cmpr) const;
	virtual bool operator<(const TWeakComparable &weak_cmpr) const;
	virtual bool operator<=(const TWeakComparable &weak_cmpr) const;
	virtual bool operator>(const TWeakComparable &weak_cmpr) const;
	virtual bool operator>=(const TWeakComparable &weak_cmpr) const;

	/*----------------------------------------------------------------------------*/

	virtual int compare(const TWeakComparable &weak_cmpr) const;
	virtual int compare(const TWeakComparable &weak_cmpr_A, const TWeakComparable &weak_cmpr_B) const = 0;
};




/*============================================================================*/
// class sTotalComparable
/*----------------------------------------------------------------------------*/

    template<class TTotalComparable>
    class sTotalComparable
    : public sWeakComparable<TTotalComparable>
    {
    public:
	virtual int rank(void) const = 0;
	
	virtual int difference(const TTotalComparable &total_cmpr) const;
	virtual int difference(const TTotalComparable &total_cmpr_A, const TTotalComparable &total_cmpr_B) const = 0;
};




/*============================================================================*/
// class sVertexKey
/*----------------------------------------------------------------------------*/
 
    class sVertexKey
    : public sWeakComparable<sVertexKey>
    {
    public:
	sVertexKey(sDigraph *graph, const sString &identifier);
	/*----------------------------------------------------------------------------*/

	virtual int compare(const sVertexKey &vertex_key) const;
	virtual int compare(const sVertexKey &vertex_key_A, const sVertexKey &vertex_key_B) const;
	
    public:
	sDigraph *m_graph;
	sString m_identifier;
};




/*============================================================================*/
// class sVertex_
/*----------------------------------------------------------------------------*/

    class sVertex_
    : public sSerializable
    {
    public:
	typedef std::list<sArc_*> Arcs_list;
	typedef std::set<sArc_*, std::less<sArc_*> > Arcs_set;
	
	/*----------------------------------------------------------------------------*/
	
    public:
	sVertex_(sVertexKey *vertex_key);
	/*----------------------------------------------------------------------------*/

	virtual sString to_String(const sString &indent = "") const;

    public:
	sVertexKey *m_vertex_key;
	Arcs_list m_in_arcs;
	Arcs_list m_out_arcs;

	int m_excess;
	int m_height;
	
	Arcs_set m_out_farcs;
	Arcs_set m_out_garcs;
};




/*============================================================================*/
// class sArcKey
/*----------------------------------------------------------------------------*/

    class sArcKey
	: public sWeakComparable<sArcKey>
    {
    public:
	sArcKey(sDigraph *graph, sVertex_ *first_vertex, sVertex_ *second_vertex);
	/*----------------------------------------------------------------------------*/

	virtual int compare(const sArcKey &arc_key) const;
	virtual int compare(const sArcKey &arc_key_A, const sArcKey &arc_key_B) const;
	
    public:
	sDigraph *m_graph;
	sVertex_ *m_first_vertex;
	sVertex_ *m_second_vertex;
};




/*============================================================================*/
// class sArc_
/*----------------------------------------------------------------------------*/

    class sArc_
    : public sSerializable
    {
    public:
	sArc_(sArcKey *arc_key, int capacity = 0, int flow = 0);
	/*----------------------------------------------------------------------------*/

	virtual sString to_String(const sString &indent = "") const;

    public:
	const sArcKey *m_arc_key;
	
	int m_capacity;
	int m_flow;
    };




/*============================================================================*/
// class sDigraph
/*----------------------------------------------------------------------------*/

    class sDigraph
    : public sSerializable
    {
    public:
	typedef std::map<sVertexKey, sVertex_, std::less<sVertexKey> > Vertices_map;
	typedef std::map<sArcKey, sArc_, std::less<sArcKey> > Arcs_map;

	/*----------------------------------------------------------------------------*/

    public:
	sDigraph();
	/*----------------------------------------------------------------------------*/

	Vertices_map::iterator add_Vertex(const sString &identifier);
	Vertices_map::iterator add_Vertex(const sVertex_ &vertex);
	
	void remove_Vertex(const sString &identifier);
	void remove_Vertex(Vertices_map::iterator vertex);
	
	Arcs_map::iterator add_Arc(const sString &first_vertex_id, const sString &second_vertex_id, int capacity = 0, int flow = 0);
	Arcs_map::iterator add_Arc(const sArc_ &arc);
	
	void remove_Arc(const sString &first_vertex_id, const sString &second_vertex_id);
	void remove_Arc(Arcs_map::iterator arc);

	void clean_Graph(void);
	/*----------------------------------------------------------------------------*/

	virtual sString to_String(const sString &indent = "") const;

    public:
	Vertices_map m_vertices;
	Arcs_map m_arcs;
};




/*============================================================================*/
// class sGoldberg
/*----------------------------------------------------------------------------*/

    class sGoldberg
    {
    public:
	typedef std::list<sVertex_*> Vertices_list;
	typedef std::list<sArc_*> Arcs_list;       
	/*----------------------------------------------------------------------------*/

    public:
	sGoldberg();

    private:
	sGoldberg(const sGoldberg &goldberg);
	const sGoldberg& operator=(const sGoldberg &goldberg);

    public:
	void init(sDigraph *graph, sVertex_ *source, sVertex_ *sink, Vertices_list &relabel);
	void push_In(sArc_ *arc, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const;
	void push_Out(sArc_ *arc, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const;
	void relabel(sVertex_ *vertex, Vertices_list &rlb, Arcs_list &push_in, Arcs_list &push_out) const;

	int compute_Flow(sDigraph *graph, sVertex_ *source, sVertex_ *sink);
	/*----------------------------------------------------------------------------*/

    public:
	sVertex_ *m_source;
	sVertex_ *m_sink;
    };




/*============================================================================*/
// sSerializable class
/*----------------------------------------------------------------------------*/
// public methods of the sSerializable class
/*----------------------------------------------------------------------------*/
    
    template<class TContainer>
    sString sSerializable::to_String_container(const sString    &begin_tag,
					       const sString    &end_tag,
					       const TContainer &container_content,
					       const sString    &indent)
    {
	sString output;
	
	output += indent + begin_tag + "\n";
	
	if (!container_content.empty())
	{
	    class TContainer::const_iterator serializable = container_content.begin();
	    
	    while (true)
	    {
		output += serializable->to_String(indent + sRELOC_INDENT) + "\n";
		
		if (++serializable == container_content.end())
		{
		    break;
		}
	    }
	}
	output += indent + end_tag;
	
	return output;
    }
    
    
    template<class TContainer>
    sString sSerializable::to_String_container(const sString    &begin_tag,
					       const sString    &end_tag,
					       const TContainer *ptrcntr_content,
					       const sString    &indent)
    {
	sString output;
	
	output += indent + begin_tag + "\n";
	
	if (!ptrcntr_content->empty())
	{
	    class TContainer::const_iterator serializable = ptrcntr_content->begin();
	    while (true)
	    {
		output += (*serializable)->to_String(indent + sRELOC_INDENT) + "\n";
		
		if (++serializable == ptrcntr_content->end())
		{
		    break;
		}
	    }
	}
	output += indent + end_tag;
	
	return output;
    }
    
    
    template<class TAssociative>
    sString sSerializable::to_String_associative(const sString      &begin_tag,
						 const sString      &end_tag,
						 const TAssociative &associative_content,
						 const sString      &indent)
    {
	sString output;
	
	output += indent + begin_tag + "\n";
	
	if (!associative_content.empty())
	{
	    class TAssociative::const_iterator serializable = associative_content.begin();
	    
	    while (true)
	    {
		output += serializable->second.to_String(indent + sRELOC_INDENT) + "\n";
		
		if (++serializable == associative_content.end())
		{
		    break;
		}
	    }
	}
	output += indent + end_tag;
	
	return output;
    }
    
    
    template<class TAssociative>
    sString sSerializable::to_String_associative(const sString      &begin_tag,
						 const sString      &end_tag,
						 const TAssociative *ptrassc_content,
						 const sString      &indent)
    {
	sString output;
	
	output += indent + begin_tag + "\n";
	
	if (!ptrassc_content->empty())
	{
	    class TAssociative::const_iterator serializable = ptrassc_content->begin();
	    
	    while (true)
	    {
		output += serializable->second->toString(indent + sRELOC_INDENT) + "\n";
		
		if (++serializable == ptrassc_content->end())
		{
		    break;
		}
	    }
	}
	output += indent + end_tag;
	
	return output;
    }
    
    

    
/*============================================================================*/
// sWeakComparable class
/*----------------------------------------------------------------------------*/
// public methods of the sWeakComparable class
/*----------------------------------------------------------------------------*/

    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator==(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) == 0);
    }


    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator!=(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) != 0);
    }
    
    
    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator<(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) < 0);
    }
    
    
    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator<=(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) <= 0);
    }
    
    
    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator>(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) > 0);
    }
    
    
    template<class TWeakComparable>
    bool sWeakComparable<TWeakComparable>::operator>=(const TWeakComparable &weak_cmpr) const
    {
	return (compare(weak_cmpr) >= 0);
    }
    
    
    template<class TWeakComparable>
    int sWeakComparable<TWeakComparable>::compare(const TWeakComparable &weak_cmpr) const
    {
	return (compare(static_cast<const TWeakComparable&>(*this), weak_cmpr));
    }




/*============================================================================*/
// sTotalComparable class
/*----------------------------------------------------------------------------*/
// public methods of the sTotalComparable class
/*----------------------------------------------------------------------------*/

    template<class TTotalComparable>
    int sTotalComparable<TTotalComparable>::difference(const TTotalComparable &total_cmpr) const
    {
	return (difference(static_cast<const TTotalComparable&>(*this), total_cmpr));
    }




/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __GRAPH_H__ */
