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
/* reloc.h / 0.20-kruh_055                                                    */
/*----------------------------------------------------------------------------*/
//
// Relocation problem solving package - original development header.
//
/*----------------------------------------------------------------------------*/


#ifndef __RELOC_H__
#define __RELOC_H__

#include <vector>
#include <list>
#include <set>
#include <map>

#include "types.h"
#include "result.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{

    class sEdge;
    class sArc;
    class sVertex;


/*----------------------------------------------------------------------------*/
// Global constants

    extern const sString sRELOC_INDENT;
    extern const sString sRELOC_1_INDENT;
    extern const sString sRELOC_2_INDENT;
    extern const sString sRELOC_3_INDENT;
    extern const sString sRELOC_4_INDENT;

    extern const sString sRELOC_SAT_SOLVER_PATH;

    
/*----------------------------------------------------------------------------*/
// sVertex

    class sVertex
    {
    public:
	static const int ORDER_UNDEFINED = -1;

    public:
	typedef std::list<sArc*> Neighbors_list;
	typedef std::vector<sArc*> Neighbors_vector;
	typedef std::vector<int> VertexIDs_vector;

    public:
	sVertex();
	sVertex(int id);
	sVertex(const sVertex &vertex);
	const sVertex& operator=(const sVertex &vertex);

	int calc_NeighborCount(void) const;
	int calc_NeighborOrder(int vertex_id) const;
	int calc_NeighborID(int order) const;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	int m_id;
	Neighbors_list m_Neighbors;
	Neighbors_list m_in_Neighbors;
	Neighbors_list m_out_Neighbors;

	bool m_visited;
	int m_distance;
	int m_prev_id;

	VertexIDs_vector m_Conflicts;
    };


/*----------------------------------------------------------------------------*/
// sArc

    class sArc
    {
    public:
	sArc();
	sArc(sEdge *edge, sVertex *source, sVertex *target);
	sArc(const sArc &arc);
	const sArc& operator=(const sArc &arc);

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	sEdge *m_edge;

	sVertex *m_source;
	sVertex *m_target;
    };


/*----------------------------------------------------------------------------*/
// sEdge

    class sEdge
    {
    public:
	sEdge(int id, sVertex *vertex_u, sVertex *vertex_v, bool directed = false);
	sEdge(const sEdge &edge);
	const sEdge& operator=(const sEdge &edge);

	struct Conflict
	{
	    Conflict(int x_ID, int y_ID)
	    : m_x_ID(x_ID), m_y_ID(y_ID) { }
	    
	    int m_x_ID;
	    int m_y_ID;
	};

	typedef std::vector<Conflict> Conflicts_vector;

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	int m_id;
	bool m_directed;

	sArc m_arc_uv;
	sArc m_arc_vu;

	Conflicts_vector m_Conflicts;
    };


/*----------------------------------------------------------------------------*/
// sUndirectedGraph

    class sUndirectedGraph
    {
    public:
	struct Edge
	{
	    Edge(int u_id, int v_id)
	    {
		m_u_id = u_id;
		m_v_id = v_id;
	    }
	    
	    int m_u_id;
	    int m_v_id;
	};

	struct Coordinates
	{
	    Coordinates() { }
	    Coordinates(int row, int column)
	    {
		m_row = row;
		m_column = column;
	    }
	    
	    int m_row;
	    int m_column;
	};

	typedef std::vector<Edge> Edges_vector;
	typedef std::vector<Coordinates> Coordinates_vector;

	typedef std::vector<int> Distances_vector; 
	typedef std::vector<Distances_vector> Distances_2d_vector;
	typedef std::vector<Distances_2d_vector> Distances_3d_vector;
	typedef std::vector<Distances_3d_vector> Distances_4d_vector;

	typedef std::vector<int> VertexIDs_vector;
	typedef std::list<int> VertexIDs_list;
	typedef std::set<int> VertexIDs_set;

	typedef std::vector<sVertex> Vertices_vector;
	typedef std::list<sVertex*> Vertices_list;	

	typedef std::list<sEdge> Edges_list;
	typedef std::multimap<int, int, std::less<int> > VertexQueue_multimap;

	typedef std::pair<int, int> Vertex_pair;
	typedef std::vector<Vertex_pair> VertexPairs_vector;
	typedef std::multimap<int, Vertex_pair, std::less<int> > VertexPairQueue_multimap;

    public:
	sUndirectedGraph();
	sUndirectedGraph(bool directed);
	sUndirectedGraph(int x_size, int y_size);
	sUndirectedGraph(int x_size, int y_size, double obstacle_prob);
	sUndirectedGraph(int x_size, int y_size, int N_obstacles);
	sUndirectedGraph(int base_cycle_size, int ear_min_size, int ear_max_size, int graph_size);

	sUndirectedGraph(bool directed, int x_size, int y_size);
	sUndirectedGraph(bool directed, int x_size, int y_size, double obstacle_prob);
	sUndirectedGraph(bool directed, int x_size, int y_size, int N_obstacles);
	sUndirectedGraph(bool directed, int base_cycle_size, int ear_min_size, int ear_max_size, int graph_size);
	
	sUndirectedGraph(const sUndirectedGraph &undirected_graph);
	sUndirectedGraph(const sUndirectedGraph &undirected_graph, bool opposite);
			 
	const sUndirectedGraph& operator=(const sUndirectedGraph &undirected_graph);
	~sUndirectedGraph();
	/*----------------------------------------------------------------------------*/

	void initialize_InverseMatrix(void);
	
	void generate_Network(int x_size, int y_size, int aisle_length, double obstacle_prob);
	void generate_Network(int x_size, int y_size, int aisle_length, int N_obstacles);
	void generate_Network(int x_size, int y_size, int min_aisle_length, int max_aisle_length, double obstacle_prob);
	void generate_Network(int x_size, int y_size, int min_aisle_length, int max_aisle_length, int N_obstacles);
	void generate_Network(int *Matrix, int size, int x_size, int y_size, int min_aisle_length, int max_aisle_length);

	void generate_Hypercube(int dimmension, int aisle_length);
	void generate_Hypercube(int dimmension, int min_aisle_length, int max_aisle_length);

	void add_Aisle(int u_id, int v_id, int min_aisle_length, int max_aisle_length, Edges_vector &Edges);
	void explicate_Conflicts(int range);

	void add_Vertex(void);
	void add_Vertices(int Vertex_cnt = 1);

	int get_VertexCount(void) const;
	sVertex* get_Vertex(int id);
	const sVertex* get_Vertex(int id) const;

	bool is_Grid(void) const;
	int get_GridHeight(void) const;
	int get_GridWidth(void) const;
	int get_GridCell(int x, int y) const;

	int calc_GridRow(int vertex_id) const;
	int calc_GridColumn(int vertex_id) const;
	int calc_GridVertexID(int grid_row, int grid_column) const;
	int calc_GridNeighborVertexID(int vertex_id, int delta_row, int delta_column) const;
	int get_GridNeighborVertexID(int vertex_id, int delta_row, int delta_column) const;	

	void add_Arrow(int u_id, int v_id);
	void add_Edge(int u_id, int v_id);
	void add_RandomArrow(int u_id, int v_id);
	int get_EdgeCount(void) const;

	bool is_Adjacent(int u_id, int v_id) const;
	bool is_LinkedTo(int u_id, int v_id) const;

	int calc_ShortestPath(int u_id, int v_id) const;
	void calc_SingleSourceShortestPaths(int s_id, Distances_vector &Distances) const;
	
	void calc_SingleSourceShortestPathsBreadth(int s_id);
	void calc_SingleSourceShortestPathsBreadth(int s_id, Distances_vector &Distances) const;

	void calc_SingleSourceShortestPathsBreadth_opposite(int s_id);	
	void calc_SingleSourceShortestPathsBreadth_opposite(int s_id, Distances_vector &Distances) const;
	
	void find_ShortestPathBreadth(int source_id, int dest_id, VertexIDs_vector &shortest_Path);

	void calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances);
	void calc_AllPairsShortestPaths(Distances_2d_vector &all_pairs_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void calc_SourceGoalShortestPaths(Distances_2d_vector &source_Distances, Distances_2d_vector &goal_Distances, const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void collect_EquidistantVertices(int s_id, int distance, VertexIDs_vector &equidistant_IDs);

	void calc_AllPairsShortestPaths(void);
	void calc_AllPairsShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	void calc_SourceGoalShortestPaths(const VertexIDs_vector &source_IDs, const VertexIDs_vector &goal_IDs);
	const Distances_2d_vector& get_AllPairsShortestPaths(void) const;

	const Distances_2d_vector& get_SourceShortestPaths(void) const;
	const Distances_2d_vector& get_GoalShortestPaths(void) const;

	int calc_ShortestCoopPath(int u1_id, int u2_id, int v1_id, int v2_id) const;
	void calc_SingleSourceShortestCoopPaths(int s1_id, int s2_id, Distances_2d_vector &Distances) const;
	void calc_AllPairsShortestCoopPaths(Distances_4d_vector &all_pairs_coop_Distances) const;
	void calc_AllPairsShortestCoopPaths(void);
	const Distances_4d_vector& get_AllPairsShortestCoopPaths(void) const;

	void find_ShortestPath(int u_id, int v_id, VertexIDs_list &path);
	void find_ShortestPathBFS(int u_id, int v_id);
	/*----------------------------------------------------------------------------*/
       
	void build_SpanningTree(int root_id, sUndirectedGraph &spanning_tree);
	void extend_SpanningTree(int root_id, sUndirectedGraph &spanning_tree);
	
	void build_SpanningTreeDFS(int root_id, sUndirectedGraph &spanning_tree);
	/*----------------------------------------------------------------------------*/

	void build_SparseGraph(const VertexPairs_vector &vertex_Pairs, sUndirectedGraph &sparse_graph);
	void extend_SparseGraph(const VertexPairs_vector &vertex_Pairs, sUndirectedGraph &sparse_graph);
	/*----------------------------------------------------------------------------*/

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Screen_vertices(const sString &indent = "") const;

	virtual void to_Stream(FILE *fw, const sString &indent = "") const;
	virtual void to_Stream_vertices(FILE *fw, const sString &indent = "") const;

	virtual sResult to_File_multirobot(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_multirobot(FILE *fw, const sString &indent = "") const;

	virtual sResult from_File_multirobot(const sString &filename);
	virtual sResult from_Stream_multirobot(FILE *fr);

	virtual sResult from_File_map(const sString &filename);
	virtual sResult from_Stream_map(FILE *fr);

	virtual sResult from_File_bgu(const sString &filename);
	virtual sResult from_Stream_bgu(FILE *fr);

	virtual sResult from_File_usc(const sString &filename);
	virtual sResult from_Stream_usc(FILE *fr);

	virtual sResult from_File_lusc(const sString &filename);
	virtual sResult from_Stream_lusc(FILE *fr);	

	virtual sResult to_File_dibox(const sString &filename, const sString &indent = "") const;
	virtual void to_Stream_dibox(FILE *fw, const sString &indent = "") const;	

	virtual sResult from_File_dibox(const sString &filename);
	virtual sResult from_Stream_dibox(FILE *fr);	
		
    public:
	bool m_directed;
	
	int m_Edge_cnt;
	int m_x_size;
	int m_y_size;
	
	int *m_Matrix;
	Coordinates_vector m_inverse_Matrix;

	Vertices_vector m_Vertices;
	Edges_list m_Edges;

	bool m_all_pairs_distances_calculated;
	Distances_2d_vector m_all_pairs_Distances;

	bool m_source_goal_distances_calculated;
	Distances_2d_vector m_source_Distances;
	Distances_2d_vector m_goal_Distances;

	bool m_all_pairs_coop_distances_calculated;
	Distances_4d_vector m_all_pairs_coop_Distances;

	VertexIDs_vector m_Queue;
	Distances_vector m_Distances;
    }; 


/*----------------------------------------------------------------------------*/
// sVectorGraph

    class sVectorGraph
    {
    public:
	typedef std::vector<int> Connections_vector;
	typedef std::vector<Connections_vector> Connections_2d_vector;

    public:
	sVectorGraph();
	sVectorGraph(const sUndirectedGraph &undirected_graph);
	~sVectorGraph();

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:	
	Connections_2d_vector m_Connections;
    };


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __RELOC_H__ */
