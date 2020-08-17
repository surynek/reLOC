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
/* search.h / 0.21-robik_041                                                  */
/*----------------------------------------------------------------------------*/
//
// Search-based solving of multi-robot path-finding problem.
//
/*----------------------------------------------------------------------------*/


#ifndef __SEARCH_H__
#define __SEARCH_H__

#include <vector>
#include <list>
#include <set>
#include <map>

#include "types.h"
#include "result.h"
#include "reloc.h"
#include "cnf.h"
#include "graph.h"
#include "reloc.h"
#include "multirobot.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{



/*----------------------------------------------------------------------------*/
// sMultirobotSolver
    
    class sMultirobotSolver
    {
    public:
	sMultirobotSolver();

    private:
	sMultirobotSolver(const sMultirobotSolver &multirobot_solver);
	const sMultirobotSolver& operator=(const sMultirobotSolver &multirobot_solver);

    public:
	virtual void setup_Solver(void);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	//virtual bool solve_Instance(sMultirobotSolution &multirobot_solution) = 0;

    public:
	sMultirobotInstance m_multirobot_instance;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_ID

    class sMultirobotSolver_ID
    : public sMultirobotSolver
    {
    public:
	sMultirobotSolver_ID();

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual void setup_Solver(int max_depth_limit);

	virtual bool solve_Instance(sMultirobotSolution &multirobot_solution);

	bool search_Goal(int current_depth, int depth_limit, sMultirobotSolution &multirobot_solution);
	void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id);

    public:
	int m_max_depth_limit;
	int m_finished_Robot_cnt;
	sRobotArrangement m_current_arrangement;
    };


/*----------------------------------------------------------------------------*/
// sAstarHeuristic

    class sAstarHeuristic
    {
    public:
	virtual void setup_Instance(const sMultirobotInstance &multirobot_instance) = 0;
	virtual int calc_Heuristic(int source_vertex_id, int destination_vertex_id) = 0;
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement) = 0;
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal) = 0;
    };


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Trivial

    class sAstarHeuristic_Trivial
    : public sAstarHeuristic
    {
    public:
	sAstarHeuristic_Trivial();

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instance);
	virtual int calc_Heuristic(int source_vertex_id, int destination_vertex_id);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal);
    };


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Placement

    class sAstarHeuristic_Placement
    : public sAstarHeuristic
    {
    public:
	sAstarHeuristic_Placement();

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instance);
	virtual int calc_Heuristic(int source_vertex_id, int destination_vertex_id);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal);
    };


/*----------------------------------------------------------------------------*/
// sAstarHeuristic_Distance

    class sAstarHeuristic_Distance
    : public sAstarHeuristic
    {
    public:
	typedef std::vector<int> MatrixRow_vector;
	typedef std::vector<MatrixRow_vector> Matrix_vector;

    public:
	sAstarHeuristic_Distance();

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instance);
	virtual int calc_Heuristic(int source_vertex_id, int destination_vertex_id);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotArrangement &goal_arrangement);
	virtual int calc_Heuristic(const sRobotArrangement &current_arrangement, const sRobotGoal &robot_goal);

	void to_String_matrix(const sString &indent = "") const;

    public:
	Matrix_vector m_distance_Matrix;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_Astar

    class sMultirobotSolver_Astar
    : public sMultirobotSolver
    {
    public:
	struct ProductionMove
	{
	    ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move);

	    ProductionMove *m_prev;
	    sMultirobotSolution::Move m_move;
	};

	static const ProductionMove UNDEFINED_PRODUCTION_MOVE;

	struct StateRecord
	{
	    StateRecord(const sRobotArrangement &robot_arrangement);
	    StateRecord(int cost, ProductionMove *production_move, const sRobotArrangement &robot_arrangement);

	    bool operator==(const StateRecord &state_record) const;
	    bool operator<(const StateRecord &state_record) const;

	    int m_cost;
	    ProductionMove *m_production_move;
	    sRobotArrangement m_robot_arrangement;
	};

	typedef struct std::list<ProductionMove> ProductionMoves_list;

	typedef struct std::set<StateRecord, std::less<StateRecord> > StateRecords_set;
	typedef struct std::multimap<int, StateRecords_set::iterator, std::less<int> > StateRecords_multimap;

    public:
	sMultirobotSolver_Astar();
    
	virtual void setup_Solver(sAstarHeuristic *heuristic);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual bool solve_Instance(sMultirobotSolution &multirobot_solution);

	bool expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution);
	int collect_Solution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution);
	void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, const sRobotArrangement &current_arrangement, sRobotArrangement &next_arrangement);

    public:
	sAstarHeuristic *m_heuristic;

	ProductionMoves_list m_production_Moves;

	StateRecords_multimap m_expansion_Queue;
	StateRecords_set m_open_state_Recs;
	StateRecords_set m_close_state_Recs;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_HCAstar

    class sMultirobotSolver_HCAstar
    : public sMultirobotSolver
    {
    public:
	static const int WINDOW_SIZE_UNDEFINED = -1;

    public:
	struct ProductionMove
	{
	    ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move);

	    ProductionMove *m_prev;
	    sMultirobotSolution::Move m_move;
	};

	static const ProductionMove UNDEFINED_PRODUCTION_MOVE;

	struct StateRecord
	{
	    StateRecord(int location_id, int time_step);

	    bool operator==(const StateRecord &state_record) const;
	    bool operator<(const StateRecord &state_record) const;

	    int m_location_id;
	    int m_time_step;

	    int m_cost;
	    ProductionMove *m_production_move;
	};

	typedef std::list<ProductionMove> ProductionMoves_list;

	typedef std::set<StateRecord, std::less<StateRecord> > StateRecords_set;
	typedef std::multimap<int, StateRecords_set::iterator, std::less<int> > StateRecords_multimap;

	struct ReservationRecord
	{
	    ReservationRecord(int vertex_id, int time_step);

	    bool operator==(const ReservationRecord &state_record) const;
	    bool operator<(const ReservationRecord &state_record) const;

	    int m_vertex_id;
	    int m_time_step;
	};
	
	typedef struct std::set<ReservationRecord, std::less<ReservationRecord> > ReservationTable_set;

    public:
	sMultirobotSolver_HCAstar();
    
	virtual void setup_Solver(sAstarHeuristic *heuristic);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual bool solve_Instance(sMultirobotSolution &multirobot_solution);

	virtual bool solve_Robot(int robot_id, sMultirobotSolution &multirobot_solution);

	bool expand_State(int robot_id, const StateRecord &current_state_rec, ReservationTable_set &reservation_Table, sMultirobotSolution &multirobot_solution);
	int collect_Subsolution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, ReservationTable_set &reservation_Table);
	//void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, const sRobotArrangement &current_arrangement, sRobotArrangement &next_arrangement);

	virtual void to_Screen(const ReservationTable_set &reservation_Table, const sString &indent = "") const;

    public:
	int m_window_size;
	sAstarHeuristic *m_heuristic;

	StateRecords_multimap m_expansion_Queue;
	StateRecords_set m_open_state_Recs;
	StateRecords_set m_close_state_Recs;

	ReservationTable_set m_reservation_Table;
	ProductionMoves_list m_production_Moves;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_WHCAstar

    class sMultirobotSolver_WHCAstar
    : public sMultirobotSolver
    {
    public:
	static const int WINDOW_SIZE_UNDEFINED = -1;

    public:
	struct ProductionMove
	{
	    ProductionMove(ProductionMove *prev, sMultirobotSolution::Move move);

	    ProductionMove *m_prev;
	    sMultirobotSolution::Move m_move;
	};

	static const ProductionMove UNDEFINED_PRODUCTION_MOVE;

	struct StateRecord
	{
	    StateRecord(int location_id, int time_step);

	    bool operator==(const StateRecord &state_record) const;
	    bool operator<(const StateRecord &state_record) const;

	    int m_location_id;
	    int m_time_step;

	    int m_cost;
	    ProductionMove *m_production_move;
	};

	typedef std::list<ProductionMove> ProductionMoves_list;

	typedef std::set<StateRecord, std::less<StateRecord> > StateRecords_set;
	typedef std::multimap<int, StateRecords_set::iterator, std::less<int> > StateRecords_multimap;

	struct ReservationRecord
	{
	    ReservationRecord(int vertex_id, int time_step);

	    bool operator==(const ReservationRecord &state_record) const;
	    bool operator<(const ReservationRecord &state_record) const;

	    int m_vertex_id;
	    int m_time_step;
	};
	
	typedef std::set<ReservationRecord, std::less<ReservationRecord> > ReservationTable_set;
	typedef std::multimap<int, int, std::less<int> > RobotPriority_multimap;

    public:
	sMultirobotSolver_WHCAstar();
    
	virtual void setup_Solver(sAstarHeuristic *heuristic,
				  int              window_size = WINDOW_SIZE_UNDEFINED,
				  int              max_N_Iterations = sDEFAULT_N_WHCA_ITERATIONS);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual bool solve_Instance(sMultirobotSolution &multirobot_solution);
	virtual bool solve_Window(sMultirobotSolution &multirobot_solution);

	virtual bool solve_Robot(int robot_id, sMultirobotSolution &multirobot_solution);

	bool expand_State(int robot_id, const StateRecord &current_state_rec, const ReservationTable_set &reservation_Table);
	int collect_Subsolution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, ReservationTable_set &reservation_Table);

	virtual void to_Screen(const ReservationTable_set &reservation_Table, const sString &indent = "") const;

    public:
	int m_window_size;
	int m_max_N_Iterations;
	sAstarHeuristic *m_heuristic;

	StateRecords_multimap m_expansion_Queue;
	StateRecords_set m_open_state_Recs;
	StateRecords_set m_close_state_Recs;

	ReservationTable_set m_reservation_Table;
	ProductionMoves_list m_production_Moves;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_IDAstar
    
    class sMultirobotSolver_IDAstar
    : public sMultirobotSolver
    {
    public:
	sMultirobotSolver_IDAstar();

	virtual void setup_Solver(int max_cost_limit, sAstarHeuristic *heuristic);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual bool solve_Instance(sMultirobotSolution &multirobot_solution);

	bool search_Goal(int current_depth, int cost_limit, int &next_cost_limit, sMultirobotSolution &multirobot_solution);
	void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id);

    public:
	sAstarHeuristic *m_heuristic;

	int m_max_cost_limit;
	int m_finished_Robot_cnt;
	sRobotArrangement m_current_arrangement;
    };


/*----------------------------------------------------------------------------*/
// sMultirobotSolver_DecomposedAstar

    class sMultirobotSolver_DecomposedAstar
    : public sMultirobotSolver
    {
    public:
	struct ProductionMove
	{
	    ProductionMove(int depth, ProductionMove *prev, sMultirobotSolution::Move move);

	    int m_depth;
	    ProductionMove *m_prev;
	    sMultirobotSolution::Move m_move;
	};

	static const ProductionMove UNDEFINED_PRODUCTION_MOVE;

	typedef std::set<int> Vertices_set;

	struct StateRecord
	{
	    StateRecord(const sRobotArrangement &robot_arrangement);
	    StateRecord(int next_robot_id, const sRobotArrangement &robot_arrangement, const Vertices_set &compromised_Vertices);
	    StateRecord(int cost, int depth, int next_robot_id, ProductionMove *production_move, const sRobotArrangement &robot_arrangement);
	    StateRecord(int cost, int depth, int next_robot_id, ProductionMove *production_move, const sRobotArrangement &robot_arrangement, const Vertices_set &compromised_Vertices);

	    bool operator==(const StateRecord &state_record) const;
	    bool operator<(const StateRecord &state_record) const;

	    int m_depth;
	    int m_cost;
	    int m_next_robot_id;

	    ProductionMove *m_production_move;
	    sRobotArrangement m_robot_arrangement;
	    Vertices_set m_compromised_Vertices;
	};

	typedef std::list<ProductionMove> ProductionMoves_list;

	typedef std::set<StateRecord, std::less<StateRecord> > StateRecords_set;
	typedef std::multimap<int, StateRecords_set::iterator, std::less<int> > StateRecords_multimap;

	typedef std::vector<std::vector<int> > Occupation_2d_vector;

	typedef std::set<int> RobotGroup_set;
	typedef std::vector<RobotGroup_set> Grouping_vector;
	typedef std::vector<int> Robots_vector;

	typedef std::vector<sMultirobotInstance> Instances_vector;
	typedef std::vector<sMultirobotSolution> Solutions_vector;

	enum ResolutionResult
	{
	    RESOLUTION_UNDEFINED,
	    RESOLUTION_UNSOLVABLE,
	    RESOLUTION_MERGED,
	    RESOLUTION_INDETERMINATE,
	    RESOLUTION_NONCOLLIDING
	};

	static const int TOTAL_TIMEOUT_UNDEFINED = -1;

	enum SolutionResult
	{
	    RESULT_SOLVABLE,
	    RESULT_UNSOLVABLE,
	    RESULT_INDETERMINATE
	};

    public:
	sMultirobotSolver_DecomposedAstar(int total_timeout = TOTAL_TIMEOUT_UNDEFINED);
    
	virtual void setup_Solver(sAstarHeuristic *heuristic);

	virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);
	virtual SolutionResult solve_Instance(sMultirobotSolution &multirobot_solution);

	virtual SolutionResult solve_DecomposedInstance(sMultirobotSolution &multirobot_solution);
	virtual SolutionResult solve_DecomposedInstance(Grouping_vector &robot_Grouping, sMultirobotSolution &multirobot_solution);

	virtual void setup_Subinstance(const sMultirobotInstance &multirobot_instnace);
	virtual SolutionResult solve_Subinstance(sMultirobotSolution &multirobot_solution);

	bool expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution);

	virtual SolutionResult solve_Subinstance(sMultirobotSolution &multirobot_solution, const Occupation_2d_vector &Occupation);
	bool expand_Arrangement(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution, const Occupation_2d_vector &Occupation);

	void collect_Solution(const StateRecord &current_state_rec, sMultirobotSolution &multirobot_solution);

	void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, sRobotArrangement &next_arrangement) const;
	void conduct_Move(int robot_id, int src_vrtx_id, int dest_vrtx_id, const sRobotArrangement &current_arrangement, sRobotArrangement &next_arrangement) const;

	void reset_Occupation(int max_depth, int N_Vertices, Occupation_2d_vector &Occupation) const;
	void fill_Occupation(int max_depth, const sRobotArrangement &initial_arrangement, const sMultirobotSolution &multirobot_solution, Occupation_2d_vector &Occupation) const;

	ResolutionResult resolve_GroupCollisions(Grouping_vector &robot_Grouping, Instances_vector &group_Instances, Solutions_vector &group_Solutions);

	bool are_GroupsColliding(int group_index_A, int group_index_B, const Instances_vector &group_Instances, Solutions_vector &group_Solutions) const;
	SolutionResult resolve_GroupCollision(int group_index_A, int group_index_B, const Grouping_vector &robot_Grouping, const Instances_vector &group_Instances, Solutions_vector &group_Solutions);

	void combine_Subsolutions(const Grouping_vector &robot_Grouping, const Solutions_vector &group_Solutions, sMultirobotSolution &multirobot_solution);
	void prepare_Subinstance(const RobotGroup_set robot_group, sMultirobotInstance &group_subinstance) const;

    public:
	int m_total_timeout;
	double m_start_seconds;
	double m_finish_seconds;

	sAstarHeuristic *m_heuristic;

	ProductionMoves_list m_production_Moves;

	StateRecords_multimap m_expansion_Queue;
	StateRecords_set m_open_state_Recs;
	StateRecords_set m_close_state_Recs;

	sMultirobotInstance m_multirobot_subinstance;
    };


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __SEARCH_H__ */
