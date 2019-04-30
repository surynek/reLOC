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
/* complete.h / 0.20-kruh_059                                                 */
/*----------------------------------------------------------------------------*/
//
// Fragments of complete polynomial time algorithms.
//
/*----------------------------------------------------------------------------*/


#ifndef __COMPLETE_H__
#define __COMPLETE_H__


#include "types.h"
#include "defs.h"
#include "result.h"
#include "search.h"


using namespace std;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


class sCompleteSolver
    : public sMultirobotSolver
{
public:
    struct Traversal
    {
	Traversal(int source_id, int target_id);

	int m_source_id;
	int m_target_id;
    };

    typedef std::set<int, std::less<int> > VertexIDs_set;
    typedef std::list<int> VertexIDs_list;
    typedef std::vector<int> VertexIDs_vector;    
    typedef std::vector<int> RobotIDs_vector;

    typedef std::vector<Traversal> Traversals_vector;

    struct Attempt
    {
	Attempt();

	Traversals_vector m_attempt_Traversals;
	sRobotArrangement m_saved_arrangement;
	Traversals_vector m_undo_Traversals;
    };

    typedef std::vector<Attempt> Attempts_vector;

public:
    sCompleteSolver();

private:
    sCompleteSolver(const sCompleteSolver &complete_solver);
    const sCompleteSolver& operator=(const sCompleteSolver &complete_solver);

public:
    virtual void setup_Solver(void);
    virtual void setup_Instance(const sMultirobotInstance &multirobot_instnace);

    bool solve(void);
    bool solve_Twin(void);

    bool relocate_Robot(int source_vertex_id, int goal_vertex_id, VertexIDs_set &locked_Vertices);
    bool relocate_TwinRobots(int first_source_id, int second_source_id, int goal_id, VertexIDs_set &locked_Vertices);

    bool push(int source_id, int target_id, VertexIDs_set &locked_Vertices);
    bool push_Twin(int first_source_id, int second_source_id, int target_id, VertexIDs_set &locked_Vertices);

    bool multipush(int source_1_id, int source_2_id, int target_id, int &pretarget_id);
    bool multipush_Twin(int source_1_id, int source_2_id, int source_3_id, int target_id, int &pretarget_id, int &prepretarget_id);

    bool clear(int pretarget_id, int vertex_id, int &first_vacant_id, int &second_vacant_id);
    bool clear_Twin(int pretarget_id, int prepretarget_id, int vertex_id, int &first_vacant_id, int &second_vacant_id, int &third_vacant_id);

    void exchange(int pretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id);
    void exchange_Twin_1(int pretarget_id, int prepretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id, int third_vacant_id);
    void exchange_Twin_2(int pretarget_id, int prepretarget_id, int vertex_id, int first_vacant_id, int second_vacant_id, int third_vacant_id);

    bool swap(int source_id, int target_id);
    bool swap_Twin(int first_source_id, int second_source_id, int target_id);

    bool clear_Vertex(int source_id, VertexIDs_set &locked_Vertices);

    void search_SingleSourcePaths(int source_id);
    void search_SingleSourcePaths(int source_id, VertexIDs_set &locked_Vertices);

    void search_BreadthFirst(const sRobotArrangement &robot_arrangement, RobotIDs_vector &sorted_Robots);
    void search_BreadthFirst(int source_id, VertexIDs_vector &sorted_Vertices);

    void undo(void);
    void enable_Undo(void);
    void disable_Undo(void);

    void commit(void);
    void commit(const Attempt &attempt);
    void enable_Attempt(void);
    void disable_Attempt(void);

    void make_Move(int source_id, int target_id);
    void make_Move_(int source_id, int target_id);
    void record_Move(int source_id, int target_id);

public:
    sRobotArrangement m_current_arrangement;

    int m_last_time_step;
    sMultirobotSolution m_multirobot_solution;

    Attempts_vector m_movement_Attempts;

    bool m_undo_enabled;
    Traversals_vector m_undo_Traversals;
};


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __COMPLETE_H__ */
