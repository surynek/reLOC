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
/* statistics.h / 0.20-kruh_055                                               */
/*----------------------------------------------------------------------------*/
//
// Statistical data collection and analytical tools.
//
/*----------------------------------------------------------------------------*/


#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <map>

#include "types.h"
#include "result.h"
#include "multirobot.h"


using namespace sReloc;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{

    
/*----------------------------------------------------------------------------*/
// sMultirobotSolutionAnalyzer

    class sMultirobotSolutionAnalyzer
    {
    public:
	typedef std::map<int, int, std::less<int> > Distribution_map;
	typedef std::map<int, int, std::less<int> > Trajectories_map;

    public:
	sMultirobotSolutionAnalyzer();
	sMultirobotSolutionAnalyzer(const sMultirobotSolutionAnalyzer &solution_analyzer);
	const sMultirobotSolutionAnalyzer& operator=(const sMultirobotSolutionAnalyzer &solution_analyzer);

	void analyze_Solution(const sMultirobotSolution &solution,
			      const sRobotArrangement   &initial_arrangement,
			      sUndirectedGraph          &environment);
	int calc_TotalCost(const sMultirobotSolution &solution,
			   const sRobotArrangement   &initial_arrangement,
			   sUndirectedGraph          &environment);
	int calc_TotalFuel(const sMultirobotSolution &solution,
			   const sRobotArrangement   &initial_arrangement,
			   sUndirectedGraph          &environment);	

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

	virtual void to_Screen_distribution(const Distribution_map &distribution, const sString &indent) const;
	virtual void to_Stream_distribution(FILE *fw, const Distribution_map &distribution, const sString &indent) const;

    private:
	int m_total_makespan;
	int m_total_distance;
	int m_total_trajectory;
	int m_total_cost;
	int m_total_fuel;	

	double m_average_parallelism;
	double m_average_distance;
	double m_average_trajectory;

	Distribution_map m_distribution_Parallelisms;
	Distribution_map m_distribution_Distances;
	Distribution_map m_distribution_Trajectories;
    };


/*----------------------------------------------------------------------------*/
// sPhaseStatistics

    class sPhaseStatistics
    {
    public:
	static const double SECONDS_UNDEFINED;
	static const sString ROOT_PHASE_NAME;

    public:
	struct Phase;
	typedef std::map<sString, Phase*, std::less<sString> > Phases_map;

	struct Phase
	{
	    Phase(const sString &name, Phase *parent_phase);

	    sString m_name;

	    double m_WC_Seconds;
	    double m_CPU_Seconds;

	    long m_total_sat_solver_Calls;
	    long m_SAT_sat_solver_Calls;
	    long m_UNSAT_sat_solver_Calls;
	    long m_INDET_sat_solver_Calls;

	    long m_move_Executions;

	    long m_produced_cnf_Variables;
	    long m_produced_cnf_Clauses;

	    long m_search_Steps;

	    Phase *m_parent_phase;
	    Phases_map m_sub_Phases;
	};

    private: /* the object cannot be copied */
	sPhaseStatistics(const sPhaseStatistics &phase_statistics);
	const sPhaseStatistics& operator=(const sPhaseStatistics &phase_statistics);

    public:
	sPhaseStatistics();
	virtual ~sPhaseStatistics();

	Phase& get_CurrentPhase(void);
	void enter_Root(void);
	void enter_Phase(const sString &phase_name);
	void leave_Phase(void);

	void restart_CurrentPhase(void);
	void suspend_CurrentPhase(void);

	static double get_WC_Seconds(void);
	static double get_CPU_Seconds(void);

    public:
	virtual void to_Screen(const sString &indent = "");
	virtual void to_Stream(FILE *fw, const sString &indent = "");

	virtual void to_Screen_subphases(const Phase &phase, const sString &indent = "");
	virtual void to_Stream_subphases(FILE *fw, const Phase &phase, const sString &indent = "");

    public:
	Phase *m_current_phase;

	double m_curr_phase_start_WC;
	double m_curr_phase_finish_WC;

	double m_curr_phase_start_CPU;
	double m_curr_phase_finish_CPU;

	Phase *m_root_phase;
    };


/*----------------------------------------------------------------------------*/
// Global objects

    extern sPhaseStatistics s_GlobalPhaseStatistics;


/*----------------------------------------------------------------------------*/
// Global functions

    double sGet_WC_Seconds(void);
    double sGet_CPU_Seconds(void);


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __STATISTICS_H__ */
