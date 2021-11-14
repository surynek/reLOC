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
/* statistics.h / 0.22-robik_100                                              */
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


using namespace sReloc;
using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{

   

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

	struct MicroPhase
	{
	    MicroPhase()
	    : m_key(-1)
	    , m_WC_Seconds(0.0)
	    , m_CPU_Seconds(0.0)
	    { /* nothing */ }
	    
	    MicroPhase(sInt_32 key)
	    : m_key(key)
	    , m_WC_Seconds(0.0)
	    , m_CPU_Seconds(0.0)	      
	    { /* nothing */ }

	    sInt_32 m_key;

	    double m_start_WC;
	    double m_finish_WC;

	    double m_start_CPU;
	    double m_finish_CPU;

	    double m_WC_Seconds;
	    double m_CPU_Seconds;
	};

	typedef std::map<sInt_32, MicroPhase, std::less<sInt_32> > MicroPhases_map;

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

	    int m_max_makespan_tested;
	    int m_max_total_cost_tested;

	    long m_search_Steps;

	    Phase *m_parent_phase;
	    Phases_map m_sub_Phases;

	    MicroPhases_map m_micro_Phases;	    
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

	void enter_MicroPhase(sInt_32 key);
	void leave_MicroPhase(void);

	void restart_CurrentPhase(void);
	void suspend_CurrentPhase(void);

	void restart_CurrentMicroPhase(void);
	void suspend_CurrentMicroPhase(void);	

	static double get_WC_Seconds(void);
	static double get_CPU_Seconds(void);

    public:
	virtual void to_Screen(const sString &indent = "");
	virtual void to_Stream(FILE *fw, const sString &indent = "");

	virtual void to_Screen_subphases(const Phase &phase, const sString &indent = "");
	virtual void to_Stream_subphases(FILE *fw, const Phase &phase, const sString &indent = "");

    public:
	Phase *m_current_phase;
	sInt_32 m_curr_micro_key;

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
