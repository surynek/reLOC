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
/* statistics.cpp / 0.22-robik_096                                            */
/*----------------------------------------------------------------------------*/
//
// Statistical data collection and analytical tools.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/times.h>

#include "config.h"
#include "compile.h"
#include "statistics.h"
#include "compress.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sPhaseStatistics

    const double sPhaseStatistics::SECONDS_UNDEFINED = 1.0;
    const sString sPhaseStatistics::ROOT_PHASE_NAME = "root_phase";


/*----------------------------------------------------------------------------*/

    sPhaseStatistics::Phase::Phase(const sString &name, Phase *parent_phase)
	: m_name(name)
	, m_WC_Seconds(0.0)
	, m_CPU_Seconds(0.0)
	, m_total_sat_solver_Calls(0)
	, m_SAT_sat_solver_Calls(0)
	, m_UNSAT_sat_solver_Calls(0)
	, m_INDET_sat_solver_Calls(0)
	, m_move_Executions(0)
	, m_produced_cnf_Variables(0)
	, m_produced_cnf_Clauses(0)
	, m_max_makespan_tested(0)
	, m_max_total_cost_tested(0)
	, m_search_Steps(0)
	, m_parent_phase(parent_phase)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sPhaseStatistics::sPhaseStatistics()
    {
	m_root_phase = new Phase(ROOT_PHASE_NAME, NULL);
	m_current_phase = m_root_phase;
	restart_CurrentPhase();
    }


    sPhaseStatistics::~sPhaseStatistics()
    {
	for (Phases_map::iterator phase = m_root_phase->m_sub_Phases.begin(); phase != m_root_phase->m_sub_Phases.end(); ++phase)
	{
	    delete phase->second;
	}
	delete m_root_phase;
    }


    sPhaseStatistics::Phase& sPhaseStatistics::get_CurrentPhase(void)
    {
	return *m_current_phase;
    }


    void sPhaseStatistics::enter_Root(void)
    {
	while (m_current_phase->m_parent_phase != NULL)
	{
	    leave_Phase();
	}
    }


    void sPhaseStatistics::enter_Phase(const sString &phase_name)
    {
	if (phase_name != m_current_phase->m_name)
	{
	    suspend_CurrentPhase();

	    Phases_map::iterator phase = m_current_phase->m_sub_Phases.find(phase_name);
	    if (phase != m_current_phase->m_sub_Phases.end())
	    {
		m_current_phase = phase->second;
	    }
	    else
	    {
		Phase *created_phase = new Phase(phase_name, m_current_phase);
		std::pair<Phases_map::iterator, bool> phase = m_current_phase->m_sub_Phases.insert(Phases_map::value_type(phase_name, created_phase));
		m_current_phase = phase.first->second;
	    }

	    restart_CurrentPhase();
	}
    }


    void sPhaseStatistics::leave_Phase(void)
    {
	suspend_CurrentPhase();
	    
	if (m_current_phase->m_parent_phase != NULL)
	{
	    m_current_phase = m_current_phase->m_parent_phase;
	}

	restart_CurrentPhase();
    }


    void sPhaseStatistics::enter_MicroPhase(sInt_32 key)
    {
	m_curr_micro_key = key;
	restart_CurrentMicroPhase();
    }    


    void sPhaseStatistics::leave_MicroPhase(void)
    {
	suspend_CurrentMicroPhase();
    }    

    
    double sPhaseStatistics::get_WC_Seconds(void)
    {
	struct timeval timeval;
	struct timezone timezone;

	if (gettimeofday(&timeval, &timezone) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return timeval.tv_sec + timeval.tv_usec / static_cast<double>(1000000.0);
	}
    }


    void sPhaseStatistics::restart_CurrentPhase(void)
    {
	m_curr_phase_start_WC = get_WC_Seconds();
	m_curr_phase_start_CPU = get_CPU_Seconds();
    }


    void sPhaseStatistics::suspend_CurrentPhase(void)
    {
	m_curr_phase_finish_WC = get_WC_Seconds();
	m_curr_phase_finish_CPU = get_CPU_Seconds();

	m_current_phase->m_WC_Seconds += m_curr_phase_finish_WC - m_curr_phase_start_WC;
	m_current_phase->m_CPU_Seconds += m_curr_phase_finish_CPU - m_curr_phase_start_CPU;
    }


    void sPhaseStatistics::restart_CurrentMicroPhase(void)
    {
	sASSERT(m_curr_micro_key != -1);
	
	MicroPhase &micro_phase = m_current_phase->m_micro_Phases[m_curr_micro_key];
	micro_phase.m_key = m_curr_micro_key;
	
	micro_phase.m_start_WC = get_WC_Seconds();
	micro_phase.m_start_CPU = get_CPU_Seconds();	
    }    

    
    void sPhaseStatistics::suspend_CurrentMicroPhase(void)
    {
	sASSERT(m_curr_micro_key != -1);
	MicroPhase &micro_phase = m_current_phase->m_micro_Phases[m_curr_micro_key];
	
	micro_phase.m_finish_WC = get_WC_Seconds();
	micro_phase.m_finish_CPU = get_CPU_Seconds();

	micro_phase.m_WC_Seconds += micro_phase.m_finish_WC - micro_phase.m_start_WC;
	micro_phase.m_CPU_Seconds += micro_phase.m_finish_CPU - micro_phase.m_start_CPU;
    }


    double sPhaseStatistics::get_CPU_Seconds(void)
    {
	struct tms tms_record;

	if (times(&tms_record) == -1)
	{
	    return SECONDS_UNDEFINED;
	}
	else
	{
	    return ((tms_record.tms_utime + tms_record.tms_stime + tms_record.tms_cutime + tms_record.tms_cstime) / (double)sysconf(_SC_CLK_TCK));
	}
    }


    void sPhaseStatistics::to_Screen(const sString &indent)
    {
	to_Stream(stdout, indent);
    }


    void sPhaseStatistics::to_Stream(FILE *fw, const sString &indent)
    {
	suspend_CurrentPhase();

	fprintf(fw, "%sPhase statistics (current phase = '%s') [\n", indent.c_str(), m_current_phase->m_name.c_str());
	to_Stream_subphases(fw, *m_root_phase, indent + sRELOC_INDENT);
	fprintf(fw, "%s]\n", indent.c_str());

	restart_CurrentPhase();
    }


    void sPhaseStatistics::to_Screen_subphases(const Phase &phase, const sString &indent)
    {
	to_Stream_subphases(stdout, phase, indent);
    }


    void sPhaseStatistics::to_Stream_subphases(FILE *fw, const Phase &phase, const sString &indent)
    {
	fprintf(fw, "%s%sPhase (name = '%s') [\n", indent.c_str(), sRELOC_INDENT.c_str(), phase.m_name.c_str());
	fprintf(fw, "%s%s%sTotal SAT solver calls            = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_total_sat_solver_Calls);
	fprintf(fw, "%s%s%sSatisfiable SAT solver calls      = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_SAT_sat_solver_Calls);
	fprintf(fw, "%s%s%sUnsatisfiable SAT solver calls    = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_UNSAT_sat_solver_Calls);
	fprintf(fw, "%s%s%sIndeterminate SAT solver calls    = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_INDET_sat_solver_Calls);
	fprintf(fw, "%s%s%sMove executions                   = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_move_Executions);
	fprintf(fw, "%s%s%sProduced CNF variables            = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_produced_cnf_Variables);
	fprintf(fw, "%s%s%sProduced CNF clauses              = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_produced_cnf_Clauses);
	fprintf(fw, "%s%s%sMaximum makespan tested           = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_max_makespan_tested);
	fprintf(fw, "%s%s%sMaximum cost tested               = %d\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_max_total_cost_tested);
	fprintf(fw, "%s%s%sSearch steps                      = %ld\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_search_Steps);
	fprintf(fw, "%s%s%sWall clock TIME (seconds)         = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_WC_Seconds);
	fprintf(fw, "%s%s%sCPU/machine TIME (seconds)        = %.3f\n", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str(), phase.m_CPU_Seconds);

	fprintf(fw, "%s%s%sMicro wall clock TIMEs (seconds)  = ", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	for (MicroPhases_map::const_iterator micro_phase = phase.m_micro_Phases.begin(); micro_phase != phase.m_micro_Phases.end(); ++micro_phase)
	{
	    fprintf(fw, "%d: %.3f  ", micro_phase->second.m_key, micro_phase->second.m_WC_Seconds);
	}
	fprintf(fw, "\n");	

	fprintf(fw, "%s%s%sMicro CPU/machine TIMEs (seconds) = ", indent.c_str(), sRELOC_INDENT.c_str(), sRELOC_INDENT.c_str());
	for (MicroPhases_map::const_iterator micro_phase = phase.m_micro_Phases.begin(); micro_phase != phase.m_micro_Phases.end(); ++micro_phase)
	{
	    fprintf(fw, "%d: %.3f  ", micro_phase->second.m_key, micro_phase->second.m_CPU_Seconds);
	}
	fprintf(fw, "\n");		
	
	fprintf(fw, "%s%s]\n", indent.c_str(), sRELOC_INDENT.c_str());

	if (!phase.m_sub_Phases.empty())
	{
	    fprintf(fw, "%s%sSub-phases {\n", indent.c_str(), sRELOC_INDENT.c_str());
	    for (Phases_map::const_iterator sub_phase = phase.m_sub_Phases.begin(); sub_phase != phase.m_sub_Phases.end(); ++sub_phase)
	    {
		to_Stream_subphases(fw, *sub_phase->second, indent + sRELOC_INDENT);
	    }
	    fprintf(fw, "%s%s}\n", indent.c_str(), sRELOC_INDENT.c_str());
	}
    }


/*----------------------------------------------------------------------------*/
// Global objects

    sPhaseStatistics s_GlobalPhaseStatistics;


/*----------------------------------------------------------------------------*/
// Global functions

    double sGet_WC_Seconds(void)
    {
	return sPhaseStatistics::get_WC_Seconds();
    }


    double sGet_CPU_Seconds(void)
    {
	return sPhaseStatistics::get_CPU_Seconds();
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
