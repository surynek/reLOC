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
/* compress.cpp / 0.21-robik_042                                              */
/*----------------------------------------------------------------------------*/
//
// Compression tools for relocation problem solutions.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <list>

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "result.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"



/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sMultirobotSolutionCompressor

    const sString sMultirobotSolutionCompressor::CNF_INVERSE_FILENAME_PREFIX = "_compress/makespan_compression_input.inv";
    const sString sMultirobotSolutionCompressor::CNF_ADVANCED_FILENAME_PREFIX = "_compress/makespan_compression_input.adv";
    const sString sMultirobotSolutionCompressor::CNF_DIFFERENTIAL_FILENAME_PREFIX = "_compress/makespan_compression_input.dif";
    const sString sMultirobotSolutionCompressor::CNF_BIJECTION_FILENAME_PREFIX = "_compress/makespan_compression_input.bij";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX = "_compress/makespan_compression_input.hadv";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX = "_compress/makespan_compression_input.hdif";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX = "_compress/makespan_compression_input.hbij";
    const sString sMultirobotSolutionCompressor::CNF_BITWISE_FILENAME_PREFIX = "_compress/makespan_compression_input.bit";
    const sString sMultirobotSolutionCompressor::CNF_FLOW_FILENAME_PREFIX = "_compress/makespan_compression_input.flw";
    const sString sMultirobotSolutionCompressor::CNF_MATCHING_FILENAME_PREFIX = "_compress/makespan_compression_input.mch";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_MATCHING_FILENAME_PREFIX = "_compress/makespan_compression_input.hmch";
    const sString sMultirobotSolutionCompressor::CNF_DIRECT_FILENAME_PREFIX = "_compress/makespan_compression_input.dir";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_DIRECT_FILENAME_PREFIX = "_compress/makespan_compression_input.hdir";
    const sString sMultirobotSolutionCompressor::CNF_SIMPLICIAL_FILENAME_PREFIX = "_compress/makespan_compression_input.sim";
    const sString sMultirobotSolutionCompressor::CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX = "_compress/makespan_compression_input.hsim";
    const sString sMultirobotSolutionCompressor::CNF_SINGULAR_FILENAME_PREFIX = "_compress/makespan_compression_input.sin";
    const sString sMultirobotSolutionCompressor::CNF_PLURAL_FILENAME_PREFIX = "_compress/makespan_compression_input.plu";
    const sString sMultirobotSolutionCompressor::CNF_PLURAL2_FILENAME_PREFIX = "_compress/makespan_compression_input.pl2";
    const sString sMultirobotSolutionCompressor::CNF_HEIGHTED_FILENAME_PREFIX = "_compress/makespan_compression_input.hei";
    const sString sMultirobotSolutionCompressor::CNF_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.mdd";
    const sString sMultirobotSolutionCompressor::CNF_MDD_UMTEX_FILENAME_PREFIX = "_compress/makespan_compression_input.mddu";
    const sString sMultirobotSolutionCompressor::CNF_MDD_MUTEX_FILENAME_PREFIX = "_compress/makespan_compression_input.mddx";    
    const sString sMultirobotSolutionCompressor::CNF_GMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.gmdd";
    const sString sMultirobotSolutionCompressor::CNF_GEMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.gemdd";
    const sString sMultirobotSolutionCompressor::CNF_ANO_FILENAME_PREFIX = "_compress/makespan_compression_input.ano";
    const sString sMultirobotSolutionCompressor::CNF_GANO_FILENAME_PREFIX = "_compress/makespan_compression_input.gano";     
    const sString sMultirobotSolutionCompressor::CNF_WATER_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.wmdd";
    const sString sMultirobotSolutionCompressor::CNF_RELAXED_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.rmdd";
    const sString sMultirobotSolutionCompressor::CNF_TOKEN_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.tmdd";
    const sString sMultirobotSolutionCompressor::CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.temdd";    
    const sString sMultirobotSolutionCompressor::CNF_PERMUTATION_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.pmdd";
    const sString sMultirobotSolutionCompressor::CNF_PERMUTATION_CMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.pcmdd";
    const sString sMultirobotSolutionCompressor::CNF_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.mmdd"; 
    const sString sMultirobotSolutionCompressor::CNF_RELAXED_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.rmmdd";
    const sString sMultirobotSolutionCompressor::CNF_TOKEN_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.tmmdd";
    const sString sMultirobotSolutionCompressor::CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.temmdd";    
    const sString sMultirobotSolutionCompressor::CNF_PERMUTATION_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.pmmdd";
    const sString sMultirobotSolutionCompressor::CNF_PERMUTATION_CMMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.pcmmdd";        
    const sString sMultirobotSolutionCompressor::CNF_MDD_plus_FILENAME_PREFIX = "_compress/makespan_compression_input.mdd+";
    const sString sMultirobotSolutionCompressor::CNF_MMDD_plus_FILENAME_PREFIX = "_compress/makespan_compression_input.mmdd+";
    const sString sMultirobotSolutionCompressor::CNF_MDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_input.mdd++";
    const sString sMultirobotSolutionCompressor::CNF_MDD_plus_plus_mutex_FILENAME_PREFIX = "_compress/makespan_compression_input.mddx++";    
    const sString sMultirobotSolutionCompressor::CNF_MDD_plus_plus_fuel_FILENAME_PREFIX = "_compress/makespan_compression_input.mddf++";    
    const sString sMultirobotSolutionCompressor::CNF_LMDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_input.lmdd++";
    const sString sMultirobotSolutionCompressor::CNF_MDD_star_FILENAME_PREFIX = "_compress/makespan_compression_input.mdd*";    
    const sString sMultirobotSolutionCompressor::CNF_MMDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_input.mmdd++";        
    const sString sMultirobotSolutionCompressor::CNF_ID_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.imdd";
    const sString sMultirobotSolutionCompressor::CNF_ID_WATER_MDD_FILENAME_PREFIX = "_compress/makespan_compression_input.iwmdd";
    const sString sMultirobotSolutionCompressor::CNF_RXMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.rxm";
    const sString sMultirobotSolutionCompressor::CNF_NOMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.nmd";
    const sString sMultirobotSolutionCompressor::CNF_RXNOMDD_FILENAME_PREFIX = "_compress/makespan_compression_input.rxn";

    const sString sMultirobotSolutionCompressor::OUTPUT_INVERSE_FILENAME_PREFIX = "_compress/makespan_compression_output.inv";
    const sString sMultirobotSolutionCompressor::OUTPUT_ADVANCED_FILENAME_PREFIX = "_compress/makespan_compression_output.adv";
    const sString sMultirobotSolutionCompressor::OUTPUT_DIFFERENTIAL_FILENAME_PREFIX = "_compress/makespan_compression_output.dif";
    const sString sMultirobotSolutionCompressor::OUTPUT_BIJECTION_FILENAME_PREFIX = "_compress/makespan_compression_output.bij";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX = "_compress/makespan_compression_output.hadv";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX = "_compress/makespan_compression_output.hdif";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX = "_compress/makespan_compression_output.hbij";
    const sString sMultirobotSolutionCompressor::OUTPUT_BITWISE_FILENAME_PREFIX = "_compress/makespan_compression_output.bit";
    const sString sMultirobotSolutionCompressor::OUTPUT_FLOW_FILENAME_PREFIX = "_compress/makespan_compression_output.flw";
    const sString sMultirobotSolutionCompressor::OUTPUT_MATCHING_FILENAME_PREFIX = "_compress/makespan_compression_output.mch";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX = "_compress/makespan_compression_output.hmch";
    const sString sMultirobotSolutionCompressor::OUTPUT_DIRECT_FILENAME_PREFIX = "_compress/makespan_compression_output.dir";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX = "_compress/makespan_compression_output.hdir";
    const sString sMultirobotSolutionCompressor::OUTPUT_SIMPLICIAL_FILENAME_PREFIX = "_compress/makespan_compression_output.sim";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX = "_compress/makespan_compression_output.hsim";
    const sString sMultirobotSolutionCompressor::OUTPUT_SINGULAR_FILENAME_PREFIX = "_compress/makespan_compression_output.sin";
    const sString sMultirobotSolutionCompressor::OUTPUT_PLURAL_FILENAME_PREFIX = "_compress/makespan_compression_output.plu";
    const sString sMultirobotSolutionCompressor::OUTPUT_PLURAL2_FILENAME_PREFIX = "_compress/makespan_compression_output.pl2";
    const sString sMultirobotSolutionCompressor::OUTPUT_HEIGHTED_FILENAME_PREFIX = "_compress/makespan_compression_output.hei";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.mdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_UMTEX_FILENAME_PREFIX = "_compress/makespan_compression_output.mddu";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_MUTEX_FILENAME_PREFIX = "_compress/makespan_compression_output.mddx";    
    const sString sMultirobotSolutionCompressor::OUTPUT_GMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.gmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_GEMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.gemdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_ANO_FILENAME_PREFIX = "_compress/makespan_compression_output.ano";
    const sString sMultirobotSolutionCompressor::OUTPUT_GANO_FILENAME_PREFIX = "_compress/makespan_compression_output.gano";    
    const sString sMultirobotSolutionCompressor::OUTPUT_WATER_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.wmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_RELAXED_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.rmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_TOKEN_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.tmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.temdd";    
    const sString sMultirobotSolutionCompressor::OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.pmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.pcmdd";                
    const sString sMultirobotSolutionCompressor::OUTPUT_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.mmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_RELAXED_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.rmmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_TOKEN_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.tmmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.temmdd";    
    const sString sMultirobotSolutionCompressor::OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.pmmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.pcmmdd";        
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_plus_FILENAME_PREFIX = "_compress/makespan_compression_output.mdd+";
    const sString sMultirobotSolutionCompressor::OUTPUT_MMDD_plus_FILENAME_PREFIX = "_compress/makespan_compression_output.mmdd+";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_output.mdd++";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX = "_compress/makespan_compression_output.mddx++";    
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX = "_compress/makespan_compression_output.mddf++";    
    const sString sMultirobotSolutionCompressor::OUTPUT_LMDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_output.lmdd++";
    const sString sMultirobotSolutionCompressor::OUTPUT_MDD_star_FILENAME_PREFIX = "_compress/makespan_compression_output.mdd*";    
    const sString sMultirobotSolutionCompressor::OUTPUT_MMDD_plus_plus_FILENAME_PREFIX = "_compress/makespan_compression_output.mmdd++";    
    const sString sMultirobotSolutionCompressor::OUTPUT_ID_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.imdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_ID_WATER_MDD_FILENAME_PREFIX = "_compress/makespan_compression_output.iwmdd";
    const sString sMultirobotSolutionCompressor::OUTPUT_RXMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.rxm";
    const sString sMultirobotSolutionCompressor::OUTPUT_NOMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.nmd";
    const sString sMultirobotSolutionCompressor::OUTPUT_RXNOMDD_FILENAME_PREFIX = "_compress/makespan_compression_output.rxn";


/*----------------------------------------------------------------------------*/

    sMultirobotSolutionCompressor::CompressionRecord::CompressionRecord()
	: m_makespan(MAKESPAN_UNDEFINED)
    {
	// nothing
    }


    sMultirobotSolutionCompressor::CompressionRecord::CompressionRecord(int start_step, int final_step)
	: m_makespan(MAKESPAN_UNDEFINED)
	, m_start_step(start_step)
	, m_final_step(final_step)
    {
	// nothing
    }


    sMultirobotSolutionCompressor::CompressionRecord::CompressionRecord(int                        makespan,
									int                        start_step,
									int                        final_step,
									CompressionRecord         *left_component,
									CompressionRecord         *right_component,
									const sMultirobotSolution &compressed_sub_solution)
	: m_makespan(makespan)
	, m_start_step(start_step)
	, m_final_step(final_step)
	, m_left_component(left_component)
	, m_right_component(right_component)
	, m_compressed_sub_solution(compressed_sub_solution)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolutionCompressor::ProcessingArgument_IN::ProcessingArgument_IN()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument_IN::ProcessingArgument_IN(const sRobotArrangement   &initial_arrangement,
										const sMultirobotSolution &original_solution,
										sUndirectedGraph          &environment,
										const sUndirectedGraph    &sparse_environment)
	: m_initial_arrangement(initial_arrangement)
	, m_original_solution(original_solution)
	, m_environment(&environment)
	, m_sparse_environment(&sparse_environment)
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument_OUT::ProcessingArgument_OUT()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument_CTX::ProcessingArgument_CTX()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument_CTX::ProcessingArgument_CTX(sMultirobotSolutionCompressor &compressor)
	: m_compressor(&compressor)
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument::ProcessingArgument()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessingArgument::ProcessingArgument(const ProcessingArgument_CTX &ctx_arg,
									  const ProcessingArgument_IN  &in_arg)
	: m_ctx_arg(ctx_arg)
	, m_in_arg(in_arg)
    {
	// nothing
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolutionCompressor::ProcessedSolutionRecord::ProcessedSolutionRecord()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::ProcessedSolutionRecord::ProcessedSolutionRecord(int start_step, int final_step, const sMultirobotSolution &processed_solution)
	: m_start_step(start_step)
	, m_final_step(final_step)
	, m_processed_solution(processed_solution)
    {
	// nothing
    }

    bool sMultirobotSolutionCompressor::ProcessedSolutionRecord::operator==(const ProcessedSolutionRecord &processed_record) const
    {
	return (m_start_step == processed_record.m_start_step);
    }


    bool sMultirobotSolutionCompressor::ProcessedSolutionRecord::operator<(const ProcessedSolutionRecord &processed_record) const
    {
	return (m_start_step < processed_record.m_start_step);
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolutionCompressor::AttemptDatabaseRecord::AttemptDatabaseRecord()
    {
	// nothing
    }


    sMultirobotSolutionCompressor::AttemptDatabaseRecord::AttemptDatabaseRecord(const sRobotArrangement &initial_arrangement, const sRobotArrangement &goal_arrangement)
	: m_initial_arrangement(initial_arrangement)
	, m_goal_arrangement(goal_arrangement)
    {
	// nothing
    }

    
    bool sMultirobotSolutionCompressor::AttemptDatabaseRecord::operator==(const AttemptDatabaseRecord &attempt_record) const
    {
	return (   m_initial_arrangement == attempt_record.m_initial_arrangement
		&& m_goal_arrangement == attempt_record.m_goal_arrangement);
    }


    bool sMultirobotSolutionCompressor::AttemptDatabaseRecord::operator<(const AttemptDatabaseRecord &attempt_record) const
    {
	return (    m_initial_arrangement < attempt_record.m_initial_arrangement
		    || (m_initial_arrangement == attempt_record.m_initial_arrangement && m_goal_arrangement < attempt_record.m_goal_arrangement));
    }


/*----------------------------------------------------------------------------*/

    sMultirobotSolutionCompressor::sMultirobotSolutionCompressor(const sString &minisat_path,
								 int            minisat_timeout,
								 int            total_timeout,
								 int            makespan_upper_bound,
								 int            N_parallel_Threads,
								 Encoding       encoding)
	: m_minisat_timeout(minisat_timeout)
	, m_total_timeout(total_timeout)
	, m_makespan_upper_bound(makespan_upper_bound)
	, m_minisat_path(minisat_path)
	, m_N_parallel_Threads(N_parallel_Threads)
	, m_encoding(encoding)
	, m_ratio(-1.0)
	, m_robustness(1)
	, m_range(0)
    {
	// nothing
    }


    void sMultirobotSolutionCompressor::set_Ratio(double ratio)
    {
	m_ratio = ratio;
    }


    void sMultirobotSolutionCompressor::set_Robustness(int robustness)
    {
	m_robustness = robustness;
    }

    
    void sMultirobotSolutionCompressor::set_Range(int range)
    {
	m_range = range;
    }        


    int sMultirobotSolutionCompressor::calc_MakespanLowerBound(const sRobotArrangement                     &start_arrangement,
							       const sRobotArrangement                     &final_arrangement,
							       const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances)
    {
	int maximum_distance = 0;
	int N_Robots = start_arrangement.get_RobotCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int u_id = start_arrangement.get_RobotLocation(robot_id);
	    int v_id = final_arrangement.get_RobotLocation(robot_id);

	    int distance = all_pairs_Distances[u_id][v_id];

	    if (distance > maximum_distance)
	    {
		maximum_distance = distance;
	    }
	}

	return maximum_distance;
    }


    int sMultirobotSolutionCompressor::calc_MakespanLowerBound(const sRobotArrangement                     &start_arrangement,
							       const sRobotGoal                            &robot_goal,
							       const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances)
    {
	int maximum_distance = 0;
	int N_Robots = start_arrangement.get_RobotCount();

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int u_id = start_arrangement.get_RobotLocation(robot_id);

	    const sRobotGoal::Vertices_set &goal_IDs = robot_goal.get_RobotGoal(robot_id);
	    sASSERT(!goal_IDs.empty());
	    sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin();
	   
	    int min_distance = all_pairs_Distances[u_id][*goal_id];
	    while (++goal_id != goal_IDs.end())
	    {
		if (min_distance > all_pairs_Distances[u_id][*goal_id])
		{
		    min_distance = all_pairs_Distances[u_id][*goal_id];
		}
	    }
	    if (min_distance > maximum_distance)
	    {
		maximum_distance = min_distance;
	    }
	}

	return maximum_distance;
    }


/*----------------------------------------------------------------------------*/

    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(const sRobotArrangement &start_arrangement,
								   const sRobotGoal        &final_arrangement,
								   const sUndirectedGraph  &environment,
								   const sUndirectedGraph  &sparse_environment,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan(start_arrangement, final_arrangement, environment, sparse_environment, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver         **solver,
								     const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan(solver, start_arrangement, final_arrangement, environment, sparse_environment, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_(const sRobotArrangement &start_arrangement,
								    const sRobotGoal        &final_arrangement,
								    const sUndirectedGraph  &environment,
								    const sUndirectedGraph  &sparse_environment,
								    int                      makespan_upper_bound,
								    int                     &optimal_makespan,
								    int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan_(start_arrangement, final_arrangement, environment, sparse_environment, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_(Glucose::Solver         **solver,
								      const sRobotArrangement &start_arrangement,
								      const sRobotGoal        &final_arrangement,
								      const sUndirectedGraph  &environment,
								      const sUndirectedGraph  &sparse_environment,
								      int                      makespan_upper_bound,
								      int                     &optimal_makespan,
								      int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan_(solver, start_arrangement, final_arrangement, environment, sparse_environment, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(const sRobotArrangement &start_arrangement,
								   const sRobotGoal        &final_arrangement,
								   const sUndirectedGraph  &environment,
								   const sUndirectedGraph  &sparse_environment,
								   int                      makespan_lower_bound,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan(start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver         **solver,
								     const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                      makespan_lower_bound,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan(solver, start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(sMultirobotInstance     &instance,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan(instance, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver         **solver,
								     sMultirobotInstance     &instance,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan(solver, instance, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_(sMultirobotInstance     &instance,
								    int                      makespan_upper_bound,
								    int                     &optimal_makespan,
								    int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan_(instance, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_(Glucose::Solver         **solver,
								      sMultirobotInstance     &instance,
								      int                      makespan_upper_bound,
								      int                     &optimal_makespan,
								      int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan_(solver, instance, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(sMultirobotInstance     &instance,
								   int                      makespan_lower_bound,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_OptimalMakespan(instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver         **solver,
								     sMultirobotInstance     &instance,
								     int                      makespan_lower_bound,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_OptimalMakespan(solver, instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, dummy_final_encoding_context, thread_id);
    }        


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(const sRobotArrangement           &start_arrangement,
								   const sRobotGoal                  &final_arrangement,
								   const sUndirectedGraph            &environment,
								   const sUndirectedGraph            &sparse_environment,
								   int                                makespan_upper_bound,
								   int                               &optimal_makespan,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalMakespan(instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver                   **solver,
								     const sRobotArrangement           &start_arrangement,
								     const sRobotGoal                  &final_arrangement,
								     const sUndirectedGraph            &environment,
								     const sUndirectedGraph            &sparse_environment,
								     int                                makespan_upper_bound,
								     int                               &optimal_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalMakespan(solver, instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }
    
    
    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(sMultirobotInstance               &instance,
								   int                                makespan_upper_bound,
								   int                               &optimal_makespan,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;

	int N_Layers = makespan_upper_bound + 1;
	optimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (N_Layers >= 2)
	{
	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	    sString cnf_filename, cnf_out_filename, output_filename;

	    switch (m_encoding)
	    {
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicAdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicBijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_InverseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BitwiseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_FlowCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicMatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_DirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_AnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_GAnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_RelaxedMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_TokenMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_TokenEmptyMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_PermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_CapacitatedPermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicDirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_SimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicSimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";

	    int preprocess_result = system(preprocess_call.c_str());

	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());

	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
	    #endif

	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);

            #ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
            #endif

	    if (strcmp(answer, "UNSAT") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
                #endif

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
		return sRESULT_SUCCESS;
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
                #endif


                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
		optimal_makespan = MAKESPAN_UNDEFINED;
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
                #endif
		optimal_makespan = N_Layers - 1;
		final_encoding_context = encoding_context;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    --N_Layers;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver                   **solver,
								     sMultirobotInstance               &instance,
								     int                                makespan_upper_bound,
								     int                               &optimal_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                sUNUSED(thread_id))
    {
	int N_Layers = makespan_upper_bound + 1;
	optimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (N_Layers >= 2)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

	    switch (m_encoding)
	    {
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		instance.to_Memory_HeuristicAdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		instance.to_Memory_HeuristicDifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		instance.to_Memory_HeuristicBijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		instance.to_Memory_DifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		instance.to_Memory_BijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		instance.to_Memory_AdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		instance.to_Memory_InverseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		instance.to_Memory_BitwiseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_FLOW:
	    {
		instance.to_Memory_FlowCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		instance.to_Memory_MatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		instance.to_Memory_HeuristicMatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		instance.to_Memory_DirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD:
	    {
		instance.to_Memory_MmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ANO:
	    {
		instance.to_Memory_AnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_GANO:
	    {
		instance.to_Memory_GAnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		instance.to_Memory_RelaxedMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		instance.to_Memory_TokenMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		instance.to_Memory_TokenEmptyMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		instance.to_Memory_PermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		instance.to_Memory_CapacitatedPermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		instance.to_Memory_MmddPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		instance.to_Memory_MmddPlusPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		instance.to_Memory_HeuristicDirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		instance.to_Memory_SimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		instance.to_Memory_HeuristicSimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		rlimit rl;
		getrlimit(RLIMIT_CPU, &rl);
		
		if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
		{
		    rl.rlim_cur = m_minisat_timeout;
		    
		    if (setrlimit(RLIMIT_CPU, &rl) == -1)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		    }
		}
	    } 
	    
	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		optimal_makespan = N_Layers - 1;
		final_encoding_context = encoding_context;
		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_makespan = MAKESPAN_UNDEFINED;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	    }
	    
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    --N_Layers;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_SpecifiedMakespan(Glucose::Solver                   **solver,
								       sMultirobotInstance               &instance,
								       int                               &specified_makespan,
								       sMultirobotEncodingContext_CNFsat &final_encoding_context,
								       int                                sUNUSED(thread_id))
    {
	int max_individual_cost = 0;
	instance.estimate_TotalCost(max_individual_cost);

	if (specified_makespan < max_individual_cost)
	{
	    specified_makespan = MAKESPAN_UNDEFINED;
	    return sRESULT_SUCCESS;
	}	
	int N_Layers = specified_makespan + 1;

	if (*solver != NULL)
	{
	    delete *solver;
	}
	*solver = new Glucose::Solver;
	    
	#ifdef sVERBOSE
	{
	    printf("Solving layer: %d\n", N_Layers);
	}
        #endif

	sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

	switch (m_encoding)
	{
	case ENCODING_HEURISTIC_ADVANCED:
	{
	    instance.to_Memory_HeuristicAdvancedCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_HEURISTIC_DIFFERENTIAL:
	{
	    instance.to_Memory_HeuristicDifferentialCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_HEURISTIC_BIJECTION:
	{
	    instance.to_Memory_HeuristicBijectionCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_DIFFERENTIAL:
	{
	    instance.to_Memory_DifferentialCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_BIJECTION:
	{
	    instance.to_Memory_BijectionCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_ADVANCED:
	{
	    instance.to_Memory_AdvancedCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_INVERSE:
	{
	    instance.to_Memory_InverseCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_BITWISE:
	{
	    instance.to_Memory_BitwiseCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_FLOW:
	{
	    instance.to_Memory_FlowCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MATCHING:
	{
	    instance.to_Memory_MatchingCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_HEURISTIC_MATCHING:
	{
	    instance.to_Memory_HeuristicMatchingCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_DIRECT:
	{
	    instance.to_Memory_DirectCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MMDD:
	{
	    instance.to_Memory_MmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_ANO:
	{
	    instance.to_Memory_AnoCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_GANO:
	{
	    instance.to_Memory_GAnoCNFsat(*solver, encoding_context, "", false);
	    break;
	}	    	    
	case ENCODING_RELAXED_MMDD:
	{
	    instance.to_Memory_RelaxedMmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_MMDD:
	{
	    instance.to_Memory_TokenMmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MMDD:
	{
	    instance.to_Memory_TokenEmptyMmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_PERMUTATION_MMDD:
	{
	    instance.to_Memory_PermutationMmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_PERMUTATION_CMMDD:
	{
	    instance.to_Memory_CapacitatedPermutationMmddCNFsat(*solver, encoding_context, "", false);
	    break;
	}	    	    		
	case ENCODING_MMDD_plus:
	{
	    instance.to_Memory_MmddPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MMDD_plus_plus:
	{
	    instance.to_Memory_MmddPlusPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_HEURISTIC_DIRECT:
	{
	    instance.to_Memory_HeuristicDirectCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_SIMPLICIAL:
	{
	    instance.to_Memory_SimplicialCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_HEURISTIC_SIMPLICIAL:
	{
	    instance.to_Memory_HeuristicSimplicialCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_SINGULAR:
	{
	    instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_PLURAL:
	{
	    instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_PLURAL2:
	{
	    instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    rlimit rl;
	    getrlimit(RLIMIT_CPU, &rl);
	    
	    if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
	    {
		rl.rlim_cur = m_minisat_timeout;
		
		if (setrlimit(RLIMIT_CPU, &rl) == -1)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		}
	    }
	} 
	
	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}

	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);
	
	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;
	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    specified_makespan = MAKESPAN_UNDEFINED;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    specified_makespan = MAKESPAN_UNDEFINED;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	}
	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_(const sRobotArrangement           &start_arrangement,
								    const sRobotGoal                  &final_arrangement,
								    const sUndirectedGraph            &environment,
								    const sUndirectedGraph            &sparse_environment,
								    int                                makespan_upper_bound,
								    int                               &optimal_makespan,
								    sMultirobotEncodingContext_CNFsat &final_encoding_context,
								    int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalMakespan_(instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_(Glucose::Solver                   **solver,
								      const sRobotArrangement           &start_arrangement,
								      const sRobotGoal                  &final_arrangement,
								      const sUndirectedGraph            &environment,
								      const sUndirectedGraph            &sparse_environment,
								      int                                makespan_upper_bound,
								      int                               &optimal_makespan,
								      sMultirobotEncodingContext_CNFsat &final_encoding_context,
								      int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalMakespan_(solver, instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_(sMultirobotInstance               &instance,
								    int                                makespan_upper_bound,
								    int                               &optimal_makespan,
								    sMultirobotEncodingContext_CNFsat &final_encoding_context,
								    int                                thread_id)
    {
	sResult result;
	int max_individual_cost = 0;
	instance.estimate_TotalCost(max_individual_cost);

	int N_Layers = max_individual_cost + 1;
	optimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (N_Layers <= makespan_upper_bound + 1)
	{

	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	    sString cnf_filename, cnf_out_filename, output_filename;

	    switch (m_encoding)
	    {
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicAdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicBijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }

	    case ENCODING_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_InverseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BitwiseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_FlowCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicMatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_DirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_AnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_GAnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_RELAXED_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_RelaxedMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_TokenMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_TokenEmptyMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_PermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_CapacitatedPermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_MmddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicDirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_SimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_HeuristicSimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename  +  " 1>/dev/null";

	    int preprocess_result = system(preprocess_call.c_str());

	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());

	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
	    #endif

	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);

            #ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
            #endif

	    if (strcmp(answer, "UNSAT") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
                #endif


                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
	    }
	    else if (strcmp(answer, "") == 0 || strcmp(answer, "INDET") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
                #endif
		optimal_makespan = MAKESPAN_UNDEFINED;

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
                #endif
		optimal_makespan = N_Layers - 1;
		final_encoding_context = encoding_context;
		return sRESULT_SUCCESS;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    ++N_Layers;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compute_SpecifiedMakespan(sMultirobotInstance               &instance,
								     int                               &specified_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                thread_id)
    {
	sResult result;

	int max_individual_cost = 0;
	instance.estimate_TotalCost(max_individual_cost);

	if (specified_makespan < max_individual_cost)
	{
	    specified_makespan = MAKESPAN_UNDEFINED;
	    return sRESULT_SUCCESS;
	}
	int N_Layers = specified_makespan + 1;

        #ifdef sVERBOSE
	printf("Solving layer: %d\n", N_Layers);
        #endif

	sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	sString cnf_filename, cnf_out_filename, output_filename;

	switch (m_encoding)
	{
	case ENCODING_HEURISTIC_ADVANCED:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_HeuristicAdvancedCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_HEURISTIC_DIFFERENTIAL:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_HeuristicDifferentialCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_HEURISTIC_BIJECTION:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_HeuristicBijectionCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}

	case ENCODING_DIFFERENTIAL:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_DifferentialCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_BIJECTION:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_BijectionCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_ADVANCED:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_AdvancedCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_INVERSE:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_InverseCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	    }
	case ENCODING_BITWISE:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_BitwiseCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_FLOW:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_FlowCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MATCHING:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MatchingCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_HEURISTIC_MATCHING:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_HeuristicMatchingCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_DIRECT:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_DirectCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_ANO:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_AnoCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
		}
	    break;
	}
	case ENCODING_GANO:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_GAnoCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	    
	case ENCODING_RELAXED_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_RelaxedMmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_TokenMmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_TokenEmptyMmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_PERMUTATION_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_PermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_PERMUTATION_CMMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_CapacitatedPermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	    	    		
	case ENCODING_MMDD_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MmddPlusCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MMDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MmddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_HEURISTIC_DIRECT:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_HeuristicDirectCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_SIMPLICIAL:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_SimplicialCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_HEURISTIC_SIMPLICIAL:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_HeuristicSimplicialCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_SINGULAR:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_PLURAL:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_PLURAL2:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename  +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	int system_result = system(system_call.c_str());
	
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
        #ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
	#endif

	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);

        #ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
        #endif

	if (strcmp(answer, "UNSAT") == 0)
	{
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
            #endif
	    specified_makespan = MAKESPAN_UNDEFINED;

            #ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
            #endif
	    return sRESULT_SUCCESS;
	}
	else if (strcmp(answer, "") == 0 || strcmp(answer, "INDET") == 0)
	{
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
            #endif
	    specified_makespan = MAKESPAN_UNDEFINED;

            #ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
            #endif
	    return sRESULT_SUCCESS;
	}
	else /*if (strcmp(answer, "SAT") == 0)*/
	{
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
            #endif
	    final_encoding_context = encoding_context;
	    return sRESULT_SUCCESS;
	}

	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_(Glucose::Solver                   **solver,
								      sMultirobotInstance               &instance,
								      int                                makespan_upper_bound,
								      int                               &optimal_makespan,
								      sMultirobotEncodingContext_CNFsat &final_encoding_context,
								      int                                sUNUSED(thread_id))
    {
	int max_individual_cost = 0;
	instance.estimate_TotalCost(max_individual_cost);

	int N_Layers = max_individual_cost + 1;
	optimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (N_Layers <= makespan_upper_bound + 1)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;

	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

	    switch (m_encoding)
	    {
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		instance.to_Memory_HeuristicAdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		instance.to_Memory_HeuristicDifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		instance.to_Memory_HeuristicBijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }

	    case ENCODING_DIFFERENTIAL:
	    {
		instance.to_Memory_DifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		instance.to_Memory_BijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		instance.to_Memory_AdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		instance.to_Memory_InverseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		instance.to_Memory_BitwiseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_FLOW:
	    {
		instance.to_Memory_FlowCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		instance.to_Memory_MatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		instance.to_Memory_HeuristicMatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		instance.to_Memory_DirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD:
	    {
		instance.to_Memory_MmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ANO:
	    {
		instance.to_Memory_AnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_GANO:
	    {
		instance.to_Memory_GAnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		instance.to_Memory_RelaxedMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		instance.to_Memory_TokenMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		instance.to_Memory_TokenEmptyMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		instance.to_Memory_PermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		instance.to_Memory_CapacitatedPermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		instance.to_Memory_MmddPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		instance.to_Memory_MmddPlusPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		instance.to_Memory_HeuristicDirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		instance.to_Memory_SimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		instance.to_Memory_HeuristicSimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		rlimit rl;
		getrlimit(RLIMIT_CPU, &rl);
		
		if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
		{
		    rl.rlim_cur = m_minisat_timeout;
		    
		    if (setrlimit(RLIMIT_CPU, &rl) == -1)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		    }
		}
	    } 
	    
	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		optimal_makespan = N_Layers - 1;
		final_encoding_context = encoding_context;
		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
//		optimal_makespan = MAKESPAN_UNDEFINED;
//		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	    }
	    
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    ++N_Layers;
	}
	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(const sRobotArrangement           &start_arrangement,
								   const sRobotGoal                  &final_arrangement,
								   const sUndirectedGraph            &environment,
								   const sUndirectedGraph            &sparse_environment,
								   int                                makespan_lower_bound,
								   int                                makespan_upper_bound,
								   int                               &optimal_makespan,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalMakespan(instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver                   **solver,
								     const sRobotArrangement           &start_arrangement,
								     const sRobotGoal                  &final_arrangement,
								     const sUndirectedGraph            &environment,
								     const sUndirectedGraph            &sparse_environment,
								     int                                makespan_lower_bound,
								     int                                makespan_upper_bound,
								     int                               &optimal_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalMakespan(solver, instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }
     

    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan(sMultirobotInstance               &instance,
								   int                                makespan_lower_bound,
								   int                                makespan_upper_bound,
								   int                               &optimal_makespan,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;
	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	optimal_makespan = MAKESPAN_UNDEFINED;

	int first_unsatisfiable_makespan = makespan_lower_bound - 1;
	int last_satisfiable_makespan = makespan_upper_bound + 1;

	while (last_satisfiable_makespan - first_unsatisfiable_makespan > 1)
	{
	    int makespan_try = last_satisfiable_makespan - (last_satisfiable_makespan - first_unsatisfiable_makespan) / 2;
	    int N_Layers = makespan_try + 1;

	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	    sString cnf_filename, cnf_out_filename, output_filename;

	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_InverseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicAdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicBijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BitwiseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_FlowCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicMatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_GAnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_RelaxedMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_TokenMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_TokenEmptyMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_PermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_CapacitatedPermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_SimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicSimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";

	    int preprocess_result = system(preprocess_call.c_str());

	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());

	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
	    #endif

	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }

	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);

            #ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
            #endif

	    if (strcmp(answer, "UNSAT") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
                #endif
		first_unsatisfiable_makespan = makespan_try;

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
                #endif
		optimal_makespan = MAKESPAN_UNDEFINED;

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
                #endif
		final_encoding_context = encoding_context;
		last_satisfiable_makespan = makespan_try;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}

	if (last_satisfiable_makespan <= makespan_upper_bound)
	{
	    optimal_makespan = last_satisfiable_makespan;
	}
	else
	{
	    optimal_makespan = MAKESPAN_UNDEFINED;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan(Glucose::Solver                   **solver,
								     sMultirobotInstance               &instance,
								     int                                makespan_lower_bound,
								     int                                makespan_upper_bound,
								     int                               &optimal_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                sUNUSED(thread_id))
    {
	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	optimal_makespan = MAKESPAN_UNDEFINED;

	int first_unsatisfiable_makespan = makespan_lower_bound - 1;
	int last_satisfiable_makespan = makespan_upper_bound + 1;

	while (last_satisfiable_makespan - first_unsatisfiable_makespan > 1)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    int makespan_try = last_satisfiable_makespan - (last_satisfiable_makespan - first_unsatisfiable_makespan) / 2;
	    int N_Layers = makespan_try + 1;

	    #ifdef sVERBOSE
	    printf("Solving layer: %d\n", N_Layers);
	    #endif

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	    sString cnf_filename, cnf_out_filename, output_filename;

	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		instance.to_Memory_InverseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		instance.to_Memory_AdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		instance.to_Memory_DifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		instance.to_Memory_BijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		instance.to_Memory_HeuristicAdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		instance.to_Memory_HeuristicDifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		instance.to_Memory_HeuristicBijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		instance.to_Memory_BitwiseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_FLOW:
	    {
		instance.to_Memory_FlowCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		instance.to_Memory_MatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		instance.to_Memory_HeuristicMatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		instance.to_Memory_DirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD:
	    {
		instance.to_Memory_MmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ANO:
	    {
		instance.to_Memory_AnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_GANO:
	    {
		instance.to_Memory_GAnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		instance.to_Memory_RelaxedMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		instance.to_Memory_TokenMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		instance.to_Memory_TokenEmptyMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		instance.to_Memory_PermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		instance.to_Memory_CapacitatedPermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		instance.to_Memory_MmddPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		instance.to_Memory_MmddPlusPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		instance.to_Memory_HeuristicDirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		instance.to_Memory_SimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		instance.to_Memory_HeuristicSimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	    
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		rlimit rl;
		getrlimit(RLIMIT_CPU, &rl);
		
		if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
		{
		    rl.rlim_cur = m_minisat_timeout;
		    
		    if (setrlimit(RLIMIT_CPU, &rl) == -1)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		    }
		}
	    } 
	    
	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		first_unsatisfiable_makespan = makespan_try;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		last_satisfiable_makespan = makespan_try;
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		first_unsatisfiable_makespan = makespan_try;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_makespan = MAKESPAN_UNDEFINED;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	    }
	    
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	if (last_satisfiable_makespan <= makespan_upper_bound)
	{
	    optimal_makespan = last_satisfiable_makespan;
	}
	else
	{
	    optimal_makespan = MAKESPAN_UNDEFINED;
	}

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalSolution(const sRobotArrangement &start_arrangement,
								   const sRobotGoal        &final_arrangement,
								   const sUndirectedGraph  &environment,
								   const sUndirectedGraph  &sparse_environment,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   sMultirobotSolution     &optimal_solution,
								   int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_OptimalMakespan(instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = extract_ComputedDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = extract_ComputedInverseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = extract_ComputedAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = extract_ComputedBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = extract_ComputedHeuristicAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = extract_ComputedHeuristicBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = extract_ComputedHeuristicDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = extract_ComputedBitwiseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = extract_ComputedFlowSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = extract_ComputedMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = extract_ComputedHeuristicMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = extract_ComputedDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = extract_ComputedMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = extract_ComputedAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = extract_ComputedGAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = extract_ComputedRelaxedMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = extract_ComputedTokenMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = extract_ComputedTokenEmptyMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		result = extract_ComputedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		result = extract_ComputedCapacitatedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = extract_ComputedMmddPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = extract_ComputedMmddPlusPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = extract_ComputedHeuristicDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = extract_ComputedSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = extract_ComputedHeuristicSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compute_SpecifiedSolution(const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                     &specified_makespan,
								     sMultirobotSolution     &specified_solution,
								     int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_SpecifiedMakespan(instance, specified_makespan, final_encoding_context, thread_id);
	
	if (sFAILED(result))
	{
	    return result;
	}
	if (specified_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = extract_ComputedDifferentialSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = extract_ComputedInverseSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = extract_ComputedAdvancedSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = extract_ComputedBijectionSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = extract_ComputedHeuristicAdvancedSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = extract_ComputedHeuristicBijectionSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = extract_ComputedHeuristicDifferentialSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = extract_ComputedBitwiseSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = extract_ComputedFlowSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = extract_ComputedMatchingSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = extract_ComputedHeuristicMatchingSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = extract_ComputedDirectSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = extract_ComputedMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = extract_ComputedAnoSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = extract_ComputedGAnoSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = extract_ComputedRelaxedMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = extract_ComputedTokenMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = extract_ComputedTokenEmptyMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		result = extract_ComputedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		result = extract_ComputedCapacitatedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = extract_ComputedMmddPlusSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = extract_ComputedMmddPlusPlusSolution(start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = extract_ComputedHeuristicDirectSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = extract_ComputedSimplicialSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = extract_ComputedHeuristicSimplicialSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, specified_makespan, final_encoding_context, specified_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotSolutionCompressor::incompute_OptimalSolution(Glucose::Solver         **solver,
								     const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     sMultirobotSolution     &optimal_solution,
								     int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_OptimalMakespan(solver, instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = intract_ComputedDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = intract_ComputedInverseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = intract_ComputedAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = intract_ComputedBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = intract_ComputedHeuristicAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = intract_ComputedHeuristicBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = intract_ComputedHeuristicDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = intract_ComputedBitwiseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = intract_ComputedFlowSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = intract_ComputedMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = intract_ComputedHeuristicMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = intract_ComputedDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = intract_ComputedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = intract_ComputedAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = intract_ComputedGAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = intract_ComputedRelaxedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = intract_ComputedTokenMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = intract_ComputedTokenEmptyMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:	
	    {
		result = intract_ComputedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:	
	    {
		result = intract_ComputedCapacitatedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = intract_ComputedMmddPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = intract_ComputedMmddPlusPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = intract_ComputedHeuristicDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = intract_ComputedSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = intract_ComputedHeuristicSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_SpecifiedSolution(Glucose::Solver         **solver,
								       const sRobotArrangement &start_arrangement,
								       const sRobotGoal        &final_arrangement,
								       const sUndirectedGraph  &environment,
								       const sUndirectedGraph  &sparse_environment,
								       int                     &specified_makespan,
								       sMultirobotSolution     &specified_solution,
								       int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_SpecifiedMakespan(solver, instance, specified_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (specified_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = intract_ComputedDifferentialSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = intract_ComputedInverseSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = intract_ComputedAdvancedSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = intract_ComputedBijectionSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = intract_ComputedHeuristicAdvancedSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = intract_ComputedHeuristicBijectionSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = intract_ComputedHeuristicDifferentialSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = intract_ComputedBitwiseSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = intract_ComputedFlowSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = intract_ComputedMatchingSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = intract_ComputedHeuristicMatchingSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = intract_ComputedDirectSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = intract_ComputedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = intract_ComputedAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = intract_ComputedGAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = intract_ComputedRelaxedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = intract_ComputedTokenMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = intract_ComputedTokenEmptyMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		result = intract_ComputedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		result = intract_ComputedCapacitatedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = intract_ComputedMmddPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = intract_ComputedMmddPlusPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = intract_ComputedHeuristicDirectSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = intract_ComputedSimplicialSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = intract_ComputedHeuristicSimplicialSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, specified_makespan, final_encoding_context, specified_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotSolutionCompressor::compute_OptimalSolution_(const sRobotArrangement &start_arrangement,
								    const sRobotGoal        &final_arrangement,
								    const sUndirectedGraph  &environment,
								    const sUndirectedGraph  &sparse_environment,
								    int                      makespan_upper_bound,
								    int                     &optimal_makespan,
								    sMultirobotSolution     &optimal_solution,
								    int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_OptimalMakespan_(instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = extract_ComputedDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = extract_ComputedInverseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = extract_ComputedAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = extract_ComputedBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = extract_ComputedHeuristicAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = extract_ComputedHeuristicBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = extract_ComputedHeuristicDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = extract_ComputedBitwiseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = extract_ComputedFlowSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = extract_ComputedMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = extract_ComputedHeuristicMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = extract_ComputedDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = extract_ComputedMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = extract_ComputedAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = extract_ComputedGAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = extract_ComputedRelaxedMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = extract_ComputedTokenMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = extract_ComputedTokenEmptyMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		result = extract_ComputedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		result = extract_ComputedCapacitatedPermutationMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = extract_ComputedMmddPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = extract_ComputedMmddPlusPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = extract_ComputedHeuristicDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = extract_ComputedSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = extract_ComputedHeuristicSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalSolution_(Glucose::Solver         **solver,
								      const sRobotArrangement &start_arrangement,
								      const sRobotGoal        &final_arrangement,
								      const sUndirectedGraph  &environment,
								      const sUndirectedGraph  &sparse_environment,
								      int                      makespan_upper_bound,
								      int                     &optimal_makespan,
								      sMultirobotSolution     &optimal_solution,
								      int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_OptimalMakespan_(solver, instance, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_DIFFERENTIAL:
	    {
		result = intract_ComputedDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_INVERSE:
	    {
		result = intract_ComputedInverseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = intract_ComputedAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = intract_ComputedBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = intract_ComputedHeuristicAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = intract_ComputedHeuristicBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = intract_ComputedHeuristicDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = intract_ComputedBitwiseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = intract_ComputedFlowSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = intract_ComputedMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = intract_ComputedHeuristicMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = intract_ComputedDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = intract_ComputedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = intract_ComputedAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = intract_ComputedGAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		result = intract_ComputedRelaxedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		result = intract_ComputedTokenMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		result = intract_ComputedTokenEmptyMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		result = intract_ComputedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		result = intract_ComputedCapacitatedPermutationMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = intract_ComputedMmddPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = intract_ComputedMmddPlusPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = intract_ComputedHeuristicDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = intract_ComputedSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = intract_ComputedHeuristicSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }
    

    sResult sMultirobotSolutionCompressor::compute_OptimalSolution(const sRobotArrangement &start_arrangement,
								   const sRobotGoal        &final_arrangement,
								   const sUndirectedGraph  &environment,
								   const sUndirectedGraph  &sparse_environment,
								   int                      makespan_lower_bound,
								   int                      makespan_upper_bound,
								   int                     &optimal_makespan,
								   sMultirobotSolution     &optimal_solution,
								   int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_OptimalMakespan(instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		result = extract_ComputedInverseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = extract_ComputedAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = extract_ComputedBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		result = extract_ComputedDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = extract_ComputedHeuristicAdvancedSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = extract_ComputedHeuristicBijectionSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = extract_ComputedHeuristicDifferentialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = extract_ComputedBitwiseSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = extract_ComputedFlowSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = extract_ComputedMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = extract_ComputedHeuristicMatchingSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = extract_ComputedDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = extract_ComputedMmddSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = extract_ComputedAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = extract_ComputedGAnoSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = extract_ComputedMmddPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = extract_ComputedMmddPlusPlusSolution(start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = extract_ComputedHeuristicDirectSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = extract_ComputedSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = extract_ComputedHeuristicSimplicialSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalSolution(Glucose::Solver         **solver,
								     const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                      makespan_lower_bound,
								     int                      makespan_upper_bound,
								     int                     &optimal_makespan,
								     sMultirobotSolution     &optimal_solution,
								     int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_OptimalMakespan(solver, instance, makespan_lower_bound, makespan_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		result = intract_ComputedInverseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = intract_ComputedAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = intract_ComputedBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		result = intract_ComputedDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = intract_ComputedHeuristicAdvancedSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = intract_ComputedHeuristicBijectionSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = intract_ComputedHeuristicDifferentialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = intract_ComputedBitwiseSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = intract_ComputedFlowSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = intract_ComputedMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = intract_ComputedHeuristicMatchingSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = intract_ComputedDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		result = intract_ComputedMmddSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		result = intract_ComputedAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		result = intract_ComputedGAnoSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_MMDD_plus:
	    {
		result = intract_ComputedMmddPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = intract_ComputedMmddPlusPlusSolution(*solver, start_arrangement, environment, instance.m_the_MDD, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = intract_ComputedHeuristicDirectSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = intract_ComputedSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = intract_ComputedHeuristicSimplicialSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/

    sResult sMultirobotSolutionCompressor::compute_OrtoOptimalMakespan(const sRobotArrangement           &start_arrangement,
								       const sRobotGoal                  &final_arrangement,
								       const sUndirectedGraph            &environment,
								       const sUndirectedGraph            &sparse_environment,
								       int                                layer_upper_bound,
								       int                               &optimal_makespan,
								       sMultirobotEncodingContext_CNFsat &final_encoding_context,
								       int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_makespan = MAKESPAN_UNDEFINED;

	int N_Layers = 1;

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

#ifdef sVERBOSE
	    printf("Solving layer %d ...\n", N_Layers);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_SINGULAR:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" +  "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_"  + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_"  + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	    
	    int preprocess_result = system(preprocess_call.c_str());
	    
	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
	    
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());
	    
	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	
	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);
	    
#ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    
	    if (strcmp(answer, "UNSAT") == 0)
	    {
#ifdef sSTATISTICS
		{
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		optimal_makespan = MAKESPAN_UNDEFINED;
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_makespan = MAKESPAN_UNDEFINED;
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		optimal_makespan = N_Layers;
		break;
	    }	    
	    if (m_encoding == ENCODING_SINGULAR)
	    {
		break;		
	    }
	    else
	    {
		if (m_encoding == ENCODING_PLURAL || m_encoding == ENCODING_PLURAL2)
		{
		    if (++N_Layers > layer_upper_bound)
		    {
			break;
		    }
		}
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OrtoOptimalMakespan(Glucose::Solver                   **solver,
									 const sRobotArrangement           &start_arrangement,
									 const sRobotGoal                  &final_arrangement,
									 const sUndirectedGraph            &environment,
									 const sUndirectedGraph            &sparse_environment,
									 int                                layer_upper_bound,
									 int                               &optimal_makespan,
									 sMultirobotEncodingContext_CNFsat &final_encoding_context,
									 int                                sUNUSED(thread_id))
    {
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_makespan = MAKESPAN_UNDEFINED;

	int N_Layers = 1;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
#ifdef sVERBOSE
	    printf("Solving layer %d ...\n", N_Layers);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_SINGULAR:
	    {
		instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		rlimit rl;
		getrlimit(RLIMIT_CPU, &rl);
		
		if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
		{
		    rl.rlim_cur = m_minisat_timeout;
		    
		    if (setrlimit(RLIMIT_CPU, &rl) == -1)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		    }
		}
	    } 
	    
	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		optimal_makespan = N_Layers;
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		optimal_makespan = MAKESPAN_UNDEFINED;		
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif		
		optimal_makespan = MAKESPAN_UNDEFINED;		
	    }	    
	    if (m_encoding == ENCODING_SINGULAR)
	    {
		break;		
	    }
	    else
	    {
		if (m_encoding == ENCODING_PLURAL || m_encoding == ENCODING_PLURAL2)
		{
		    if (++N_Layers > layer_upper_bound)
		    {
			break;
		    }
		}
	    }
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_OrtoOptimalSolution(const sRobotArrangement &start_arrangement,
								       const sRobotGoal        &final_arrangement,
								       const sUndirectedGraph  &environment,
								       const sUndirectedGraph  &sparse_environment,
								       int                      layer_upper_bound,
								       int                     &optimal_makespan,
								       sMultirobotSolution     &optimal_solution,
								       int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = compute_OrtoOptimalMakespan(start_arrangement, final_arrangement, environment, sparse_environment, layer_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OrtoOptimalSolution(Glucose::Solver         **solver,
									 const sRobotArrangement &start_arrangement,
									 const sRobotGoal        &final_arrangement,
									 const sUndirectedGraph  &environment,
									 const sUndirectedGraph  &sparse_environment,
									 int                      layer_upper_bound,
									 int                     &optimal_makespan,
									 sMultirobotSolution     &optimal_solution,
									 int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = incompute_OrtoOptimalMakespan(solver, start_arrangement, final_arrangement, environment, sparse_environment, layer_upper_bound, optimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, optimal_makespan, final_encoding_context, optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalCost(sMultirobotInstance               &instance,
							       int                                max_total_cost,
							       int                               &optimal_cost,
							       int                               &expansion_count,
							       sMultirobotEncodingContext_CNFsat &final_encoding_context,
							       int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	expansion_count = 0;

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;	    

#ifdef sVERBOSE
	    printf("Solving cost %d (%d) ...\n", total_cost, max_individual_cost);
#endif

	    if (sFAILED(result = compute_CostSolvability(instance, total_cost, final_encoding_context, thread_id)))
	    {
		return result;
	    }
	    
	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		optimal_cost = total_cost;
		return sRESULT_SUCCESS;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    {
		break;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		optimal_cost = MAKESPAN_UNDEFINED;
		return result;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    total_cost += 1;
	    
	    if (total_cost > max_total_cost)
	    {
		break;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }

	    ++expansion_count;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalCost(Glucose::Solver                   **solver,
								 sMultirobotInstance               &instance,
								 int                                max_total_cost,
								 int                               &optimal_cost,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	expansion_count = 0;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;

	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;	    

#ifdef sVERBOSE
	    printf("Solving cost %d (%d) ...\n", total_cost, max_individual_cost);
#endif

	    if (sFAILED(result = incompute_CostSolvability(solver, instance, total_cost, final_encoding_context, thread_id)))
	    {
		return result;
	    }
	    
	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		optimal_cost = total_cost;
		return sRESULT_SUCCESS;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    {
		break;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		optimal_cost = MAKESPAN_UNDEFINED;
		return result;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    total_cost += 1;
	    
	    if (total_cost > max_total_cost)
	    {
		break;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }

	    ++expansion_count;
	}
	return sRESULT_SUCCESS;
    }
    
    
    sResult sMultirobotSolutionCompressor::compute_OptimalCost(const sRobotArrangement           &start_arrangement,
							       const sRobotGoal                  &final_arrangement,
							       const sUndirectedGraph            &environment,
							       const sUndirectedGraph            &sparse_environment,
							       int                                max_total_cost,
							       int                               &optimal_cost,
							       int                               &expansion_count,
							       sMultirobotEncodingContext_CNFsat &final_encoding_context,
							       int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalCost(instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalCost(Glucose::Solver                   **solver,
								 const sRobotArrangement           &start_arrangement,
								 const sRobotGoal                  &final_arrangement,
								 const sUndirectedGraph            &environment,
								 const sUndirectedGraph            &sparse_environment,
								 int                                max_total_cost,
								 int                               &optimal_cost,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalCost(solver, instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }    

/*
    sResult sMultirobotSolutionCompressor::incompute_OptimalCost(Glucose::Solver                   **solver,
								 sMultirobotInstance               &instance,
								 int                                max_total_cost,
								 int                               &optimal_cost,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	expansion_count = 0;	

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;

#ifdef sVERBOSE
	    printf("Solving cost %d (%d) ...\n", total_cost, max_individual_cost);
#endif

	    if (sFAILED(result = incompute_CostSolvability(solver, instance, total_cost, final_encoding_context, thread_id)))
	    {
		return result;
	    }
	    
	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		optimal_cost = total_cost;
		return sRESULT_SUCCESS;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    {
		break;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		optimal_cost = MAKESPAN_UNDEFINED;
		return result;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }

	    total_cost += 1;
	    
	    if (total_cost > max_total_cost)
	    {
		break;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }

	    ++expansion_count;
	}
	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_OptimalCost(Glucose::Solver                   **solver,
								 const sRobotArrangement           &start_arrangement,
								 const sRobotGoal                  &final_arrangement,
								 const sUndirectedGraph            &environment,
								 const sUndirectedGraph            &sparse_environment,
								 int                                max_total_cost,
								 int                               &optimal_cost,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalCost(solver, instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }
*/


    sResult sMultirobotSolutionCompressor::compute_OptimalFuel(sMultirobotInstance               &instance,
							       int                                max_total_fuel,
							       int                               &optimal_fuel,
							       int                               &fuel_makespan,
							       int                               &expansion_count,
							       sMultirobotEncodingContext_CNFsat &final_encoding_context,
							       int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_fuel = MAKESPAN_UNDEFINED;

	int max_individual_fuel;
	int total_fuel = instance.estimate_TotalFuel(max_individual_fuel);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	expansion_count = 0;

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    
	    encoding_context.m_max_total_fuel = total_fuel;

#ifdef sVERBOSE
		printf("Solving fuel %d (individual = %d) ...\n", total_fuel, max_individual_fuel);
#endif	    

	    for (int fuel_makespan_ = max_individual_fuel; fuel_makespan_ <= encoding_context.m_max_total_fuel; ++fuel_makespan_)
	    {
		encoding_context.m_fuel_makespan = fuel_makespan_;

#ifdef sVERBOSE
		printf("  Checking makespan = %d ...\n", fuel_makespan_);
#endif	    		
	    
		if (sFAILED(result = compute_FuelSolvability(instance, total_fuel, fuel_makespan_, final_encoding_context, thread_id)))
		{
		    return result;
		}
		
		switch (result)
		{
		case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
		{
		    optimal_fuel = total_fuel;
		    fuel_makespan = fuel_makespan_;
		    return sRESULT_SUCCESS;
		}
		case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
		{
		    break;
		}
		case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
		{
		    optimal_fuel = MAKESPAN_UNDEFINED;
		    fuel_makespan = MAKESPAN_UNDEFINED;
		    return result;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}

		finish_seconds = sGet_CPU_Seconds();
		
		if (finish_seconds - start_seconds  > m_total_timeout)
		{
		    break;
		}
		
	    }
	    ++expansion_count;			    
	    total_fuel += 1;
	    
	    if (total_fuel > max_total_fuel)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalFuel(Glucose::Solver                   **solver,
								 sMultirobotInstance               &instance,
								 int                                max_total_fuel,
								 int                               &optimal_fuel,
								 int                               &fuel_makespan,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_fuel = MAKESPAN_UNDEFINED;

	int max_individual_fuel;
	int total_fuel = instance.estimate_TotalFuel(max_individual_fuel);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	expansion_count = 0;

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_fuel = total_fuel;

#ifdef sVERBOSE
		printf("Solving fuel %d (individual = %d) ...\n", total_fuel, max_individual_fuel);
#endif	    

	    for (int fuel_makespan_ = max_individual_fuel; fuel_makespan_ <= encoding_context.m_max_total_fuel; ++fuel_makespan_)
	    {
		if (*solver != NULL)
		{
		    delete *solver;
		}
		*solver = new Glucose::Solver;
		
		encoding_context.m_fuel_makespan = fuel_makespan_;

#ifdef sVERBOSE
		printf("  Checking makespan = %d ...\n", fuel_makespan_);
#endif	    		

		if (sFAILED(result = incompute_FuelSolvability(solver, instance, total_fuel, fuel_makespan_, final_encoding_context, thread_id)))
		{
		    return result;
		}
		
		switch (result)
		{
		case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
		{
		    optimal_fuel = total_fuel;
		    fuel_makespan = fuel_makespan_;
		    return sRESULT_SUCCESS;
		}
		case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
		{
		    break;
		}
		case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
		{
		    optimal_fuel = MAKESPAN_UNDEFINED;
		    fuel_makespan = MAKESPAN_UNDEFINED;
		    return result;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}

		finish_seconds = sGet_CPU_Seconds();
		
		if (finish_seconds - start_seconds  > m_total_timeout)
		{
		    return sRESULT_SUCCESS;
		}
	    }
	    ++expansion_count;				
	    total_fuel += 1;
		
	    if (total_fuel > max_total_fuel)
	    {
		break;
	    }		    
	}
	return sRESULT_SUCCESS;
    }
    
    
    sResult sMultirobotSolutionCompressor::compute_OptimalFuel(const sRobotArrangement           &start_arrangement,
							       const sRobotGoal                  &final_arrangement,
							       const sUndirectedGraph            &environment,
							       const sUndirectedGraph            &sparse_environment,
							       int                                max_total_fuel,
							       int                               &optimal_fuel,
							       int                               &fuel_makespan,
							       int                               &expansion_count,
							       sMultirobotEncodingContext_CNFsat &final_encoding_context,
							       int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalFuel(instance, max_total_fuel, optimal_fuel, fuel_makespan, expansion_count, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalFuel(Glucose::Solver                   **solver,
								 const sRobotArrangement           &start_arrangement,
								 const sRobotGoal                  &final_arrangement,
								 const sUndirectedGraph            &environment,
								 const sUndirectedGraph            &sparse_environment,
								 int                                max_total_fuel,
								 int                               &optimal_fuel,
								 int                               &fuel_makespan,
								 int                               &expansion_count,
								 sMultirobotEncodingContext_CNFsat &final_encoding_context,
								 int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalFuel(solver, instance, max_total_fuel, optimal_fuel, fuel_makespan, expansion_count, final_encoding_context, thread_id)))
	{
	    return result;
	}

	return sRESULT_SUCCESS;
    }    
    

    sResult sMultirobotSolutionCompressor::compute_OptimalCost_avoid(const sRobotArrangement           &start_arrangement,
								     const sRobotGoal                  &final_arrangement,
								     const sUndirectedGraph            &environment,
								     const sUndirectedGraph            &sparse_environment,
								     const Arrangements_vector         &blocking_solution,
								     int                                optimal_cost,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalCost_avoid(instance, blocking_solution, optimal_cost, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
	
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalCost_avoid(Glucose::Solver                   **solver,
								       const sRobotArrangement           &start_arrangement,
								       const sRobotGoal                  &final_arrangement,
								       const sUndirectedGraph            &environment,
								       const sUndirectedGraph            &sparse_environment,
								       const Arrangements_vector         &blocking_solution,
								       int                                optimal_cost,
								       sMultirobotEncodingContext_CNFsat &final_encoding_context,
								       int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalCost_avoid(solver, instance, blocking_solution, optimal_cost, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
	
    }    

    
    sResult sMultirobotSolutionCompressor::compute_OptimalCost_avoid(sMultirobotInstance               &instance,
								     const Arrangements_vector         &blocking_solution,
								     int                                optimal_cost,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = optimal_cost;
	encoding_context.m_max_total_fuel = optimal_cost;	

#ifdef sVERBOSE
	printf("Solving avoiding for cost %d ...\n", optimal_cost);
#endif

	if (sFAILED(result = compute_CostSolvability_avoid(instance, optimal_cost, blocking_solution, final_encoding_context, thread_id)))
	{
	    return result;
	}
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	{
	    return result;
	}	
	default:
	{
	    sASSERT(false);
	    break;
	}
	}	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalCost_avoid(Glucose::Solver                   **solver,
								       sMultirobotInstance               &instance,
								       const Arrangements_vector         &blocking_solution,
								       int                                optimal_cost,
								       sMultirobotEncodingContext_CNFsat &final_encoding_context,
								       int                                thread_id)
    {
	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = optimal_cost;
	encoding_context.m_max_total_fuel = optimal_cost;	

#ifdef sVERBOSE
	printf("Solving avoiding for cost %d ...\n", optimal_cost);
#endif

	if (sFAILED(result = incompute_CostSolvability_avoid(solver, instance, optimal_cost, blocking_solution, final_encoding_context, thread_id)))
	{	    
	    return result;
	}
		
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	{
	    return result;
	}	
	default:
	{
	    sASSERT(false);
	    break;
	}
	}	
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_avoid(const sRobotArrangement           &start_arrangement,
									 const sRobotGoal                  &final_arrangement,
									 const sUndirectedGraph            &environment,
									 const sUndirectedGraph            &sparse_environment,
									 const Arrangements_vector         &blocking_solution,
									 int                                optimal_makespan,
									 sMultirobotEncodingContext_CNFsat &final_encoding_context,
									 int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = compute_OptimalMakespan_avoid(instance, blocking_solution, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_avoid(Glucose::Solver                   **solver,
									   const sRobotArrangement           &start_arrangement,
									   const sRobotGoal                  &final_arrangement,
									   const sUndirectedGraph            &environment,
									   const sUndirectedGraph            &sparse_environment,
									   const Arrangements_vector         &blocking_solution,
									   int                                optimal_makespan,
									   sMultirobotEncodingContext_CNFsat &final_encoding_context,
									   int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (sFAILED(result = incompute_OptimalMakespan_avoid(solver, instance, blocking_solution, optimal_makespan, final_encoding_context, thread_id)))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }
    
    
    sResult sMultirobotSolutionCompressor::compute_OptimalMakespan_avoid(sMultirobotInstance               &instance,
									 const Arrangements_vector         &blocking_solution,
									 int                                optimal_makespan,
									 sMultirobotEncodingContext_CNFsat &final_encoding_context,
									 int                                thread_id)
    {
	sResult result;	
	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(optimal_makespan);

#ifdef sVERBOSE
	printf("Solving avoiding for makespan %d ...\n", optimal_makespan);
#endif

	if (sFAILED(result = compute_MakespanSolvability_avoid(instance, optimal_makespan, blocking_solution, final_encoding_context, thread_id)))
	{
	    return result;
	}
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	{
	    return result;
	}	
	default:
	{
	    sASSERT(false);
	    break;
	}
	}	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_OptimalMakespan_avoid(Glucose::Solver                   **solver,
									   sMultirobotInstance               &instance,
									   const Arrangements_vector         &blocking_solution,
									   int                                optimal_makespan,
									   sMultirobotEncodingContext_CNFsat &final_encoding_context,
									   int                                thread_id)
    {
	sResult result;	
	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(optimal_makespan);

#ifdef sVERBOSE
	printf("Solving avoiding for makespan %d ...\n", optimal_makespan);
#endif

	if (sFAILED(result = incompute_MakespanSolvability_avoid(solver, instance, optimal_makespan, blocking_solution, final_encoding_context, thread_id)))
	{
	    return result;
	}
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return result;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	{
	    return result;
	}	
	default:
	{
	    sASSERT(false);
	    break;
	}
	}	
	return sRESULT_SUCCESS;
    }            


    sResult sMultirobotSolutionCompressor::compute_CostSolvability(sMultirobotInstance               &instance,
								   int                                total_cost,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;

	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_max_total_fuel = total_cost;	

#ifdef sVERBOSE
	printf("Solving cost %d ...\n", total_cost);
#endif

	switch (m_encoding)
	{
	case ENCODING_HEIGHTED:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_HEIGHTED_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_HeightedCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD:	    
	case ENCODING_ID_MDD:
	case ENCODING_AD_MDD:
	case ENCODING_BMDD:
	case ENCODING_BCMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_UMTEX:	    
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddUmtexCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_MUTEX:	    
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddMutexCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}		
	case ENCODING_GMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_GMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_GEMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_GEMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_WATER_MDD:
	case ENCODING_ID_WATER_MDD:
	case ENCODING_AD_WATER_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_WaterMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_RELAXED_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_RelaxedMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_TokenMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_TokenEmptyMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_PERMUTATION_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_PermutationMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_PERMUTATION_CMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_CapacitatedPermutationMddCNFsat(cnf_filename, encoding_context, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}				
	case ENCODING_MDD_plus:
	case ENCODING_ID_MDD_plus:
	case ENCODING_AD_MDD_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus:	    
	case ENCODING_ID_MDD_plus_plus:
	case ENCODING_AD_MDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus_mutex:	    
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddPlusPlusMutexCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_LMDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_LMddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus_fuel:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddPlusPlusFuelCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}		
	case ENCODING_MDD_star:
	case ENCODING_ID_MDD_star:
	case ENCODING_AD_MDD_star:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddStarCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	    	    	
	case ENCODING_RXMDD:
	case ENCODING_RXMDD_BINARY:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_RXMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_NOMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_NoMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_RXNOMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_RXNoMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
	
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}

	//	    s_GlobalPhaseStatistics.enter_Phase("SAT Solving");
	int system_result = system(system_call.c_str());
	//	    s_GlobalPhaseStatistics.leave_Phase();
	
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	    
#ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
//      s_GlobalPhaseStatistics.leave_Phase();
#endif
	
	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);
	    
#ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif  
	if (strcmp(answer, "UNSAT") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else if (strcmp(answer, "INDET") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	else /*if (strcmp(answer, "SAT") == 0)*/
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;
	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostSolvability(Glucose::Solver                   **solver,
								     sMultirobotInstance               &instance,
								     int                                total_cost,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                sUNUSED(thread_id))
    {
	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_max_total_fuel = total_cost;		

#ifdef sVERBOSE
	printf("Solving cost %d ...\n", total_cost);
#endif

	switch (m_encoding)
	{
	case ENCODING_HEIGHTED:
	{
	    instance.to_Memory_HeightedCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD:
	case ENCODING_ID_MDD:
	case ENCODING_AD_MDD:
	case ENCODING_BMDD:
	case ENCODING_BCMDD:
	{
	    instance.to_Memory_MddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_UMTEX:
	{
	    instance.to_Memory_MddUmtexCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_MUTEX:
	{
	    instance.to_Memory_MddMutexCNFsat(*solver, encoding_context, "", false);
	    break;
	}		
	case ENCODING_GMDD:
	{
	    instance.to_Memory_GMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_GEMDD:
	{
	    instance.to_Memory_GEMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_WATER_MDD:
	case ENCODING_ID_WATER_MDD:
	case ENCODING_AD_WATER_MDD:
	{
	    instance.to_Memory_WaterMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_RELAXED_MDD:
	{
	    instance.to_Memory_RelaxedMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_MDD:
	{
	    instance.to_Memory_TokenMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MDD:
	{
	    instance.to_Memory_TokenEmptyMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_PERMUTATION_MDD:
	{
	    instance.to_Memory_PermutationMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_PERMUTATION_CMDD:
	{
	    instance.to_Memory_CapacitatedPermutationMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}				
	case ENCODING_MDD_plus:
	case ENCODING_ID_MDD_plus:
	case ENCODING_AD_MDD_plus:
	{
	    instance.to_Memory_MddPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus:
	case ENCODING_ID_MDD_plus_plus:
	case ENCODING_AD_MDD_plus_plus:
	{
	    instance.to_Memory_MddPlusPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus_mutex:
	{
	    instance.to_Memory_MddPlusPlusMutexCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_LMDD_plus_plus:
	{
	    instance.to_Memory_LMddPlusPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus_fuel:
	{
	    instance.to_Memory_MddPlusPlusFuelCNFsat(*solver, encoding_context, "", false);
	    break;
	}		
	case ENCODING_MDD_star:
	case ENCODING_ID_MDD_star:
	case ENCODING_AD_MDD_star:
	{
	    instance.to_Memory_MddStarCNFsat(*solver, encoding_context, "", false);
	    break;
	}	    	    	
	case ENCODING_RXMDD:
	case ENCODING_RXMDD_BINARY:
	{
	    instance.to_Memory_RXMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_NOMDD:
	{
	    instance.to_Memory_NoMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_RXNOMDD:
	{
	    instance.to_Memory_RXNoMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}

	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	
	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);
	
	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compute_FuelSolvability(sMultirobotInstance               &instance,
								   int                                total_fuel,
								   int                                fuel_makespan,
								   sMultirobotEncodingContext_CNFsat &final_encoding_context,
								   int                                thread_id)
    {
	sResult result;

	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_fuel = total_fuel;
	encoding_context.m_fuel_makespan = fuel_makespan;	
	
	switch (m_encoding)
	{
	case ENCODING_MDD_plus_plus_fuel:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_fuel) + "-" + sInt_32_to_String(fuel_makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    result = instance.to_File_MddPlusPlusFuelCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}		
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
	
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}

	//	    s_GlobalPhaseStatistics.enter_Phase("SAT Solving");
	int system_result = system(system_call.c_str());
	//	    s_GlobalPhaseStatistics.leave_Phase();
	
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	    
#ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
//      s_GlobalPhaseStatistics.leave_Phase();
#endif
	
	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);
	    
#ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif  
	if (strcmp(answer, "UNSAT") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else if (strcmp(answer, "INDET") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	else /*if (strcmp(answer, "SAT") == 0)*/
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;
	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_FuelSolvability(Glucose::Solver                   **solver,
								     sMultirobotInstance               &instance,
								     int                                total_fuel,
								     int                                fuel_makespan,
								     sMultirobotEncodingContext_CNFsat &final_encoding_context,
								     int                                sUNUSED(thread_id))
    {
	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_fuel = total_fuel;
	encoding_context.m_fuel_makespan = fuel_makespan;

	switch (m_encoding)
	{
	case ENCODING_MDD_plus_plus_fuel:
	{
	    instance.to_Memory_MddPlusPlusFuelCNFsat(*solver, encoding_context, "", false);
	    break;
	}		
	default:
	{
	    sASSERT(false);
	}
	}

	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	
	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);
	
	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	}
	return sRESULT_SUCCESS;
    }
    

    sResult sMultirobotSolutionCompressor::compute_CostSolvability_avoid(sMultirobotInstance               &instance,
									 int                                total_cost,
									 const Arrangements_vector         &blocking_solution,									 
									 sMultirobotEncodingContext_CNFsat &final_encoding_context,
									 int                                thread_id)
    {
	sResult result;

	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_max_total_fuel = total_cost;		

#ifdef sVERBOSE
	printf("Solving avoid cost %d ...\n", total_cost);
#endif

	switch (m_encoding)
	{
	case ENCODING_AD_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_WATER_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_WaterMddCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_AD_MDD_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddPlusCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_MDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddPlusPlusCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_MDD_star:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MddStarCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}			
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
	
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}

	//	    s_GlobalPhaseStatistics.enter_Phase("SAT Solving");
	int system_result = system(system_call.c_str());
	//	    s_GlobalPhaseStatistics.leave_Phase();
	
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	    
#ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
//      s_GlobalPhaseStatistics.leave_Phase();
#endif
	
	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);
	    
#ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif  
	if (strcmp(answer, "UNSAT") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else if (strcmp(answer, "INDET") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	else /*if (strcmp(answer, "SAT") == 0)*/
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;
	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostSolvability_avoid(Glucose::Solver                   **solver,
									   sMultirobotInstance               &instance,
									   int                                total_cost,
									   const Arrangements_vector         &blocking_solution,									 
									   sMultirobotEncodingContext_CNFsat &final_encoding_context,
									   int                                sUNUSED(thread_id))
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat encoding_context(0);
	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_max_total_fuel = total_cost;		

#ifdef sVERBOSE
	printf("Solving avoid cost %d ...\n", total_cost);
#endif

	switch (m_encoding)
	{
	case ENCODING_AD_MDD:
	{
	    result = instance.to_Memory_MddCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_WATER_MDD:
	{
	    result = instance.to_Memory_WaterMddCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_AD_MDD_plus:
	{
	    result = instance.to_Memory_MddPlusCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_MDD_plus_plus:
	{
	    result = instance.to_Memory_MddPlusPlusCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_AD_MDD_star:
	{
	    result = instance.to_Memory_MddStarCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}			
	default:
	{
	    sASSERT(false);
	}
	}

	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	
	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);
	
	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	}
	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotSolutionCompressor::compute_MakespanSolvability_avoid(sMultirobotInstance               &instance,
									     int                                makespan,
									     const Arrangements_vector         &blocking_solution,
									     sMultirobotEncodingContext_CNFsat &final_encoding_context,
									     int                                thread_id)
    {
	sResult result;

	sString cnf_filename, cnf_out_filename, output_filename;

	sMultirobotEncodingContext_CNFsat encoding_context(makespan);

#ifdef sVERBOSE
	printf("Solving avoid makespan %d ...\n", makespan);
#endif

	switch (m_encoding)
	{
	case ENCODING_MMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MmddCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_MMDD_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MmddPlusCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MMDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(makespan) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    //		s_GlobalPhaseStatistics.enter_Phase("CNF generation");
	    result = instance.to_File_MmddPlusPlusCNFsat_avoid(cnf_filename, encoding_context, blocking_solution, "", false);
	    //		s_GlobalPhaseStatistics.leave_Phase();
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}		
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
	
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}

	//	    s_GlobalPhaseStatistics.enter_Phase("SAT Solving");
	int system_result = system(system_call.c_str());
	//	    s_GlobalPhaseStatistics.leave_Phase();
	
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	    
#ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
//      s_GlobalPhaseStatistics.leave_Phase();
#endif
	
	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);
	    
#ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif  
	if (strcmp(answer, "UNSAT") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else if (strcmp(answer, "INDET") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
		
#ifndef sDEBUG
	    {
		if (unlink(output_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	else /*if (strcmp(answer, "SAT") == 0)*/
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;
	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}	
	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_MakespanSolvability_avoid(Glucose::Solver                   **solver,
									       sMultirobotInstance               &instance,
									       int                                makespan,
									       const Arrangements_vector         &blocking_solution,
									       sMultirobotEncodingContext_CNFsat &final_encoding_context,
									       int                                sUNUSED(thread_id))
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat encoding_context(makespan);

#ifdef sVERBOSE
	printf("Solving avoid makespan %d ...\n", makespan);
#endif

	switch (m_encoding)
	{
	case ENCODING_MMDD:
	{	    
	    result = instance.to_Memory_MmddCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_MMDD_plus:
	{
	    result = instance.to_Memory_MmddPlusCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MMDD_plus_plus:
	{
	    result = instance.to_Memory_MmddPlusPlusCNFsat_avoid(*solver, encoding_context, blocking_solution, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	    {
		return result;
	    }
	    break;
	}		
	default:
	{
	    sASSERT(false);
	}
	}

	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	
	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);
	
	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    final_encoding_context = encoding_context;	    
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;	    
	}	
	
	return sRESULT_SUCCESS;
    }        
    

    sResult sMultirobotSolutionCompressor::incompute_OptimalCost_binary(Glucose::Solver                   **solver,
									const sRobotArrangement           &start_arrangement,
									const sRobotGoal                  &final_arrangement,
									sUndirectedGraph                  &environment,
									const sUndirectedGraph            &sparse_environment,
									int                                max_total_cost,
									int                               &optimal_cost,
									sMultirobotEncodingContext_CNFsat &final_encoding_context,
									int                                thread_id)
    {
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	sMultirobotEncodingContext_CNFsat upper_encoding_context;

	int upper_bound, lower_bound;

	lower_bound = total_cost;

//	s_GlobalPhaseStatistics.enter_Phase("Upper");
	incompute_UpperCost(solver,
			    start_arrangement,
			    final_arrangement,
			    environment,
			    sparse_environment,
			    max_total_cost,
			    upper_bound,
			    upper_encoding_context,
			    thread_id);
//	s_GlobalPhaseStatistics.leave_Phase();
//	bool binary_phase = false;//false;

	int distance = upper_bound - lower_bound;
	int step = distance / 4;
//	int step = sqrt(distance);

	bool final_phase = false;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    printf("Lower bound: %d\n", lower_bound);
	    printf("Upper bound: %d\n", upper_bound);
	    printf("Total total: %d\n", upper_encoding_context.m_max_total_cost);

	    if (final_phase)
	    {
		total_cost = lower_bound + 1;
	    }
	    else
	    {
		total_cost = lower_bound + step;
		step = step / 2;

		if (step == 1)
		{
		    final_phase = true;
		}
	    }
//	    binary_phase = !binary_phase;
	    printf("Total cost:%d\n", total_cost);

	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;		    

#ifdef sVERBOSE
	    printf("Solving cost %d ...\n", total_cost);
#endif	    
	    switch (m_encoding)
	    {
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {		
		instance.to_Memory_MddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_RXNOMDD:
	    {
		instance.to_Memory_MddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }
	    
	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		if (final_phase)
		{
		    final_encoding_context = encoding_context;
		    optimal_cost = total_cost;
		    return sRESULT_SUCCESS;
		}
		else
		{
		    final_phase = true;
		}
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		lower_bound = total_cost;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compute_OptimalCost_binary(const sRobotArrangement           &start_arrangement,
								      const sRobotGoal                  &final_arrangement,
								      sUndirectedGraph                  &environment,
								      const sUndirectedGraph            &sparse_environment,
								      int                                max_total_cost,
								      int                               &optimal_cost,
								      sMultirobotEncodingContext_CNFsat &final_encoding_context,
								      int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	sMultirobotEncodingContext_CNFsat upper_encoding_context;

	int upper_bound, lower_bound;

	lower_bound = total_cost;

//	s_GlobalPhaseStatistics.enter_Phase("Upper");
	compute_UpperCost(start_arrangement,
			  final_arrangement,
			  environment,
			  sparse_environment,
			  max_total_cost,
			  upper_bound,
			  upper_encoding_context,
			  thread_id);
//	s_GlobalPhaseStatistics.leave_Phase();
//	bool binary_phase = false;//false;

	int distance = upper_bound - lower_bound;
	int step = distance / 4;
//	int step = sqrt(distance);

	bool final_phase = false;

	while (true)
	{
	    printf("Lower bound: %d\n", lower_bound);
	    printf("Upper bound: %d\n", upper_bound);
	    printf("Total total: %d\n", upper_encoding_context.m_max_total_cost);

	    if (final_phase)
	    {
		total_cost = lower_bound + 1;
	    }
	    else
	    {
		total_cost = lower_bound + step;
		step = step / 2;

		if (step == 1)
		{
		    final_phase = true;
		}
	    }
//	    binary_phase = !binary_phase;
	    printf("Total cost:%d\n", total_cost);

	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;		    

#ifdef sVERBOSE
	    printf("Solving cost %d ...\n", total_cost);
#endif	    
	    switch (m_encoding)
	    {
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_MddCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_RXNOMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_MddCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	    
	    int preprocess_result = system(preprocess_call.c_str());
	    
	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
	    
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());
	    
	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	
	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);
	    
#ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif  
	    if (strcmp(answer, "UNSAT") == 0)
	    {
#ifdef sSTATISTICS
		{
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		lower_bound = total_cost;
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
#endif
		if (final_phase)
		{
		    final_encoding_context = encoding_context;
		    optimal_cost = total_cost;
		    return sRESULT_SUCCESS;
		}
		else
		{
		    final_phase = true;
		}
	    }
//	    total_cost += 30;

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }
    
/*
    sResult sMultirobotSolutionCompressor::incompute_OptimalCost_binary(Glucose::Solver                   **solver,
									const sRobotArrangement           &start_arrangement,
									const sRobotGoal                  &final_arrangement,
									sUndirectedGraph                  &environment,
									const sUndirectedGraph            &sparse_environment,
									int                                max_total_cost,
									int                               &optimal_cost,
									sMultirobotEncodingContext_CNFsat &final_encoding_context,
									int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	sMultirobotEncodingContext_CNFsat upper_encoding_context;

	int upper_bound, lower_bound;

	lower_bound = total_cost;

//	s_GlobalPhaseStatistics.enter_Phase("Upper");
	incompute_UpperCost(solver,
			    start_arrangement,
			    final_arrangement,
			    environment,
			    sparse_environment,
			    max_total_cost,
			    upper_bound,
			    upper_encoding_context,
			    thread_id);
//	s_GlobalPhaseStatistics.leave_Phase();
//	bool binary_phase = false;//false;

	int distance = upper_bound - lower_bound;
	int step = distance / 4;
//	int step = sqrt(distance);

	bool final_phase = false;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    printf("Lower bound: %d\n", lower_bound);
	    printf("Upper bound: %d\n", upper_bound);
	    printf("Total total: %d\n", upper_encoding_context.m_max_total_cost);

	    if (final_phase)
	    {
		total_cost = lower_bound + 1;
	    }
	    else
	    {
		total_cost = lower_bound + step;
		step = step / 2;

		if (step == 1)
		{
		    final_phase = true;
		}
	    }
//	    binary_phase = !binary_phase;
	    printf("Total cost:%d\n", total_cost);

	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;

#ifdef sVERBOSE
	    printf("Solving cost %d ...\n", total_cost);
#endif	    
	    switch (m_encoding)
	    {
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		instance.to_Memory_MddCNFsat(solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_RXNOMDD:
	    {		
		instance.to_Memory_MddCNFsat(solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }
	    
	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		if (final_phase)
		{
		    final_encoding_context = encoding_context;
		    optimal_cost = total_cost;
		    return sRESULT_SUCCESS;
		}
		else
		{
		    final_phase = true;
		}
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		lower_bound = total_cost;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }
*/        

    sResult sMultirobotSolutionCompressor::compute_CostSolvability(sMultirobotInstance               &instance,
								   int                                total_cost,
								   int                                extra_cost,
								   Solvability                       &solvability,
								   sMultirobotSolution               &solution,
								   sMultirobotEncodingContext_CNFsat &encoding_context,
								   int                                thread_id)
    {
	printf("Solvability for: %d,%d\n", total_cost, extra_cost);

	sResult result;
	sString cnf_filename, cnf_out_filename, output_filename;

	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_extra_cost = extra_cost;

	encoding_context.m_max_total_fuel = total_cost;
	encoding_context.m_extra_fuel = extra_cost;	

	switch (m_encoding)
	{
	case ENCODING_MDD:
	case ENCODING_ID_MDD:
	case ENCODING_AD_MDD:
	case ENCODING_BMDD:
	case ENCODING_BCMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_UMTEX:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_UMTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddUmtexCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_MUTEX:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_MUTEX_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddMutexCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}			
	case ENCODING_GMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_GMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_GMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_GEMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_GEMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_GEMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_WATER_MDD:
	case ENCODING_ID_WATER_MDD:
	case ENCODING_AD_WATER_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_WATER_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_WaterMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_RELAXED_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_RELAXED_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_RelaxedMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_TokenMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_TokenEmptyMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_PERMUTATION_MDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_PermutationMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_PERMUTATION_CMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_CapacitatedPermutationMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}				
	case ENCODING_MDD_plus:
	case ENCODING_ID_MDD_plus:
	case ENCODING_AD_MDD_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus:
	case ENCODING_ID_MDD_plus_plus:
	case ENCODING_AD_MDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus_mutex:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddPlusPlusMutexCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}	
	case ENCODING_LMDD_plus_plus:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_LMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_LMddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	case ENCODING_MDD_plus_plus_fuel:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddPlusPlusFuelCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}		
	case ENCODING_MDD_star:
	case ENCODING_ID_MDD_star:
	case ENCODING_AD_MDD_star:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_MDD_star_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_MddStarCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}			
	case ENCODING_NOMDD:
	case ENCODING_BNOMDD:
	case ENCODING_BCNOMDD:
	{
	    if (thread_id != THREAD_ID_UNDEFINED)
	    {
		cnf_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		cnf_out_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
	    }
	    else
	    {
		cnf_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		cnf_out_filename = CNF_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		output_filename = OUTPUT_NOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
	    }
	    
	    result = instance.to_File_NoMddCNFsat(cnf_filename, encoding_context, "", false);
	    
	    if (sFAILED(result))
	    {
		return result;
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}
	
#ifdef PREPROCESS
	sString preprocess_call;
	preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	
	int preprocess_result = system(preprocess_call.c_str());
	
	if (preprocess_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
	FILE *fro;
	if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	{
	    cnf_out_filename = cnf_filename;
	}
	else
	{
	    fclose(fro);
	}
#else
	cnf_out_filename = cnf_filename;
#endif
	
	sString system_call;
	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
	else
	{
	    system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	}
//	s_GlobalPhaseStatistics.enter_Phase("SAT Search");
	int system_result = system(system_call.c_str());
//	s_GlobalPhaseStatistics.leave_Phase();
		
	if (system_result < 0)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	}
		
#ifdef sSTATISTICS
	{
	    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	}
#endif
		
	FILE *fr;
	if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	}
	
	char answer[32];
	answer[0] = '\0';
	
	fscanf(fr, "%s\n", answer);
	fclose(fr);
		
#ifndef sDEBUG
	{
	    if (unlink(cnf_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif  
	if (strcmp(answer, "UNSAT") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
 	    solvability = SOLVABILITY_UNSAT;
	}
	else if (strcmp(answer, "INDET") == 0)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    solvability = SOLVABILITY_INDET; 
	}
	else /*if (strcmp(answer, "SAT") == 0) */
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
	    }
#endif
	    sMultirobotInstance::MDD_vector MDD, extra_MDD;		

	    switch (m_encoding)
	    {
	    case ENCODING_MDD:
	    case ENCODING_ID_MDD:
	    case ENCODING_AD_MDD:		
	    case ENCODING_BMDD:
	    case ENCODING_BCMDD:
	    {
		extract_ComputedMddSolution(instance.m_initial_arrangement,
					    instance.m_environment,
					    instance.m_the_MDD,
					    total_cost,
					    encoding_context,
					    solution,
					    thread_id);
		break;
	    }
	    case ENCODING_MDD_UMTEX:
	    {
		extract_ComputedMddUmtexSolution(instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution,
						 thread_id);
		break;
	    }
	    case ENCODING_MDD_MUTEX:
	    {
		extract_ComputedMddMutexSolution(instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution,
						 thread_id);
		break;
	    }	    	    
	    case ENCODING_GMDD:
	    {
		extract_ComputedGMddSolution(instance.m_initial_arrangement,
					     instance.m_environment,
					     instance.m_the_MDD,
					     total_cost,
					     encoding_context,
					     solution,
					     thread_id);
		break;
	    }
	    case ENCODING_GEMDD:
	    {
		extract_ComputedGEMddSolution(instance.m_initial_arrangement,
					      instance.m_environment,
					      instance.m_the_MDD,
					      total_cost,
					      encoding_context,
					      solution,
					      thread_id);
		break;
	    }
	    case ENCODING_WATER_MDD:
	    case ENCODING_ID_WATER_MDD:
	    case ENCODING_AD_WATER_MDD:		
	    {
		extract_ComputedWaterMddSolution(instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution,
						 thread_id);
		break;
	    }	    
	    case ENCODING_RELAXED_MDD:
	    {
		extract_ComputedRelaxedMddSolution(instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution,
						   thread_id);
		break;
	    }
	    case ENCODING_TOKEN_MDD:
	    {
		extract_ComputedTokenMddSolution(instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution,
						   thread_id);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MDD:
	    {
		extract_ComputedTokenEmptyMddSolution(instance.m_initial_arrangement,
						      instance.m_environment,
						      instance.m_the_MDD,
						      total_cost,
						      encoding_context,
						      solution,
						      thread_id);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MDD:
	    {
		extract_ComputedPermutationMddSolution(instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution,
						   thread_id);
		break;
	    }
	    case ENCODING_PERMUTATION_CMDD:
	    {
		extract_ComputedCapacitatedPermutationMddSolution(instance.m_initial_arrangement,
								  instance.m_environment,
								  instance.m_the_MDD,
								  total_cost,
								  encoding_context,
								  solution,
								  thread_id);
		break;
	    }	    	    	    	    
	    case ENCODING_MDD_plus:
	    case ENCODING_ID_MDD_plus:
	    case ENCODING_AD_MDD_plus:
	    {
		extract_ComputedMddPlusSolution(instance.m_initial_arrangement,
						instance.m_environment,
						instance.m_the_MDD,
						total_cost,
						encoding_context,
						solution,
						thread_id);
		break;
	    }
	    case ENCODING_MDD_plus_plus:
	    case ENCODING_ID_MDD_plus_plus:
	    case ENCODING_AD_MDD_plus_plus:
	    {
		extract_ComputedMddPlusPlusSolution(instance.m_initial_arrangement,
						    instance.m_environment,
						    instance.m_the_MDD,
						    total_cost,
						    encoding_context,
						    solution,
						    thread_id);
		break;
	    }
	    case ENCODING_MDD_plus_plus_mutex:
	    {
		extract_ComputedMddPlusPlusMutexSolution(instance.m_initial_arrangement,
							 instance.m_environment,
							 instance.m_the_MDD,
							 total_cost,
							 encoding_context,
							 solution,
							 thread_id);
		break;
	    }	    
	    case ENCODING_LMDD_plus_plus:
	    {
		extract_ComputedLMddPlusPlusSolution(instance.m_initial_arrangement,
						     instance.m_environment,
						     instance.m_the_MDD,
						     total_cost,
						     encoding_context,
						     solution,
						     thread_id);
		break;
	    }
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		extract_ComputedMddPlusPlusFuelSolution(instance.m_initial_arrangement,
							instance.m_environment,
							instance.m_the_MDD,
							total_cost,
							total_cost,
							encoding_context,
							solution,
							thread_id);
		break;
	    }	    	    
	    case ENCODING_MDD_star:
	    case ENCODING_ID_MDD_star:
	    case ENCODING_AD_MDD_star:
	    {
		extract_ComputedMddStarSolution(instance.m_initial_arrangement,
						instance.m_environment,
						instance.m_the_MDD,
						total_cost,
						encoding_context,
						solution,
						thread_id);
		break;
	    }	    	    	    
	    case ENCODING_NOMDD:
	    case ENCODING_BNOMDD:
	    case ENCODING_BCNOMDD:
	    {
		extract_ComputedNoMddSolution(instance.m_initial_arrangement,
					      instance.m_environment,
					      instance.m_the_MDD,
					      total_cost,
					      encoding_context,
					      solution,
					      thread_id);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    solvability = SOLVABILITY_SAT;
	    return sRESULT_SUCCESS;
	}	

#ifndef sDEBUG	
	{
	    if (unlink(output_filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	}
#endif
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostSolvability(Glucose::Solver                   **solver,
								     sMultirobotInstance               &instance,
								     int                                total_cost,
								     int                                extra_cost,
								     Solvability                       &solvability,
								     sMultirobotSolution               &solution,
								     sMultirobotEncodingContext_CNFsat &encoding_context,
								     int                                sUNUSED(thread_id))
    {
	printf("Solvability for: %d,%d\n", total_cost, extra_cost);
	sString cnf_filename, cnf_out_filename, output_filename;

	encoding_context.m_max_total_cost = total_cost;
	encoding_context.m_extra_cost = extra_cost;

	encoding_context.m_max_total_fuel = total_cost;
	encoding_context.m_extra_fuel = extra_cost;	

	switch (m_encoding)
	{
	case ENCODING_MDD:
	case ENCODING_ID_MDD:
	case ENCODING_AD_MDD:
	case ENCODING_BMDD:
	case ENCODING_BCMDD:
	{	    
	    instance.to_Memory_MddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_UMTEX:
	{	    
	    instance.to_Memory_MddUmtexCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_MUTEX:
	{	    
	    instance.to_Memory_MddMutexCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_GMDD:
	{	    
	    instance.to_Memory_GMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_GEMDD:
	{	    
	    instance.to_Memory_GEMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_WATER_MDD:
	case ENCODING_ID_WATER_MDD:
	case ENCODING_AD_WATER_MDD:
	{	    
	    instance.to_Memory_WaterMddCNFsat(*solver, encoding_context, "", false);	    
	    break;
	}	
	case ENCODING_RELAXED_MDD:
	{	    
	    instance.to_Memory_RelaxedMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_MDD:
	{	    
	    instance.to_Memory_TokenMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_TOKEN_EMPTY_MDD:
	{	    
	    instance.to_Memory_TokenEmptyMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_PERMUTATION_MDD:
	{	    
	    instance.to_Memory_PermutationMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_PERMUTATION_CMDD:
	{	    
	    instance.to_Memory_CapacitatedPermutationMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}					
	case ENCODING_MDD_plus:
	case ENCODING_ID_MDD_plus:
	case ENCODING_AD_MDD_plus:
	{	    
	    instance.to_Memory_MddPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus:
	case ENCODING_ID_MDD_plus_plus:
	case ENCODING_AD_MDD_plus_plus:
	{	    
	    instance.to_Memory_MddPlusPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus_mutex:
	{	    
	    instance.to_Memory_MddPlusPlusMutexCNFsat(*solver, encoding_context, "", false);
	    break;
	}	
	case ENCODING_LMDD_plus_plus:
	{	    
	    instance.to_Memory_LMddPlusPlusCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	case ENCODING_MDD_plus_plus_fuel:
	{	    
	    instance.to_Memory_MddPlusPlusFuelCNFsat(*solver, encoding_context, "", false);
	    break;
	}		
	case ENCODING_MDD_star:
	case ENCODING_ID_MDD_star:
	case ENCODING_AD_MDD_star:
	{	    
	    instance.to_Memory_MddStarCNFsat(*solver, encoding_context, "", false);
	    break;
	}			
	case ENCODING_NOMDD:
	case ENCODING_BNOMDD:
	case ENCODING_BCNOMDD:
	{	    
	    instance.to_Memory_NoMddCNFsat(*solver, encoding_context, "", false);
	    break;
	}
	default:
	{
	    sASSERT(false);
	}
	}

	if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	{
	    rlimit rl;
	    getrlimit(RLIMIT_CPU, &rl);
	    
	    if (rl.rlim_max == RLIM_INFINITY || (rlim_t)m_minisat_timeout < rl.rlim_max)
	    {
		rl.rlim_cur = m_minisat_timeout;
		
		if (setrlimit(RLIMIT_CPU, &rl) == -1)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR;
		}
	    }
	} 

	if (!(*solver)->simplify())
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif		
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	}

	Glucose::vec<Glucose::Lit> dummy;
	Glucose::lbool ret = (*solver)->solveLimited(dummy);

	if (ret == l_True)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	    sMultirobotInstance::MDD_vector MDD, extra_MDD;		

	    switch (m_encoding)
	    {
	    case ENCODING_MDD:
	    case ENCODING_ID_MDD:
	    case ENCODING_AD_MDD:		
	    case ENCODING_BMDD:
	    case ENCODING_BCMDD:
	    {
		intract_ComputedMddSolution(*solver,
					    instance.m_initial_arrangement,
					    instance.m_environment,
					    instance.m_the_MDD,
					    total_cost,
					    encoding_context,
					    solution);
		break;
	    }
	    case ENCODING_MDD_UMTEX:
	    {
		intract_ComputedMddUmtexSolution(*solver,
						 instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution);
		break;
	    }
	    case ENCODING_MDD_MUTEX:
	    {
		intract_ComputedMddMutexSolution(*solver,
						 instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution);
		break;
	    }	    	    
	    case ENCODING_GMDD:
	    {
		intract_ComputedGMddSolution(*solver,
					     instance.m_initial_arrangement,
					     instance.m_environment,
					     instance.m_the_MDD,
					     total_cost,
					     encoding_context,
					     solution);
		break;
	    }
	    case ENCODING_GEMDD:
	    {
		intract_ComputedGEMddSolution(*solver,
					      instance.m_initial_arrangement,
					      instance.m_environment,
					      instance.m_the_MDD,
					      total_cost,
					      encoding_context,
					      solution);
		break;
	    }
	    case ENCODING_WATER_MDD:
	    case ENCODING_ID_WATER_MDD:
	    case ENCODING_AD_WATER_MDD:		
	    {
		intract_ComputedWaterMddSolution(*solver,
						 instance.m_initial_arrangement,
						 instance.m_environment,
						 instance.m_the_MDD,
						 total_cost,
						 encoding_context,
						 solution);
		break;
	    }	    
	    case ENCODING_RELAXED_MDD:
	    {
		intract_ComputedRelaxedMddSolution(*solver,
						   instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution);
		break;
	    }
	    case ENCODING_TOKEN_MDD:
	    {
		intract_ComputedTokenMddSolution(*solver,
						   instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MDD:
	    {
		intract_ComputedTokenEmptyMddSolution(*solver,
						      instance.m_initial_arrangement,
						      instance.m_environment,
						      instance.m_the_MDD,
						      total_cost,
						      encoding_context,
						      solution);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MDD:
	    {
		intract_ComputedPermutationMddSolution(*solver,
						   instance.m_initial_arrangement,
						   instance.m_environment,
						   instance.m_the_MDD,
						   total_cost,
						   encoding_context,
						   solution);
		break;
	    }
	    case ENCODING_PERMUTATION_CMDD:
	    {
		intract_ComputedCapacitatedPermutationMddSolution(*solver,
								  instance.m_initial_arrangement,
								  instance.m_environment,
								  instance.m_the_MDD,
								  total_cost,
								  encoding_context,
								  solution);
		break;
	    }	    	    	    	    
	    case ENCODING_MDD_plus:
	    case ENCODING_ID_MDD_plus:
	    case ENCODING_AD_MDD_plus:
	    {
		intract_ComputedMddPlusSolution(*solver,
						instance.m_initial_arrangement,
						instance.m_environment,
						instance.m_the_MDD,
						total_cost,
						encoding_context,
						solution);
		break;
	    }
	    case ENCODING_MDD_plus_plus:
	    case ENCODING_ID_MDD_plus_plus:
	    case ENCODING_AD_MDD_plus_plus:
	    {
		intract_ComputedMddPlusPlusSolution(*solver,
						    instance.m_initial_arrangement,
						    instance.m_environment,
						    instance.m_the_MDD,
						    total_cost,
						    encoding_context,
						    solution);
		break;
	    }
	    case ENCODING_MDD_plus_plus_mutex:
	    {
		intract_ComputedMddPlusPlusMutexSolution(*solver,
							 instance.m_initial_arrangement,
							 instance.m_environment,
							 instance.m_the_MDD,
							 total_cost,
							 encoding_context,
							 solution);
		break;
	    }	    
	    case ENCODING_LMDD_plus_plus:
	    {
		intract_ComputedLMddPlusPlusSolution(*solver,
						     instance.m_initial_arrangement,
						     instance.m_environment,
						     instance.m_the_MDD,
						     total_cost,
						     encoding_context,
						     solution);
		break;
	    }
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		intract_ComputedMddPlusPlusFuelSolution(*solver,
							instance.m_initial_arrangement,
							instance.m_environment,
							instance.m_the_MDD,
							total_cost,
							total_cost,
							encoding_context,
							solution);
		break;
	    }	    	    
	    case ENCODING_MDD_star:
	    case ENCODING_ID_MDD_star:
	    case ENCODING_AD_MDD_star:
	    {
		intract_ComputedMddStarSolution(*solver,
						instance.m_initial_arrangement,
						instance.m_environment,
						instance.m_the_MDD,
						total_cost,
						encoding_context,
						solution);
		break;
	    }	    	    	    
	    case ENCODING_NOMDD:
	    case ENCODING_BNOMDD:
	    case ENCODING_BCNOMDD:
	    {
		intract_ComputedNoMddSolution(*solver,
					      instance.m_initial_arrangement,
					      instance.m_environment,
					      instance.m_the_MDD,
					      total_cost,
					      encoding_context,
					      solution);
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    solvability = SOLVABILITY_SAT;
	    return sRESULT_SUCCESS;	    	    
	}
	else if (ret == l_False)
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
	    }
#endif
	    solvability = SOLVABILITY_UNSAT; 
	}
	else
	{
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
	    }
#endif
	    solvability = SOLVABILITY_INDET; 
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_BestExtraCost_binary(sMultirobotInstance               &instance,
									int                                total_cost,
									int                                lower_extra_cost,
									int                                upper_extra_cost,
									int                               &best_extra_cost,
									sMultirobotSolution               &best_solution,
									sMultirobotEncodingContext_CNFsat &encoding_context,
									int                                thread_id)
    {
	sResult result;
	best_extra_cost = -1;

	bool sat_solution_computed = false;
	sMultirobotSolution sat_solution;

	while (true)
	{
	    printf("Lower extra bound: %d\n", lower_extra_cost);
	    printf("Upper extra bound: %d\n", upper_extra_cost);

	    int total_extra_cost = (lower_extra_cost + upper_extra_cost) / 2;
	    printf("Total cost:%d\n", total_extra_cost);

	    Solvability solvability;
	    sMultirobotSolution solution;
	    
	    result = compute_CostSolvability(instance,
					     total_cost,
					     total_extra_cost,
					     solvability,
					     solution,
					     encoding_context,
					     thread_id);
	    printf("%d,%d\n", result, solvability);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    switch (solvability)
	    {
	    case SOLVABILITY_UNSAT:
	    {
		lower_extra_cost = total_extra_cost;
	
		if (lower_extra_cost + 1 >= upper_extra_cost)
		{
		    best_extra_cost = total_extra_cost + 1;

		    if (sat_solution_computed)
		    {
			best_solution = sat_solution;
		    }
		    else
		    {
			result = compute_CostSolvability(instance,
							 total_cost,
							 total_extra_cost + 1,
							 solvability,
							 best_solution,
							 encoding_context,
							 thread_id);
			if (sFAILED(result))
			{
			    return result;
			}
			sASSERT(solvability == SOLVABILITY_SAT);
		    }

		    return sRESULT_SUCCESS;
		}
		break;
	    }
	    case SOLVABILITY_SAT:
	    {
		upper_extra_cost = total_extra_cost;
		sat_solution = solution;
		sat_solution_computed = true;

		if (lower_extra_cost >= upper_extra_cost - 1)
		{
		    best_extra_cost = total_extra_cost;
		    best_solution = sat_solution;
		    return sRESULT_SUCCESS;
		}
		break;
	    }
	    case SOLVABILITY_INDET:
	    {
		return sRESULT_SUCCESS;
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}	   
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_BestExtraCost_binary(Glucose::Solver                   **solver,
									  sMultirobotInstance               &instance,
									  int                                total_cost,
									  int                                lower_extra_cost,
									  int                                upper_extra_cost,
									  int                               &best_extra_cost,
									  sMultirobotSolution               &best_solution,
									  sMultirobotEncodingContext_CNFsat &encoding_context,
									  int                                thread_id)
    {
	sResult result;
	best_extra_cost = -1;

	bool sat_solution_computed = false;
	sMultirobotSolution sat_solution;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    printf("Lower extra bound: %d\n", lower_extra_cost);
	    printf("Upper extra bound: %d\n", upper_extra_cost);

	    int total_extra_cost = (lower_extra_cost + upper_extra_cost) / 2;
	    printf("Total cost:%d\n", total_extra_cost);

	    Solvability solvability;
	    sMultirobotSolution solution;
	    
	    result = incompute_CostSolvability(solver,
					       instance,
					       total_cost,
					       total_extra_cost,
					       solvability,
					       solution,
					       encoding_context,
					       thread_id);
	    printf("%d,%d\n", result, solvability);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    switch (solvability)
	    {
	    case SOLVABILITY_UNSAT:
	    {
		lower_extra_cost = total_extra_cost;
	
		if (lower_extra_cost + 1 >= upper_extra_cost)
		{
		    best_extra_cost = total_extra_cost + 1;

		    if (sat_solution_computed)
		    {
			best_solution = sat_solution;
		    }
		    else
		    {
			result = incompute_CostSolvability(solver,
							   instance,
							   total_cost,
							   total_extra_cost + 1,
							   solvability,
							   best_solution,
							   encoding_context,
							   thread_id);
			if (sFAILED(result))
			{
			    return result;
			}
			sASSERT(solvability == SOLVABILITY_SAT);
		    }

		    return sRESULT_SUCCESS;
		}
		break;
	    }
	    case SOLVABILITY_SAT:
	    {
		upper_extra_cost = total_extra_cost;
		sat_solution = solution;
		sat_solution_computed = true;

		if (lower_extra_cost >= upper_extra_cost - 1)
		{
		    best_extra_cost = total_extra_cost;
		    best_solution = sat_solution;
		    return sRESULT_SUCCESS;
		}
		break;
	    }
	    case SOLVABILITY_INDET:
	    {
		return sRESULT_SUCCESS;
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}	   
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_BestExtraCost_linear(sMultirobotInstance               &instance,
									int                                total_cost,
									int                                lower_extra_cost,
									int                                sUNUSED(upper_extra_cost),
									int                               &best_extra_cost,
									sMultirobotSolution               &best_solution,
									sMultirobotEncodingContext_CNFsat &encoding_context,
									int                                thread_id)
    {
	sResult result;
	best_extra_cost = -1;

	sMultirobotSolution sat_solution;

	int total_extra_cost = lower_extra_cost;

	while (true)
	{
//	    printf("Lower extra bound: %d\n", lower_extra_cost);
//	    printf("Upper extra bound: %d\n", upper_extra_cost);

	    Solvability solvability;
	    sMultirobotSolution solution;
	    result = compute_CostSolvability(instance,
					     total_cost,
					     total_extra_cost,
					     solvability,
					     solution,
					     encoding_context,
					     thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    switch (solvability)
	    {
	    case SOLVABILITY_UNSAT:
	    {
		++total_extra_cost;	
		break;
	    }
	    case SOLVABILITY_SAT:
	    {
		best_extra_cost = total_extra_cost;
		best_solution = solution;
		return sRESULT_SUCCESS;
		break;
	    }
	    case SOLVABILITY_INDET:
	    {
		return sRESULT_SUCCESS;
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}	   
	return sRESULT_SUCCESS;
    }


/*
#ifdef sSTATISTICS
		    {
			++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		    }
#endif
		    high_extra_cost = total_extra_cost;
		    
		    if (low_extra_cost + 1 >= high_extra_cost)
		    {
			final_encoding_context = encoding_context;
			optimal_cost = total_cost;
			
			int extra_cost;
			sMultirobotSolution best_solution;
			
			sMultirobotInstance::MDD_vector MDD, extra_MDD;		
			instance.construct_MDD(final_encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);
			
			extract_ComputedMddSolution(start_arrangement,
						    environment,
						    MDD,
						    optimal_cost,
						    final_encoding_context,
						    best_solution,
						    thread_id);
			final_encoding_context.m_extra_cost = extra_cost;
//			best_solution.to_Screen();
			
			sMultirobotSolutionAnalyzer analyzer;
			int best_cost = analyzer.calc_TotalCost(best_solution, start_arrangement, environment);
			high_extra_cost = total_extra_cost;
			printf("Best,last:%d,%d\n", best_cost, last_cost);

			int cost_reduction;

			if (best_cost > last_cost)
			{
			    return sRESULT_SUCCESS;
			}
			if (best_cost == last_cost)
			{
			    int max_individual_cost;
			    int min_total_cost = instance.estimate_TotalCost(max_individual_cost);

			    if (best_cost == min_total_cost)
			    {
				return sRESULT_SUCCESS;
			    }
			    printf("Max. ind.: %d, Min. tot.:%d\n", max_individual_cost, min_total_cost);

#ifdef sSTATISTICS

			    {
				s_GlobalPhaseStatistics.enter_Phase("Proving");
			    }
#endif

			    sMultirobotEncodingContext_CNFsat prove_encoding_context(0);
			    
			    if (m_encoding ==  ENCODING_BCMDD)
			    {
				int N_Robots_1 = N_Robots / 2;
				int N_Robots_2 = N_Robots - N_Robots_1;

				sRobotArrangement start_arrangement_relaxed_1(N_Vertices, N_Robots_1);
				sRobotGoal final_arrangement_relaxed_1(N_Vertices, N_Robots_1);
				
				for (int r = 1; r <= N_Robots_1; ++r)
				{
				    start_arrangement_relaxed_1.place_Robot(r, start_arrangement.get_RobotLocation(r));
				    final_arrangement_relaxed_1.charge_Robot(r, *final_arrangement.get_RobotGoal(r).begin());
				}
				
				sRobotArrangement start_arrangement_relaxed_2(N_Vertices, N_Robots_2);
				sRobotGoal final_arrangement_relaxed_2(N_Vertices, N_Robots_2);
				
				for (int r = 1; r <= N_Robots_2; ++r)
				{
				    start_arrangement_relaxed_2.place_Robot(r, start_arrangement.get_RobotLocation(N_Robots_1 + r));
				    final_arrangement_relaxed_2.charge_Robot(r, *final_arrangement.get_RobotGoal(N_Robots_1 + r).begin());
				}
				
				sMultirobotInstance relaxed_instance_1(environment, sparse_environment, start_arrangement_relaxed_1, final_arrangement_relaxed_1);
				sMultirobotInstance relaxed_instance_2(environment, sparse_environment, start_arrangement_relaxed_2, final_arrangement_relaxed_2);
							
				int max_individual_cost_1;
				int min_total_cost_1 = relaxed_instance_1.estimate_TotalCost(max_individual_cost_1);
				
				
				int max_individual_cost_2;
				int min_total_cost_2 = relaxed_instance_2.estimate_TotalCost(max_individual_cost_2);
				
				int optimal_cost_relaxed_1;
				int optimal_cost_relaxed_2;
			    
				sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_1;

				finish_seconds = sGet_CPU_Seconds();
		
				if (finish_seconds - start_seconds  > m_total_timeout)
				  {
				    return sRESULT_SUCCESS;
				  }

				
				result = compute_OptimalCost(start_arrangement_relaxed_1,
							     final_arrangement_relaxed_1,
							     environment,
							     sparse_environment,
							     max_total_cost,
							     optimal_cost_relaxed_1,
							     final_encoding_context_relaxed_1,
							     thread_id);
				if (sFAILED(result))
				{
				    return result;
				}
				
				sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_2;

				finish_seconds = sGet_CPU_Seconds();
		
				if (finish_seconds - start_seconds  > m_total_timeout)
				  {
				    return sRESULT_SUCCESS;
				  }
				
				result = compute_OptimalCost(start_arrangement_relaxed_2,
							     final_arrangement_relaxed_2,
							     environment,
							     sparse_environment,
							     max_total_cost,
							     optimal_cost_relaxed_2,
							     final_encoding_context_relaxed_2,
							     thread_id);
				if (sFAILED(result))
				{
				    return result;
				}
				printf("Max. ind. 1: %d, Min. tot. 1:%d\n", max_individual_cost_1, min_total_cost_1);
				printf("Max. ind. 2: %d, Min. tot. 2:%d\n", max_individual_cost_2, min_total_cost_2);

				printf("Optimal relaxed cost 1: %d\n", optimal_cost_relaxed_1);
				printf("Optimal relaxed cost 2: %d\n", optimal_cost_relaxed_2);

				int cost_reduction_1 = optimal_cost_relaxed_1 - min_total_cost_1;
				int cost_reduction_2 = optimal_cost_relaxed_2 - min_total_cost_2;
				cost_reduction = sMIN(cost_reduction_1, cost_reduction_2);

				printf("Cost reductions: %d,%d,%d\n", cost_reduction_1, cost_reduction_2, cost_reduction);
				prove_encoding_context.m_extra_cost = best_cost - min_total_cost - 1;
			    }	
			    else
			    {
				cost_reduction = 0;
			    }
			    prove_encoding_context.m_max_total_cost = best_cost - cost_reduction - 1;
			    int prove_total_extra_cost = best_cost - 1;

			    switch (m_encoding)
			    {
			    case ENCODING_BMDD:
			    case ENCODING_BCMDD:
			    {
				if (thread_id != THREAD_ID_UNDEFINED)
				{
				    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
				    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
				    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
				}
				else
				{
				    cnf_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
				    cnf_out_filename = CNF_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
				    output_filename = OUTPUT_MDD_FILENAME_PREFIX + "_" + sInt_32_to_String(prove_total_extra_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
				}
				
				result = instance.to_File_MddCNFsat(cnf_filename, prove_encoding_context, "", false);
				
				if (sFAILED(result))
				{
				    return result;
				}
				break;
			    }
			    default:
			    {
				sASSERT(false);
			    }
			    }
#ifdef PREPROCESS
			    sString preprocess_call;
			    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	    
			    int preprocess_result = system(preprocess_call.c_str());
			    
			    if (preprocess_result < 0)
			    {
				return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
			    }
			    FILE *fro;
			    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
			    {
				cnf_out_filename = cnf_filename;
			    }
			    else
			    {
				fclose(fro);
			    }
#else
			    cnf_out_filename = cnf_filename;
#endif
			    sString system_call;
			    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
			    {
				system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
			    }
			    else
			    {
				system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
			    }

//			    s_GlobalPhaseStatistics.enter_Phase("SAT Prove");
			    int system_result = system(system_call.c_str());
//			    s_GlobalPhaseStatistics.leave_Phase();
			    
			    if (system_result < 0)
			    {
				return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
			    }
			    
#ifdef sSTATISTICS
			    {
				++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
			    }
#endif			    
			    FILE *fr;
			    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
			    {
				return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
			    }
			    
			    char answer[32];
			    answer[0] = '\0';
			    
			    fscanf(fr, "%s\n", answer);
			    fclose(fr);

#ifdef sSTATISTICS
			    {
				s_GlobalPhaseStatistics.leave_Phase();
			    }
#endif

#ifndef sDEBUG
			    {
				if (unlink(cnf_filename.c_str()) < 0)
				{
				    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
				}
			    }
#endif  
			    if (strcmp(answer, "UNSAT") == 0)
			    {
#ifdef sSTATISTICS
				{
				    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
				}
#endif
				
#ifndef sDEBUG
				{
				    if (unlink(output_filename.c_str()) < 0)
				    {
					return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
				    }
				}
#endif
				printf("UNSAT\n");
				printf("Best cost final: %d,%d\n", best_cost, high_extra_cost);
				return sRESULT_SUCCESS;
			    }
			    else if (strcmp(answer, "SAT") == 0)
			    {
#ifdef sSTATISTICS
				{
				    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
				}
#endif
				{
				    if (unlink(output_filename.c_str()) < 0)
				    {
					return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
				    }
				}
				--best_cost;
				printf("SAT\n");
			    }
			    else // INDET
			    {
#ifdef sSTATISTICS
				{
				    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
				}
#endif
				optimal_cost = MAKESPAN_UNDEFINED;
		    
#ifndef sDEBUG
				{
				    if (unlink(output_filename.c_str()) < 0)
				    {
					return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
				    }
				}
#endif		    
				printf("INDET\n");
				break;
			    }
			}
			last_cost = best_cost;
			break;
		    }
*/

    
    sResult sMultirobotSolutionCompressor::incompute_BestExtraCost_linear(Glucose::Solver                   **solver,
									  sMultirobotInstance               &instance,
									  int                                total_cost,
									  int                                lower_extra_cost,
									  int                                sUNUSED(upper_extra_cost),
									  int                               &best_extra_cost,
									  sMultirobotSolution               &best_solution,
									  sMultirobotEncodingContext_CNFsat &encoding_context,
									  int                                thread_id)
    {
	sResult result;
	best_extra_cost = -1;

	sMultirobotSolution sat_solution;

	int total_extra_cost = lower_extra_cost;

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
//	    printf("Lower extra bound: %d\n", lower_extra_cost);
//	    printf("Upper extra bound: %d\n", upper_extra_cost);

	    Solvability solvability;
	    sMultirobotSolution solution;
	    result = incompute_CostSolvability(solver,
					       instance,
					       total_cost,
					       total_extra_cost,
					       solvability,
					       solution,
					       encoding_context,
					       thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    switch (solvability)
	    {
	    case SOLVABILITY_UNSAT:
	    {
		++total_extra_cost;	
		break;
	    }
	    case SOLVABILITY_SAT:
	    {
		best_extra_cost = total_extra_cost;
		best_solution = solution;
		return sRESULT_SUCCESS;
		break;
	    }
	    case SOLVABILITY_INDET:
	    {
		return sRESULT_SUCCESS;
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	}	   
	return sRESULT_SUCCESS;
    }
    

    sResult sMultirobotSolutionCompressor::compute_CostReduction(sMultirobotInstance &instance,
								 int                  max_total_cost,
								 int                 &cost_reduction,
								 int                  thread_id)
    {
	sResult result;

	int N_Vertices = instance.m_initial_arrangement.get_VertexCount();
	int N_Robots = instance.m_initial_arrangement.get_RobotCount();

	int N_Robots_1 = N_Robots / 2;
	int N_Robots_2 = N_Robots - N_Robots_1;
	
	sRobotArrangement start_arrangement_relaxed_1(N_Vertices, N_Robots_1);
	sRobotGoal final_arrangement_relaxed_1(N_Vertices, N_Robots_1);
	
	for (int r = 1; r <= N_Robots_1; ++r)
	{
	    start_arrangement_relaxed_1.place_Robot(r, instance.m_initial_arrangement.get_RobotLocation(r));
	    final_arrangement_relaxed_1.charge_Robot(r, *instance.m_goal_specification.get_RobotGoal(r).begin());
	}
	
	sRobotArrangement start_arrangement_relaxed_2(N_Vertices, N_Robots_2);
	sRobotGoal final_arrangement_relaxed_2(N_Vertices, N_Robots_2);
	
	for (int r = 1; r <= N_Robots_2; ++r)
	{
	    start_arrangement_relaxed_2.place_Robot(r, instance.m_initial_arrangement.get_RobotLocation(N_Robots_1 + r));
	    final_arrangement_relaxed_2.charge_Robot(r, *instance.m_goal_specification.get_RobotGoal(N_Robots_1 + r).begin());
	}
	
	sMultirobotInstance relaxed_instance_1(instance.m_environment, instance.m_sparse_environment, start_arrangement_relaxed_1, final_arrangement_relaxed_1);
	sMultirobotInstance relaxed_instance_2(instance.m_environment, instance.m_sparse_environment, start_arrangement_relaxed_2, final_arrangement_relaxed_2);
	
	int max_individual_cost_1;
	int min_total_cost_1 = relaxed_instance_1.estimate_TotalCost(max_individual_cost_1);
	
	
	int max_individual_cost_2;
	int min_total_cost_2 = relaxed_instance_2.estimate_TotalCost(max_individual_cost_2);
	
	int optimal_cost_relaxed_1;
	int optimal_cost_relaxed_2;

	int expansion_count_relaxed_1;
	int expansion_count_relaxed_2;
	
	sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_1;

	result = compute_OptimalCost(relaxed_instance_1,
				     max_total_cost,
				     optimal_cost_relaxed_1,
				     expansion_count_relaxed_1,
				     final_encoding_context_relaxed_1,
				     thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost_relaxed_1 < 0)
	{
	    cost_reduction = -1;
	    return sRESULT_SUCCESS;
	}
	
	sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_2;
	
	result = compute_OptimalCost(relaxed_instance_2,
				     max_total_cost,
				     optimal_cost_relaxed_2,
				     expansion_count_relaxed_2,
				     final_encoding_context_relaxed_2,
				     thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost_relaxed_2 < 0)
	{
	    cost_reduction = -1;
	    return sRESULT_SUCCESS;
	}

	printf("Max. ind. 1: %d, Min. tot. 1:%d\n", max_individual_cost_1, min_total_cost_1);
	printf("Max. ind. 2: %d, Min. tot. 2:%d\n", max_individual_cost_2, min_total_cost_2);
	
	printf("Optimal relaxed cost 1: %d\n", optimal_cost_relaxed_1);
	printf("Optimal relaxed cost 2: %d\n", optimal_cost_relaxed_2);
	
	int cost_reduction_1 = optimal_cost_relaxed_1 - min_total_cost_1;
	int cost_reduction_2 = optimal_cost_relaxed_2 - min_total_cost_2;
	cost_reduction = sMIN(cost_reduction_1, cost_reduction_2);
	
	printf("Cost reductions: %d,%d,%d\n", cost_reduction_1, cost_reduction_2, cost_reduction);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostReduction(Glucose::Solver     **solver,
								   sMultirobotInstance &instance,
								   int                  max_total_cost,
								   int                 &cost_reduction,
								   int                  thread_id)
    {
	sResult result;

	int N_Vertices = instance.m_initial_arrangement.get_VertexCount();
	int N_Robots = instance.m_initial_arrangement.get_RobotCount();

	int N_Robots_1 = N_Robots / 2;
	int N_Robots_2 = N_Robots - N_Robots_1;
	
	sRobotArrangement start_arrangement_relaxed_1(N_Vertices, N_Robots_1);
	sRobotGoal final_arrangement_relaxed_1(N_Vertices, N_Robots_1);
	
	for (int r = 1; r <= N_Robots_1; ++r)
	{
	    start_arrangement_relaxed_1.place_Robot(r, instance.m_initial_arrangement.get_RobotLocation(r));
	    final_arrangement_relaxed_1.charge_Robot(r, *instance.m_goal_specification.get_RobotGoal(r).begin());
	}
	
	sRobotArrangement start_arrangement_relaxed_2(N_Vertices, N_Robots_2);
	sRobotGoal final_arrangement_relaxed_2(N_Vertices, N_Robots_2);
	
	for (int r = 1; r <= N_Robots_2; ++r)
	{
	    start_arrangement_relaxed_2.place_Robot(r, instance.m_initial_arrangement.get_RobotLocation(N_Robots_1 + r));
	    final_arrangement_relaxed_2.charge_Robot(r, *instance.m_goal_specification.get_RobotGoal(N_Robots_1 + r).begin());
	}
	
	sMultirobotInstance relaxed_instance_1(instance.m_environment, instance.m_sparse_environment, start_arrangement_relaxed_1, final_arrangement_relaxed_1);
	sMultirobotInstance relaxed_instance_2(instance.m_environment, instance.m_sparse_environment, start_arrangement_relaxed_2, final_arrangement_relaxed_2);
	
	int max_individual_cost_1;
	int min_total_cost_1 = relaxed_instance_1.estimate_TotalCost(max_individual_cost_1);
	
	
	int max_individual_cost_2;
	int min_total_cost_2 = relaxed_instance_2.estimate_TotalCost(max_individual_cost_2);
	
	int optimal_cost_relaxed_1;
	int optimal_cost_relaxed_2;

	int expansion_count_relaxed_1;
	int expansion_count_relaxed_2;
	
	sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_1;

	result = incompute_OptimalCost(solver,
				       relaxed_instance_1,
				       max_total_cost,
				       optimal_cost_relaxed_1,
				       expansion_count_relaxed_1,
				       final_encoding_context_relaxed_1,
				       thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost_relaxed_1 < 0)
	{
	    cost_reduction = -1;
	    return sRESULT_SUCCESS;
	}
	
	sMultirobotEncodingContext_CNFsat final_encoding_context_relaxed_2;
	
	result = incompute_OptimalCost(solver,
				       relaxed_instance_2,
				       max_total_cost,
				       optimal_cost_relaxed_2,
				       expansion_count_relaxed_2,
				       final_encoding_context_relaxed_2,
				       thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost_relaxed_2 < 0)
	{
	    cost_reduction = -1;
	    return sRESULT_SUCCESS;
	}

	printf("Max. ind. 1: %d, Min. tot. 1:%d\n", max_individual_cost_1, min_total_cost_1);
	printf("Max. ind. 2: %d, Min. tot. 2:%d\n", max_individual_cost_2, min_total_cost_2);
	
	printf("Optimal relaxed cost 1: %d\n", optimal_cost_relaxed_1);
	printf("Optimal relaxed cost 2: %d\n", optimal_cost_relaxed_2);
	
	int cost_reduction_1 = optimal_cost_relaxed_1 - min_total_cost_1;
	int cost_reduction_2 = optimal_cost_relaxed_2 - min_total_cost_2;
	cost_reduction = sMIN(cost_reduction_1, cost_reduction_2);
	
	printf("Cost reductions: %d,%d,%d\n", cost_reduction_1, cost_reduction_2, cost_reduction);

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_BestCost(const sRobotArrangement           &start_arrangement,
							    const sRobotGoal                  &final_arrangement,
							    sUndirectedGraph                  &environment,
							    const sUndirectedGraph            &sparse_environment,
							    int                                max_total_cost,
							    int                               &optimal_cost,
							    sMultirobotSolution               &optimal_solution,
							    sMultirobotEncodingContext_CNFsat &sUNUSED(final_encoding_context),
							    int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	optimal_cost = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	int lower_total_bound, upper_total_bound;
	sMultirobotEncodingContext_CNFsat upper_encoding_context;

	compute_UpperCost(start_arrangement,
			  final_arrangement,
			  environment,
			  sparse_environment,
			  max_total_cost,
			  upper_total_bound,
			  upper_encoding_context,
			  thread_id);
	lower_total_bound = upper_encoding_context.m_max_total_cost;


	int max_individual_cost;
	int min_total_cost = instance.estimate_TotalCost(max_individual_cost);

	sMultirobotEncodingContext_CNFsat encoding_context(0);

	while (true)
	{	    
	    int lower_extra_cost = lower_total_bound - min_total_cost;
	    int upper_extra_cost = upper_total_bound - min_total_cost;

	    int best_extra_cost;
	    sMultirobotSolution best_solution;

	    result = compute_BestExtraCost_binary(instance,
						  lower_total_bound,
						  lower_extra_cost,
						  upper_extra_cost,
						  best_extra_cost,
						  best_solution,
						  encoding_context,
						  thread_id);
/*
	    result = compute_BestExtraCost_linear(instance,
						  lower_total_bound,
						  lower_extra_cost,
						  upper_extra_cost,
						  best_extra_cost,
						  best_solution,
						  encoding_context,
						  thread_id);
*/
	    best_solution.to_Screen();

	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (best_extra_cost < 0)
	    {
		return sRESULT_SUCCESS;
	    }
	    int next_upper_total_bound = min_total_cost + best_extra_cost;

//	    printf("NLUB:%d,%d,%d,%d\n", next_upper_total_bound, lower_total_bound, upper_total_bound, best_extra_cost);

	    if (lower_total_bound >= next_upper_total_bound)
	    {
		optimal_cost = next_upper_total_bound;
		optimal_solution = best_solution;
		return sRESULT_SUCCESS;
	    }
	    else
	    {
		if (next_upper_total_bound >= upper_total_bound)
		{
		    int cost_reduction = 0;
		    if (m_encoding == ENCODING_BCMDD || m_encoding == ENCODING_BCNOMDD)
		    {
			result = compute_CostReduction(instance,
						       max_total_cost,
						       cost_reduction,
						       thread_id);
			if (sFAILED(result))
			{
			    return result;
			}
			if (cost_reduction < 0)
			{
			    return sRESULT_SUCCESS;
			}
		    }

		    Solvability prove_solvability;
		    sMultirobotSolution prove_solution;
		    int prove_total_cost = next_upper_total_bound;
		    int prove_extra_cost = prove_total_cost - min_total_cost;

		    int check_total_cost = (lower_total_bound + upper_total_bound) / 2;

		    bool proof_proceed = true;

		    if (prove_total_cost - 1 > check_total_cost)
		    {
			Solvability check_solvability;
			sMultirobotSolution check_solution;
	

			result = compute_CostSolvability(instance,
							 check_total_cost,
							 prove_extra_cost - 1,
							 check_solvability,
							 check_solution,
							 encoding_context,
							 thread_id);

			switch (check_solvability)
			{
			case SOLVABILITY_UNSAT:
			{
			    proof_proceed = true;
			    break;
			}
			case SOLVABILITY_SAT:
			{
			    proof_proceed = false;
			    break;
			}
			case SOLVABILITY_INDET:
			{
			    return sRESULT_SUCCESS;
			    break;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		    if (proof_proceed)
		    {
			result = compute_CostSolvability(instance,
							 prove_total_cost - cost_reduction - 1,
							 prove_extra_cost - 1,
							 prove_solvability,
							 prove_solution,
							 encoding_context,
							 thread_id);
			
			switch (prove_solvability)
			{
			case SOLVABILITY_UNSAT:
			{
			    optimal_cost = prove_total_cost;
			    optimal_solution = best_solution;
			    return sRESULT_SUCCESS;
			}
			case SOLVABILITY_SAT:
			{
			    --next_upper_total_bound;
			    break;
			}
			case SOLVABILITY_INDET:
			{
			    return sRESULT_SUCCESS;
			    break;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}	    
		    }
		}
	    }
	    upper_total_bound = next_upper_total_bound;
	    finish_seconds = sGet_CPU_Seconds();
	    
	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		return sRESULT_SUCCESS;
	    }
	    ++lower_total_bound;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_BestCost(Glucose::Solver                   **solver,
							      const sRobotArrangement           &start_arrangement,
							      const sRobotGoal                  &final_arrangement,
							      sUndirectedGraph                  &environment,
							      const sUndirectedGraph            &sparse_environment,
							      int                                max_total_cost,
							      int                               &optimal_cost,
							      sMultirobotSolution               &optimal_solution,
							      sMultirobotEncodingContext_CNFsat &sUNUSED(final_encoding_context),
							      int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	optimal_cost = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	int lower_total_bound, upper_total_bound;
	sMultirobotEncodingContext_CNFsat upper_encoding_context;

	incompute_UpperCost(solver,
			    start_arrangement,
			    final_arrangement,
			    environment,
			    sparse_environment,
			    max_total_cost,
			    upper_total_bound,
			    upper_encoding_context,
			    thread_id);
	lower_total_bound = upper_encoding_context.m_max_total_cost;


	int max_individual_cost;
	int min_total_cost = instance.estimate_TotalCost(max_individual_cost);

	sMultirobotEncodingContext_CNFsat encoding_context(0);

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    int lower_extra_cost = lower_total_bound - min_total_cost;
	    int upper_extra_cost = upper_total_bound - min_total_cost;

	    int best_extra_cost;
	    sMultirobotSolution best_solution;

	    result = incompute_BestExtraCost_binary(solver,
						    instance,
						    lower_total_bound,
						    lower_extra_cost,
						    upper_extra_cost,
						    best_extra_cost,
						    best_solution,
						    encoding_context,
						    thread_id);
/*
	    result = compute_BestExtraCost_linear(instance,
						  lower_total_bound,
						  lower_extra_cost,
						  upper_extra_cost,
						  best_extra_cost,
						  best_solution,
						  encoding_context,
						  thread_id);
*/
	    best_solution.to_Screen();

	    if (sFAILED(result))
	    {
		return result;
	    }
	    if (best_extra_cost < 0)
	    {
		return sRESULT_SUCCESS;
	    }
	    int next_upper_total_bound = min_total_cost + best_extra_cost;

//	    printf("NLUB:%d,%d,%d,%d\n", next_upper_total_bound, lower_total_bound, upper_total_bound, best_extra_cost);

	    if (lower_total_bound >= next_upper_total_bound)
	    {
		optimal_cost = next_upper_total_bound;
		optimal_solution = best_solution;
		return sRESULT_SUCCESS;
	    }
	    else
	    {
		if (next_upper_total_bound >= upper_total_bound)
		{
		    int cost_reduction = 0;
		    if (m_encoding == ENCODING_BCMDD || m_encoding == ENCODING_BCNOMDD)
		    {
			result = incompute_CostReduction(solver,
							 instance,
							 max_total_cost,
							 cost_reduction,
							 thread_id);
			if (sFAILED(result))
			{
			    return result;
			}
			if (cost_reduction < 0)
			{
			    return sRESULT_SUCCESS;
			}
		    }

		    Solvability prove_solvability;
		    sMultirobotSolution prove_solution;
		    int prove_total_cost = next_upper_total_bound;
		    int prove_extra_cost = prove_total_cost - min_total_cost;

		    int check_total_cost = (lower_total_bound + upper_total_bound) / 2;

		    bool proof_proceed = true;

		    if (prove_total_cost - 1 > check_total_cost)
		    {
			Solvability check_solvability;
			sMultirobotSolution check_solution;
	

			result = incompute_CostSolvability(solver,
							   instance,
							   check_total_cost,
							   prove_extra_cost - 1,
							   check_solvability,
							   check_solution,
							   encoding_context,
							   thread_id);

			switch (check_solvability)
			{
			case SOLVABILITY_UNSAT:
			{
			    proof_proceed = true;
			    break;
			}
			case SOLVABILITY_SAT:
			{
			    proof_proceed = false;
			    break;
			}
			case SOLVABILITY_INDET:
			{
			    return sRESULT_SUCCESS;
			    break;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		    if (proof_proceed)
		    {
			result = incompute_CostSolvability(solver,
							   instance,
							   prove_total_cost - cost_reduction - 1,
							   prove_extra_cost - 1,
							   prove_solvability,
							   prove_solution,
							   encoding_context,
							   thread_id);
			
			switch (prove_solvability)
			{
			case SOLVABILITY_UNSAT:
			{
			    optimal_cost = prove_total_cost;
			    optimal_solution = best_solution;
			    return sRESULT_SUCCESS;
			}
			case SOLVABILITY_SAT:
			{
			    --next_upper_total_bound;
			    break;
			}
			case SOLVABILITY_INDET:
			{
			    return sRESULT_SUCCESS;
			    break;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}	    
		    }
		}
	    }
	    upper_total_bound = next_upper_total_bound;
	    finish_seconds = sGet_CPU_Seconds();
	    
	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		return sRESULT_SUCCESS;
	    }
	    ++lower_total_bound;
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_UpperCost(const sRobotArrangement           &start_arrangement,
							     const sRobotGoal                  &final_arrangement,
							     sUndirectedGraph                  &environment,
							     const sUndirectedGraph            &sparse_environment,
							     int                                max_total_cost,
							     int                               &optimal_cost,
							     sMultirobotEncodingContext_CNFsat &final_encoding_context,
							     int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	sString cnf_filename, cnf_out_filename, output_filename;
	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;	    

#ifdef sVERBOSE
	    printf("Solving cost %d ...\n", total_cost);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_BMDD:
	    case ENCODING_BCMDD:
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_RXMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_RXMddCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_RXNOMDD:
	    case ENCODING_BNOMDD:
	    case ENCODING_BCNOMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_RXNOMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(total_cost) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_RXNoMddCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	    
	    int preprocess_result = system(preprocess_call.c_str());
	    
	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());
	    
	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	
	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);
	    
#ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif  
	    if (strcmp(answer, "UNSAT") == 0)
	    {
#ifdef sSTATISTICS
		{
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0) */
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
#endif
	
		final_encoding_context = encoding_context;
		optimal_cost = total_cost;

		sMultirobotSolution upper_solution;

		switch (m_encoding)
		{
		case ENCODING_BMDD:
		case ENCODING_BCMDD:
		case ENCODING_RXMDD:
		{
		    extract_ComputedRXMddSolution(start_arrangement,
						  environment,
						  instance.m_the_MDD,
						  optimal_cost,
						  final_encoding_context,
						  upper_solution,
						  thread_id);
		    break;
		}
		case ENCODING_RXNOMDD:
		case ENCODING_BNOMDD:
		case ENCODING_BCNOMDD:
		{
		    extract_ComputedRXNoMddSolution(start_arrangement,
						    environment,
						    instance.m_the_MDD,
						    optimal_cost,
						    final_encoding_context,
						    upper_solution,
						    thread_id);
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}

//		final_encoding_context.m_extra_cost = extra_cost;
//		printf("Upper solution\n");
		upper_solution.to_Screen();

		sMultirobotSolutionAnalyzer analyzer;
		optimal_cost = analyzer.calc_TotalCost(upper_solution, start_arrangement, environment);
		break;
	    }
//	    total_cost += 30;

	    if (++total_cost > max_total_cost)
	    {
		break;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_UpperCost(Glucose::Solver                   **solver,
							       const sRobotArrangement           &start_arrangement,
							       const sRobotGoal                  &final_arrangement,
							       sUndirectedGraph                  &environment,
							       const sUndirectedGraph            &sparse_environment,
							       int                                max_total_cost,
							       int                               &optimal_cost,
							       sMultirobotEncodingContext_CNFsat &final_encoding_context,
							       int                                sUNUSED(thread_id))
    {
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	optimal_cost = MAKESPAN_UNDEFINED;

	int max_individual_cost;
	int total_cost = instance.estimate_TotalCost(max_individual_cost);

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    sMultirobotEncodingContext_CNFsat encoding_context(0);
	    encoding_context.m_max_total_cost = total_cost;
	    encoding_context.m_max_total_fuel = total_cost;	    

#ifdef sVERBOSE
	    printf("Solving cost %d ...\n", total_cost);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_BMDD:
	    case ENCODING_BCMDD:
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		instance.to_Memory_RXMddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_RXNOMDD:
	    case ENCODING_BNOMDD:
	    case ENCODING_BCNOMDD:
	    {
		instance.to_Memory_RXNoMddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
#endif
		
		final_encoding_context = encoding_context;
		optimal_cost = total_cost;

		sMultirobotSolution upper_solution;
		
		switch (m_encoding)
		{
		case ENCODING_BMDD:
		case ENCODING_BCMDD:
		case ENCODING_RXMDD:
		{
		    intract_ComputedRXMddSolution(*solver,
						  start_arrangement,
						  environment,
						  instance.m_the_MDD,
						  optimal_cost,
						  final_encoding_context,
						  upper_solution);
		    break;
		}
		case ENCODING_RXNOMDD:
		case ENCODING_BNOMDD:
		case ENCODING_BCNOMDD:
		{
		    intract_ComputedRXNoMddSolution(*solver,
						    start_arrangement,
						    environment,
						    instance.m_the_MDD,
						    optimal_cost,
						    final_encoding_context,
						    upper_solution);
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}

//		final_encoding_context.m_extra_cost = extra_cost;
//		printf("Upper solution\n");
		upper_solution.to_Screen();

		sMultirobotSolutionAnalyzer analyzer;
		optimal_cost = analyzer.calc_TotalCost(upper_solution, start_arrangement, environment);
		break;
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		optimal_cost = MAKESPAN_UNDEFINED;		
	    }

	    if (++total_cost > max_total_cost)
	    {
		break;
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolution(const sRobotArrangement                        &start_arrangement,
								       const sRobotGoal                               &final_arrangement,
								       const sUndirectedGraph                         &environment,
								       const sUndirectedGraph                         &sparse_environment,
								       const sMultirobotInstance::Environments_vector &sUNUSED(heighted_Environments),
								       int                                             max_total_cost,
								       int                                            &optimal_cost,
								       sMultirobotSolution                            &optimal_solution,
								       int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_OptimalCost(instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_HEIGHTED:
	    {
		instance.build_HeightedEnvironments_(optimal_cost);

		result = extract_ComputedHeightedSolution(start_arrangement,
							  environment,
							  instance.m_heighted_Environments,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolution(Glucose::Solver                                **solver,
									 const sRobotArrangement                        &start_arrangement,
									 const sRobotGoal                               &final_arrangement,
									 const sUndirectedGraph                         &environment,
									 const sUndirectedGraph                         &sparse_environment,
									 const sMultirobotInstance::Environments_vector &sUNUSED(heighted_Environments),
									 int                                             max_total_cost,
									 int                                            &optimal_cost,
									 sMultirobotSolution                            &optimal_solution,
									 int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_OptimalCost(solver, instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_HEIGHTED:
	    {
		instance.build_HeightedEnvironments_(optimal_cost);

		result = intract_ComputedHeightedSolution(*solver,
							  start_arrangement,
							  environment,
							  optimal_cost,
							  instance.m_heighted_Environments,
							  final_encoding_context,
							  optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolution(const sRobotArrangement                        &start_arrangement,
								       const sRobotGoal                               &final_arrangement,
								       const sUndirectedGraph                         &environment,
								       const sUndirectedGraph                         &sparse_environment,
								       const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
								       int                                             max_total_cost,
								       int                                            &optimal_cost,
								       sMultirobotSolution                            &optimal_solution,
								       int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);
	
	result = compute_OptimalCost(instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}	
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	    	   
	    switch (m_encoding)
	    {
	    case ENCODING_MDD:
	    case ENCODING_ID_MDD:
	    case ENCODING_AD_MDD:		
	    {		
		result = extract_ComputedMddSolution(start_arrangement,
						     environment,
						     instance.m_the_MDD,
						     optimal_cost,
						     final_encoding_context,
						     optimal_solution,
						     thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_MDD_UMTEX:
	    {		
		result = extract_ComputedMddUmtexSolution(start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_MUTEX:
	    {		
		result = extract_ComputedMddMutexSolution(start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_GMDD:
	    {		
		result = extract_ComputedGMddSolution(start_arrangement,
						      environment,
						      instance.m_the_MDD,
						      optimal_cost,
						      final_encoding_context,
						      optimal_solution,
						      thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GEMDD:
	    {		
		result = extract_ComputedGEMddSolution(start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution,
						       thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_WATER_MDD:		
	    case ENCODING_ID_WATER_MDD:
	    case ENCODING_AD_WATER_MDD:		
	    {		
		result = extract_ComputedWaterMddSolution(start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_RELAXED_MDD:
	    {		
		result = extract_ComputedRelaxedMddSolution(start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution,
							    thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MDD:
	    {		
		result = extract_ComputedTokenMddSolution(start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution,
							    thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MDD:
	    {		
		result = extract_ComputedTokenEmptyMddSolution(start_arrangement,
							       environment,
							       instance.m_the_MDD,
							       optimal_cost,
							       final_encoding_context,
							       optimal_solution,
							       thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MDD:
	    {		
		result = extract_ComputedPermutationMddSolution(start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution,
							    thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMDD:
	    {		
		result = extract_ComputedCapacitatedPermutationMddSolution(start_arrangement,
									   environment,
									   instance.m_the_MDD,
									   optimal_cost,
									   final_encoding_context,
									   optimal_solution,
									   thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MDD_plus:
	    case ENCODING_ID_MDD_plus:
	    case ENCODING_AD_MDD_plus:
	    {
		result = extract_ComputedMddPlusSolution(start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution,
							 thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus:
	    case ENCODING_ID_MDD_plus_plus:
	    case ENCODING_AD_MDD_plus_plus:
	    {
		result = extract_ComputedMddPlusPlusSolution(start_arrangement,
							     environment,
							     instance.m_the_MDD,
							     optimal_cost,
							     final_encoding_context,
							     optimal_solution,
							     thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus_mutex:
	    {
		result = extract_ComputedMddPlusPlusMutexSolution(start_arrangement,
								  environment,
								  instance.m_the_MDD,
								  optimal_cost,
								  final_encoding_context,
								  optimal_solution,
								  thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_LMDD_plus_plus:
	    {
		result = extract_ComputedLMddPlusPlusSolution(start_arrangement,
							      environment,
							      instance.m_the_MDD,
							      optimal_cost,
							      final_encoding_context,
							      optimal_solution,
							      thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		result = extract_ComputedMddPlusPlusFuelSolution(start_arrangement,
								 environment,
								 instance.m_the_MDD,
								 optimal_cost,
								 optimal_cost,
								 final_encoding_context,
								 optimal_solution,
								 thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_MDD_star:
	    case ENCODING_ID_MDD_star:
	    case ENCODING_AD_MDD_star:
	    {
		result = extract_ComputedMddStarSolution(start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution,
							 thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    
	    case ENCODING_RXMDD:
	    {
		result = extract_ComputedRXMddSolution(start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution,
						       thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_NOMDD:
	    {
		result = extract_ComputedNoMddSolution(start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution,
						       thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_RXNOMDD:
	    {
		result = extract_ComputedRXNoMddSolution(start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution,
							 thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);       
	int real_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	optimal_solution.m_optimality_ratio = (double)real_cost / final_encoding_context.m_max_total_cost;

 	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolution(Glucose::Solver                                **solver,
									 const sRobotArrangement                        &start_arrangement,
									 const sRobotGoal                               &final_arrangement,
									 const sUndirectedGraph                         &environment,
									 const sUndirectedGraph                         &sparse_environment,
									 const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
									 int                                             max_total_cost,
									 int                                            &optimal_cost,
									 sMultirobotSolution                            &optimal_solution,
									 int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);
	
	result = incompute_OptimalCost(solver, instance, max_total_cost, optimal_cost, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}	
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	    	   
	    switch (m_encoding)
	    {
	    case ENCODING_MDD:
	    case ENCODING_ID_MDD:
	    case ENCODING_AD_MDD:		
	    {		
		result = intract_ComputedMddSolution(*solver,
						     start_arrangement,
						     environment,
						     instance.m_the_MDD,
						     optimal_cost,
						     final_encoding_context,
						     optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_UMTEX:
	    {		
		result = intract_ComputedMddUmtexSolution(*solver,
							  start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_MUTEX:
	    {		
		result = intract_ComputedMddMutexSolution(*solver,
							  start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_GMDD:
	    {		
		result = intract_ComputedGMddSolution(*solver,
						      start_arrangement,
						      environment,
						      instance.m_the_MDD,
						      optimal_cost,
						      final_encoding_context,
						      optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GEMDD:
	    {		
		result = intract_ComputedGEMddSolution(*solver,
						       start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_WATER_MDD:
	    case ENCODING_ID_WATER_MDD:
	    case ENCODING_AD_WATER_MDD:		
	    {		
		result = intract_ComputedWaterMddSolution(*solver,
							  start_arrangement,
							  environment,
							  instance.m_the_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_RELAXED_MDD:
	    {		
		result = intract_ComputedRelaxedMddSolution(*solver,
							    start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MDD:
	    {		
		result = intract_ComputedTokenMddSolution(*solver,
							    start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MDD:
	    {		
		result = intract_ComputedTokenEmptyMddSolution(*solver,
							       start_arrangement,
							       environment,
							       instance.m_the_MDD,
							       optimal_cost,
							       final_encoding_context,
							       optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MDD:
	    {		
		result = intract_ComputedPermutationMddSolution(*solver,
							    start_arrangement,
							    environment,
							    instance.m_the_MDD,
							    optimal_cost,
							    final_encoding_context,
							    optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMDD:
	    {		
		result = intract_ComputedCapacitatedPermutationMddSolution(*solver,
									   start_arrangement,
									   environment,
									   instance.m_the_MDD,
									   optimal_cost,
									   final_encoding_context,
									   optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_MDD_plus:
	    case ENCODING_ID_MDD_plus:
	    case ENCODING_AD_MDD_plus:
	    {
		result = intract_ComputedMddPlusSolution(*solver,
							 start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus:
	    case ENCODING_ID_MDD_plus_plus:
	    case ENCODING_AD_MDD_plus_plus:
	    {
		result = intract_ComputedMddPlusPlusSolution(*solver,
							     start_arrangement,
							     environment,
							     instance.m_the_MDD,
							     optimal_cost,
							     final_encoding_context,
							     optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus_mutex:
	    {
		result = intract_ComputedMddPlusPlusMutexSolution(*solver,
								  start_arrangement,
								  environment,
								  instance.m_the_MDD,
								  optimal_cost,
								  final_encoding_context,
								  optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_LMDD_plus_plus:
	    {
		result = intract_ComputedLMddPlusPlusSolution(*solver,
							      start_arrangement,
							      environment,
							      instance.m_the_MDD,
							      optimal_cost,
							      final_encoding_context,
							      optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		result = intract_ComputedMddPlusPlusFuelSolution(*solver,
								 start_arrangement,
								 environment,
								 instance.m_the_MDD,
								 optimal_cost,
								 optimal_cost,
								 final_encoding_context,
								 optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_MDD_star:
	    case ENCODING_ID_MDD_star:
	    case ENCODING_AD_MDD_star:
	    {
		result = intract_ComputedMddStarSolution(*solver,
							 start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    
	    case ENCODING_RXMDD:
	    {
		result = intract_ComputedRXMddSolution(*solver,
						       start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_NOMDD:
	    {
		result = intract_ComputedNoMddSolution(*solver,
						       start_arrangement,
						       environment,
						       instance.m_the_MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_RXNOMDD:
	    {
		result = intract_ComputedRXNoMddSolution(*solver,
							 start_arrangement,
							 environment,
							 instance.m_the_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);       
	int real_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	optimal_solution.m_optimality_ratio = (double)real_cost / final_encoding_context.m_max_total_cost;

 	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compute_FuelOptimalSolution(const sRobotArrangement                        &start_arrangement,
								       const sRobotGoal                               &final_arrangement,
								       const sUndirectedGraph                         &environment,
								       const sUndirectedGraph                         &sparse_environment,
								       const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
								       int                                             max_total_fuel,
								       int                                            &optimal_fuel,
								       int                                            &fuel_makespan,
								       sMultirobotSolution                            &optimal_solution,
								       int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);
	
	result = compute_OptimalFuel(instance, max_total_fuel, optimal_fuel, fuel_makespan, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}	
	if (optimal_fuel != MAKESPAN_UNDEFINED)
	{	    	   
	    switch (m_encoding)
	    {
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		result = extract_ComputedMddPlusPlusFuelSolution(start_arrangement,
								 environment,
								 instance.m_the_MDD,
								 optimal_fuel,
								 fuel_makespan,
								 final_encoding_context,
								 optimal_solution,
								 thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);       
	int real_fuel = solution_analyzer.calc_TotalFuel(optimal_solution, start_arrangement, environment_copy);

	optimal_solution.m_optimality_ratio = (double)real_fuel / final_encoding_context.m_max_total_fuel;

 	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_FuelOptimalSolution(Glucose::Solver                                **solver,
									 const sRobotArrangement                        &start_arrangement,
									 const sRobotGoal                               &final_arrangement,
									 const sUndirectedGraph                         &environment,
									 const sUndirectedGraph                         &sparse_environment,
									 const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
									 int                                             max_total_fuel,
									 int                                            &optimal_fuel,
									 int                                            &fuel_makespan,
									 sMultirobotSolution                            &optimal_solution,
									 int                                             thread_id)
    {
	sResult result;

	int expansion_count;
	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);
	
	result = incompute_OptimalFuel(solver, instance, max_total_fuel, optimal_fuel, fuel_makespan, expansion_count, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}	
	if (optimal_fuel != MAKESPAN_UNDEFINED)
	{	    	   
	    switch (m_encoding)
	    {
	    case ENCODING_MDD_plus_plus_fuel:
	    {
		result = intract_ComputedMddPlusPlusFuelSolution(*solver,
								 start_arrangement,
								 environment,
								 instance.m_the_MDD,
								 optimal_fuel,
								 fuel_makespan,
								 final_encoding_context,
								 optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);       
	int real_fuel = solution_analyzer.calc_TotalFuel(optimal_solution, start_arrangement, environment_copy);

	optimal_solution.m_optimality_ratio = (double)real_fuel / final_encoding_context.m_max_total_fuel;

 	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolution_avoid(const sRobotArrangement                        &start_arrangement,
									     const sRobotGoal                               &final_arrangement,
									     const sUndirectedGraph                         &environment,
									     const sUndirectedGraph                         &sparse_environment,
									     const Arrangements_vector                      &blocked_solution,
									     int                                             optimal_cost,
									     sMultirobotSolution                            &optimal_solution,
									     int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);
		
	result = compute_OptimalCost_avoid(instance, blocked_solution, optimal_cost, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_AD_MDD:
	    {		
		result = extract_ComputedMddSolution(start_arrangement,
						     environment,
						     instance.m_the_reduced_MDD,
						     optimal_cost,
						     final_encoding_context,
						     optimal_solution,
						     thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_WATER_MDD:
	    {		
		result = extract_ComputedWaterMddSolution(start_arrangement,
							  environment,
							  instance.m_the_reduced_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    
	    case ENCODING_AD_MDD_plus:
	    {		
		result = extract_ComputedMddPlusSolution(start_arrangement,
							 environment,
							 instance.m_the_reduced_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution,
							 thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_MDD_plus_plus:
	    {		
		result = extract_ComputedMddPlusPlusSolution(start_arrangement,
							     environment,
							     instance.m_the_reduced_MDD,
							     optimal_cost,
							     final_encoding_context,
							     optimal_solution,
							     thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_MDD_star:
	    {		
		result = extract_ComputedMddStarSolution(start_arrangement,
							 environment,
							 instance.m_the_reduced_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution,
							 thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	else
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolution_avoid(Glucose::Solver                                **solver,
									       const sRobotArrangement                        &start_arrangement,
									       const sRobotGoal                               &final_arrangement,
									       const sUndirectedGraph                         &environment,
									       const sUndirectedGraph                         &sparse_environment,
									       const Arrangements_vector                      &blocked_solution,
									       int                                             optimal_cost,
									       sMultirobotSolution                            &optimal_solution,
									       int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (*solver != NULL)
	{
	    delete *solver;
	}
	*solver = new Glucose::Solver;	
		
	result = incompute_OptimalCost_avoid(solver, instance, blocked_solution, optimal_cost, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_AD_MDD:
	    {		
		result = intract_ComputedMddSolution(*solver,
						     start_arrangement,
						     environment,
						     instance.m_the_reduced_MDD,
						     optimal_cost,
						     final_encoding_context,
						     optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_WATER_MDD:
	    {		
		result = intract_ComputedWaterMddSolution(*solver,
							  start_arrangement,
							  environment,
							  instance.m_the_reduced_MDD,
							  optimal_cost,
							  final_encoding_context,
							  optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    
	    case ENCODING_AD_MDD_plus:
	    {		
		result = intract_ComputedMddPlusSolution(*solver,
							 start_arrangement,
							 environment,
							 instance.m_the_reduced_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_MDD_plus_plus:
	    {		
		result = intract_ComputedMddPlusPlusSolution(*solver,
							     start_arrangement,
							     environment,
							     instance.m_the_reduced_MDD,
							     optimal_cost,
							     final_encoding_context,
							     optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_AD_MDD_star:
	    {		
		result = intract_ComputedMddStarSolution(*solver,
							 start_arrangement,
							 environment,
							 instance.m_the_reduced_MDD,
							 optimal_cost,
							 final_encoding_context,
							 optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	else
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_MakespanOptimalSolution_avoid(const sRobotArrangement                        &start_arrangement,
										 const sRobotGoal                               &final_arrangement,
										 const sUndirectedGraph                         &environment,
										 const sUndirectedGraph                         &sparse_environment,
										 const Arrangements_vector                      &blocked_solution,
										 int                                             optimal_makespan,
										 sMultirobotSolution                            &optimal_solution,
										 int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_OptimalMakespan_avoid(instance, blocked_solution, optimal_makespan, final_encoding_context, thread_id);
		
	if (sFAILED(result))
	{
	    return result;
	}
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_MMDD:
	    {		
		result = extract_ComputedMmddSolution(start_arrangement,
						      environment,
						      instance.m_the_reduced_MDD,
						      optimal_makespan - 1,
						      final_encoding_context,
						      optimal_solution,
						      thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_MMDD_plus:
	    {		
		result = extract_ComputedMmddPlusSolution(start_arrangement,
							  environment,
							  instance.m_the_reduced_MDD,
							  optimal_makespan - 1,
							  final_encoding_context,
							  optimal_solution,
							  thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = extract_ComputedMmddPlusPlusSolution(start_arrangement,
							      environment,
							      instance.m_the_reduced_MDD,
							      optimal_makespan - 1,
							      final_encoding_context,
							      optimal_solution,
							      thread_id);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	else
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_MakespanOptimalSolution_avoid(Glucose::Solver                                **solver,
										   const sRobotArrangement                        &start_arrangement,
										   const sRobotGoal                               &final_arrangement,
										   const sUndirectedGraph                         &environment,
										   const sUndirectedGraph                         &sparse_environment,
										   const Arrangements_vector                      &blocked_solution,
										   int                                             optimal_makespan,
										   sMultirobotSolution                            &optimal_solution,
										   int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	if (*solver != NULL)
	{
	    delete *solver;
	}
	*solver = new Glucose::Solver;	

	result = incompute_OptimalMakespan_avoid(solver, instance, blocked_solution, optimal_makespan, final_encoding_context, thread_id);
		
	if (sFAILED(result))
	{
	    return result;
	}
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_MMDD:
	    {		
		result = intract_ComputedMmddSolution(*solver,
						      start_arrangement,
						      environment,
						      instance.m_the_reduced_MDD,
						      optimal_makespan - 1,
						      final_encoding_context,
						      optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_MMDD_plus:
	    {		
		result = intract_ComputedMmddPlusSolution(*solver,
							  start_arrangement,
							  environment,
							  instance.m_the_reduced_MDD,
							  optimal_makespan - 1,
							  final_encoding_context,
							  optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		result = intract_ComputedMmddPlusPlusSolution(*solver,
							      start_arrangement,
							      environment,
							      instance.m_the_reduced_MDD,
							      optimal_makespan - 1,
							      final_encoding_context,
							      optimal_solution);
		
		if (sFAILED(result))
		{
		    return result;
		}
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }	    	    
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}
	else
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }            


    typedef std::vector<int> RobotIDs_vector;
    typedef std::vector<RobotIDs_vector> RobotGroups_vector;
    typedef std::vector<int> SolutionCosts_vector;
    typedef std::vector<int> SolutionMakespans_vector;    

    void merge_RobotGroups(int group_A_idx, int group_B_idx, RobotGroups_vector &robot_Groups)
    {
	RobotIDs_vector &robot_group_A = robot_Groups[group_A_idx];
	RobotIDs_vector &robot_group_B = robot_Groups[group_B_idx];

	for (int i = 0; i < robot_group_B.size(); ++i)
	{
	    robot_group_A.push_back(robot_group_B[i]);
	}
	robot_group_B = *robot_Groups.rbegin();
	robot_Groups.pop_back();
    }


    void initialize_RobotGroups(int N_Robots, RobotGroups_vector &robot_Groups)
    {
	for (int r = 1; r <= N_Robots; ++r)
	{
	    RobotIDs_vector robot_group;
	    robot_group.push_back(r);
	    robot_Groups.push_back(robot_group);
	}
    }


    void construct_InstanceFromGroup(int                       group_idx,
				     const RobotGroups_vector &robot_Groups,
				     const sUndirectedGraph   &environment,
				     const sRobotArrangement  &start_arrangement,
				     const sRobotGoal         &final_arrangement,
				     sRobotArrangement        &group_start_arrangement,
				     sRobotGoal               &group_final_arrangement)
    {
	const RobotIDs_vector &robot_group = robot_Groups[group_idx];

	group_start_arrangement = sRobotArrangement(environment.get_VertexCount(), robot_group.size());
	group_final_arrangement = sRobotGoal(environment.get_VertexCount(), robot_group.size());

	for (int i = 0; i < robot_group.size(); ++i)
	{
	    group_start_arrangement.place_Robot(i + 1, start_arrangement.get_RobotLocation(robot_group[i]));
	    sASSERT(final_arrangement.get_RobotGoal(robot_group[i]).size() == 1);
	    group_final_arrangement.charge_Robot(i + 1, *final_arrangement.get_RobotGoal(robot_group[i]).begin());
	}
    }


    void construct_InstanceFromGroupComplement(int                       group_idx,
					       const RobotGroups_vector &robot_Groups,
					       const sUndirectedGraph   &environment,
					       const sRobotArrangement  &start_arrangement,
					       const sRobotGoal         &final_arrangement,
					       sRobotArrangement        &complement_start_arrangement,
					       sRobotGoal               &complement_final_arrangement)
    {
	int N_Robots = 0;
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    N_Robots += robot_Groups[g].size();
	}
	N_Robots -= robot_Groups[group_idx].size();
	
	complement_start_arrangement = sRobotArrangement(environment.get_VertexCount(), N_Robots);
	complement_final_arrangement = sRobotGoal(environment.get_VertexCount(), N_Robots);
	    
	int robot_id = 1;
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    if (g != group_idx)
	    {
		const RobotIDs_vector &robot_group = robot_Groups[g];

		for (int i = 0; i < robot_group.size(); ++i)
		{
		    complement_start_arrangement.place_Robot(robot_id, start_arrangement.get_RobotLocation(robot_group[i]));
		    sASSERT(final_arrangement.get_RobotGoal(robot_group[i]).size() == 1);
		    complement_final_arrangement.charge_Robot(robot_id, *final_arrangement.get_RobotGoal(robot_group[i]).begin());

		    ++robot_id;
		}	    
	    }
	}
    }    


    bool check_GroupIntersection(const sRobotArrangement &A_start_arrangement, const sRobotArrangement &B_start_arrangement)
    {
	int A_group_N_Robots = A_start_arrangement.get_RobotCount();

	for (int r = 1; r <= A_group_N_Robots; ++r)
	{
	    int robot_location_id = A_start_arrangement.get_RobotLocation(r);

	    if (B_start_arrangement.get_VertexOccupancy(robot_location_id) != sRobotArrangement::VACANT_VERTEX)
	    {
		return true;
	    }
	}
	return false;
    }


    bool check_GroupDependence(const sRobotArrangement   &A_start_arrangement,
			       const sRobotArrangement   &B_start_arrangement,
			       const sMultirobotSolution &group_A_solution,
			       const sMultirobotSolution &group_B_solution)
    {
	sRobotArrangement A_current_arrangement = A_start_arrangement;
	sRobotArrangement B_current_arrangement = B_start_arrangement;

	int step_A = 0;
	int step_B = 0;

	if (group_A_solution.get_StepCount() < group_B_solution.get_StepCount())
	{
	    while (step_A < group_A_solution.get_StepCount())
	    {
		if (   group_A_solution.check_Step(B_current_arrangement, step_A)
		    && group_B_solution.check_Step(A_current_arrangement, step_B))
		{
		    group_A_solution.execute_Step(A_current_arrangement, step_A++);
		    group_B_solution.execute_Step(B_current_arrangement, step_B++);

		    if (check_GroupIntersection(A_current_arrangement, B_current_arrangement))
		    {
			return true;
		    }
		}
		else
		{
		    return true;
		}
	    }
	    while (step_B < group_B_solution.get_StepCount())
	    {
		if (group_B_solution.check_Step(A_current_arrangement, step_B))
		{
		    group_B_solution.execute_Step(B_current_arrangement, step_B++);

		    if (check_GroupIntersection(A_current_arrangement, B_current_arrangement))
		    {
			return true;
		    }
		}
		else
		{
		    return true;
		}
	    }
	}
	else
	{
	    if (group_A_solution.get_StepCount() > group_B_solution.get_StepCount())
	    {
		while (step_B < group_B_solution.get_StepCount())
		{
		    if (   group_A_solution.check_Step(B_current_arrangement, step_A)
			&& group_B_solution.check_Step(A_current_arrangement, step_B))
		    {
			group_A_solution.execute_Step(A_current_arrangement, step_A++);
			group_B_solution.execute_Step(B_current_arrangement, step_B++);
			
			if (check_GroupIntersection(A_current_arrangement, B_current_arrangement))
			{
			    return true;
			}
		    }
		    else
		    {
			return true;
		    }
		}
		while (step_A < group_A_solution.get_StepCount())
		{
		    if (group_A_solution.check_Step(B_current_arrangement, step_A))
		    {
			group_A_solution.execute_Step(A_current_arrangement, step_A++);
			
			if (check_GroupIntersection(A_current_arrangement, B_current_arrangement))
		{
			    return true;
			}
		    }
		    else
		    {
			return true;
		    }
		}
	    }
	    else
	    {
		while (step_A < group_A_solution.get_StepCount())
		{
		    if (   group_A_solution.check_Step(B_current_arrangement, step_A)
			&& group_B_solution.check_Step(A_current_arrangement, step_B))
		    {
			group_A_solution.execute_Step(A_current_arrangement, step_A++);
			group_B_solution.execute_Step(B_current_arrangement, step_B++);

			if (check_GroupIntersection(A_current_arrangement, B_current_arrangement))
			{
			    return true;
			}
		    }
		    else
		    {
			return true;
		    }
		}
	    }
	}
	return false;
    }

    
    void print_RobotGroups(const RobotGroups_vector &robot_Groups)
    {
	printf("Robot groups:\n");
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    printf("%d:  { ", g);
	    for (int i = 0; i < robot_Groups[g].size(); ++i)
	    {
		printf("%d ", robot_Groups[g][i]);
	    }
	    printf("}\n");
	}
    }


    typedef std::vector<sRobotArrangement> RobotArrangements_vector;
    typedef std::vector<sRobotGoal> RobotGoals_vector;
    typedef std::vector<sMultirobotSolution> MultirobotSolutions_vector;

    typedef std::vector<int> AdjacencyRow_vector;
    typedef std::vector<AdjacencyRow_vector> AdjacencyMatrix_vector;

    void initialize_AdjacencyMatrix(int n, AdjacencyMatrix_vector &adjacency_Matrix)
    {
	adjacency_Matrix.resize(n);

	for (int i = 0; i < n; ++i)
	{
	    adjacency_Matrix[i].resize(n, 0);
	}
    }


    bool is_Adjacent(int x, int y, const AdjacencyMatrix_vector &adjacency_Matrix)
    {
	return (adjacency_Matrix[x][y] != 0);
    }


    void set_Adjacent(int x, int y, int value, AdjacencyMatrix_vector &adjacency_Matrix)
    {
	adjacency_Matrix[x][y] = adjacency_Matrix[y][x] = value;
    }


    typedef std::vector<int> Indicators_vector;

    bool check_RobotGroups(int N_robots, const RobotGroups_vector &robot_Groups)
    {
	Indicators_vector robot_Indicators;
	robot_Indicators.resize(N_robots + 1, 0);

	for (RobotGroups_vector::const_iterator robot_group = robot_Groups.begin(); robot_group != robot_Groups.end(); ++robot_group)
	{
	    for (int i = 0; i < robot_group->size(); ++i)
	    {
		if (robot_Indicators[(*robot_group)[i]] == 0)
		{
		    robot_Indicators[(*robot_group)[i]] = 1;
		}
		else
		{
		    return false;
		}
	    }
	}
	return true;
    }

    bool merge_RobotGroups(const RobotGroups_vector         &robot_Groups,
			   const MultirobotSolutions_vector &group_optimal_Solutions,
			   const AdjacencyMatrix_vector     &adjacency_Matrix,
			   RobotGroups_vector               &merged_robot_Groups,
			   MultirobotSolutions_vector       &merged_group_optimal_Solutions)
    {
	bool merged = false;

	Indicators_vector merge_Indicators;
	merge_Indicators.resize(robot_Groups.size(), 0);

	for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	{
	    for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
	    {
		if (merge_Indicators[gA] == 0 && merge_Indicators[gB] == 0)
		{
		    if (is_Adjacent(gA, gB, adjacency_Matrix))
		    {
			RobotIDs_vector merged_robot_group;

			const RobotIDs_vector &robot_group_A = robot_Groups[gA];
			const RobotIDs_vector &robot_group_B = robot_Groups[gB];

			for (int ra = 0; ra < robot_group_A.size(); ++ra)
			{
			    merged_robot_group.push_back(robot_group_A[ra]);
			}
			for (int rb = 0; rb < robot_group_B.size(); ++rb)
			{
			    merged_robot_group.push_back(robot_group_B[rb]);
			}
			merged_robot_Groups.push_back(merged_robot_group);
			merged_group_optimal_Solutions.push_back(sMultirobotSolution());

			merge_Indicators[gA] = merge_Indicators[gB] = 1;

			merged = true;
		    }
		}
	    }
	}
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    if (merge_Indicators[g] == 0)
	    {
		merged_robot_Groups.push_back(robot_Groups[g]);
		merged_group_optimal_Solutions.push_back(group_optimal_Solutions[g]);
	    }
	}

	return merged;
    }


    void merge_RobotGroups(int                         gA,
			   int                         gB,
			   RobotGroups_vector         &robot_Groups,
			   MultirobotSolutions_vector &group_optimal_Solutions,
			   SolutionCosts_vector       &group_optimal_Costs)
    {
	sASSERT(gA < gB);

	RobotIDs_vector &robot_group_A = robot_Groups[gA];
	RobotIDs_vector &robot_group_B = robot_Groups[gB];
	
	for (int rb = 0; rb < robot_group_B.size(); ++rb)
	{
	    robot_group_A.push_back(robot_group_B[rb]);
	}
	robot_Groups[gB] = *robot_Groups.rbegin();
	robot_Groups.pop_back();
	
	group_optimal_Solutions[gA] = sMultirobotSolution();
	group_optimal_Costs[gA] = -1;

	group_optimal_Solutions[gB] = *group_optimal_Solutions.rbegin();
	
	group_optimal_Solutions.pop_back();

	group_optimal_Costs[gB] = *group_optimal_Costs.rbegin();
	group_optimal_Costs.pop_back();
    }
    

    bool merge_RobotGroups_(const RobotGroups_vector         &robot_Groups,
			    const MultirobotSolutions_vector &group_optimal_Solutions,
			    const AdjacencyMatrix_vector     &adjacency_Matrix,
			    RobotGroups_vector               &merged_robot_Groups,
			    MultirobotSolutions_vector       &merged_group_optimal_Solutions)
    {
	bool merged = false;

	Indicators_vector merge_Colors;
	merge_Colors.resize(robot_Groups.size(), 0);

	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    merge_Colors[g] = g;
	}

	for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	{
	    for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
	    {
		if (is_Adjacent(gA, gB, adjacency_Matrix))
		{
		    for (int g = 0; g < robot_Groups.size(); ++g)
		    {
			if (merge_Colors[g] == merge_Colors[gB])
			{
			    merge_Colors[g] = merge_Colors[gA];
			}
		    }
		}
	    }
	}

#ifdef sVERBOSE
	printf("MERGE colors\n");
	for (int c = 0; c < robot_Groups.size(); ++c)
	{
	    printf("  %d: %d\n", c, merge_Colors[c]);
	}
#endif

	for (int c = 0; c < robot_Groups.size(); ++c)
	{
	    int group_size = 0;
	    int single_group_idx;
	    RobotIDs_vector merged_robot_group;

	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
		if (merge_Colors[g] == c)
		{
		    for (int ra = 0; ra < robot_Groups[g].size(); ++ra)
		    {
			merged_robot_group.push_back(robot_Groups[g][ra]);
		    }
		    ++group_size;
		    single_group_idx = g;
		}
	    }
	    if (group_size > 0)
	    {
		if (group_size == 1)
		{
		    merged_robot_Groups.push_back(robot_Groups[single_group_idx ]);
		    merged_group_optimal_Solutions.push_back(group_optimal_Solutions[single_group_idx ]);
		}
		else
		{
		    merged_robot_Groups.push_back(merged_robot_group);
		    merged_group_optimal_Solutions.push_back(sMultirobotSolution());

		    merged = true;
		}
	    }
	}

#ifdef sVERBOSE
	printf("PRE-MERGE\n");
	print_RobotGroups(robot_Groups);
	printf("POST-MERGE\n");
	print_RobotGroups(merged_robot_Groups);
#endif

	return merged;
    }


    void merge_GroupRobotSolutions(const RobotGroups_vector &robot_Groups, const MultirobotSolutions_vector &group_optimal_Solutions, sMultirobotSolution &multirobot_solution)
    {
	int base_robot_id = 1;
		
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    int time_step = 0;
	    for (sMultirobotSolution::Steps_vector::const_iterator step = group_optimal_Solutions[g].m_Steps.begin(); step != group_optimal_Solutions[g].m_Steps.end(); ++step)
	    {
		for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		{
		    /*
		    multirobot_solution.add_Move(time_step, sMultirobotSolution::Move(robot_Groups[g][move->m_robot_id - 1] + base_robot_id,
										      move->m_src_vrtx_id,
										      move->m_dest_vrtx_id));
		    */
		    multirobot_solution.add_Move(time_step, sMultirobotSolution::Move(move->m_robot_id + base_robot_id - 1,
										      move->m_src_vrtx_id,
										      move->m_dest_vrtx_id));

		}
		++time_step;
	    }
	    base_robot_id += robot_Groups[g].size();
	}
    }


    void merge_GroupRobotSolutionsComplement(int excluded_gID, const RobotGroups_vector &robot_Groups, const MultirobotSolutions_vector &group_optimal_Solutions, sMultirobotSolution &multirobot_solution)
    {
	int base_robot_id = 1;
	
	for (int g = 0; g < robot_Groups.size(); ++g)
	{
	    if (g != excluded_gID)
	    {
		int time_step = 0;
		for (sMultirobotSolution::Steps_vector::const_iterator step = group_optimal_Solutions[g].m_Steps.begin(); step != group_optimal_Solutions[g].m_Steps.end(); ++step)
		{
		    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
		    {
			/*
			multirobot_solution.add_Move(time_step, sMultirobotSolution::Move(robot_Groups[g][move->m_robot_id - 1] + base_robot_id,
											  move->m_src_vrtx_id,
											  move->m_dest_vrtx_id));
			*/
			multirobot_solution.add_Move(time_step, sMultirobotSolution::Move(move->m_robot_id + base_robot_id - 1,
											  move->m_src_vrtx_id,
											  move->m_dest_vrtx_id));			
		    }
		    ++time_step;
		}
		base_robot_id += robot_Groups[g].size();
	    }
	}
    }    


    void compute_SingleCostOptimalSolution(const sRobotArrangement &group_start_arrangement,
					   const sRobotGoal        &group_final_arrangement,
					   sUndirectedGraph        &environment,
					   int                     &group_optimal_cost,
					   sMultirobotSolution     &group_optimal_solution)
    {
	sUndirectedGraph::VertexIDs_vector shortest_Path;
	sASSERT(group_final_arrangement.get_RobotGoal(1).size() == 1);

	environment.find_ShortestPathBreadth(group_start_arrangement.get_RobotLocation(1), *group_final_arrangement.get_RobotGoal(1).begin(), shortest_Path);

	group_optimal_cost = 0;
	for (int i = shortest_Path.size() - 1; i > 0; --i)
	{
	    group_optimal_solution.add_Move(shortest_Path.size() - i - 1, sMultirobotSolution::Move(1, shortest_Path[i], shortest_Path[i-1]));
	    ++group_optimal_cost;
	}
    }


    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolutionID(const sRobotArrangement                        &start_arrangement,
									 const sRobotGoal                               &final_arrangement,
									 const sUndirectedGraph                         &environment,
									 const sUndirectedGraph                         &sparse_environment,
									 const sMultirobotInstance::MDD_vector          &MDD,
									 int                                             max_total_cost,
									 int                                            &optimal_cost,
									 sMultirobotSolution                            &optimal_solution,
									 int                                             thread_id)
    {
	sResult result;

	optimal_cost = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged;
	MultirobotSolutions_vector group_optimal_Solutions;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());

	do
	{
	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		printf("Initiating instance for group %d\n", g);
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = compute_CostOptimalSolution(group_start_arrangement,
							     group_final_arrangement,
							     environment,
							     sparse_environment,
							     MDD,
							     max_total_cost,
							     group_optimal_cost,
							     group_optimal_solution,
							     thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
			
#ifdef sVERBOSE
		    printf("Computed group solution cost: %d\n", group_optimal_cost);
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif

	    AdjacencyMatrix_vector adjacency_Matrix;
	    initialize_AdjacencyMatrix(robot_Groups.size(), adjacency_Matrix);

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{
#ifdef sVERBOSE
		    printf("Checking dependence between groups: %d x %d\n", gA, gB);
		    group_optimal_Solutions[gA].to_Screen();
		    group_optimal_Solutions[gB].to_Screen();
#endif
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			set_Adjacent(gA, gB, 1, adjacency_Matrix);
		    }
		}
	    }
	    RobotGroups_vector merged_robot_Groups;
	    merged = merge_RobotGroups_(robot_Groups,
					group_optimal_Solutions,
					adjacency_Matrix,
					merged_robot_Groups,
					merged_group_optimal_Solutions);
	    sASSERT(check_RobotGroups(N_Robots, merged_robot_Groups));

#ifdef sVERBOSE
	    print_RobotGroups(merged_robot_Groups);
#endif

	    robot_Groups = merged_robot_Groups;
	    group_optimal_Solutions = merged_group_optimal_Solutions;
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);
	
	optimal_solution.to_Screen();
	print_RobotGroups(robot_Groups);

	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);
	optimal_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolutionID(Glucose::Solver                                **solver,
									   const sRobotArrangement                        &start_arrangement,
									   const sRobotGoal                               &final_arrangement,
									   const sUndirectedGraph                         &environment,
									   const sUndirectedGraph                         &sparse_environment,
									   const sMultirobotInstance::MDD_vector          &MDD,
									   int                                             max_total_cost,
									   int                                            &optimal_cost,
									   sMultirobotSolution                            &optimal_solution,
									   int                                             thread_id)
    {
	sResult result;

	optimal_cost = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged;
	MultirobotSolutions_vector group_optimal_Solutions;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());

	do
	{
	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		printf("Initiating instance for group %d\n", g);
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = incompute_CostOptimalSolution(solver,
							       group_start_arrangement,
							       group_final_arrangement,
							       environment,
							       sparse_environment,
							       MDD,
							       max_total_cost,
							       group_optimal_cost,
							       group_optimal_solution,
							       thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
			
#ifdef sVERBOSE
		    printf("Computed group solution cost: %d\n", group_optimal_cost);
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif

	    AdjacencyMatrix_vector adjacency_Matrix;
	    initialize_AdjacencyMatrix(robot_Groups.size(), adjacency_Matrix);

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{
#ifdef sVERBOSE
		    printf("Checking dependence between groups: %d x %d\n", gA, gB);
		    group_optimal_Solutions[gA].to_Screen();
		    group_optimal_Solutions[gB].to_Screen();
#endif
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			set_Adjacent(gA, gB, 1, adjacency_Matrix);
		    }
		}
	    }
	    RobotGroups_vector merged_robot_Groups;
	    merged = merge_RobotGroups_(robot_Groups,
					group_optimal_Solutions,
					adjacency_Matrix,
					merged_robot_Groups,
					merged_group_optimal_Solutions);
	    sASSERT(check_RobotGroups(N_Robots, merged_robot_Groups));

#ifdef sVERBOSE
	    print_RobotGroups(merged_robot_Groups);
#endif

	    robot_Groups = merged_robot_Groups;
	    group_optimal_Solutions = merged_group_optimal_Solutions;
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);
	
	optimal_solution.to_Screen();
	print_RobotGroups(robot_Groups);

	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);
	optimal_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::resolve_GroupDependency(RobotGroups_vector         &robot_Groups,
								   int                         group_A_idx,
								   int                         group_B_idx,
								   const sRobotArrangement    &start_arrangement,
								   const sRobotGoal           &final_arrangement,
								   const sUndirectedGraph     &environment,
								   const sUndirectedGraph     &sparse_environment,
								   const sMultirobotInstance::MDD_vector &MDD,
								   int                         max_total_cost,								   
								   RobotArrangements_vector   &group_start_Arrangements,
								   RobotGoals_vector          &group_final_Arrangements,
								   MultirobotSolutions_vector &group_optimal_Solutions,
								   SolutionCosts_vector       &group_optimal_Costs,
								   int                         thread_id)
    {
	sResult result;
	
	sRobotArrangement complement_start_arrangement;
	sRobotGoal complement_final_arrangement;
	
	construct_InstanceFromGroupComplement(group_A_idx,
					      robot_Groups,
					      environment,
					      start_arrangement,
					      final_arrangement,
					      complement_start_arrangement,
					      complement_final_arrangement);
	
	sMultirobotSolution blocking_solution;
	merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, blocking_solution);
	
	Arrangements_vector unfolded_blocking_Arrangements;       
	force_Solution(complement_start_arrangement, blocking_solution, unfolded_blocking_Arrangements);
				
#ifdef sVEROBE
	{
	    printf("Group optimal costs:\n");
	    for (int g = 0; g < group_optimal_Costs.size(); ++g)
	    {
		printf("%d\n", group_optimal_Costs[g]);
	    }
	}
#endif
		
	sMultirobotSolution avoiding_optimal_solution;
	result = compute_CostOptimalSolution_avoid(group_start_Arrangements[group_A_idx],
						   group_final_Arrangements[group_A_idx],
						   environment,
						   sparse_environment,
						   unfolded_blocking_Arrangements,
						   group_optimal_Costs[group_A_idx],
						   avoiding_optimal_solution,
						   thread_id);	    

	if (sFAILED(result))
	{
	    return result;
	}	
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    group_optimal_Solutions[group_A_idx] = avoiding_optimal_solution;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	{
	    merge_RobotGroups(group_A_idx, group_B_idx, robot_Groups, group_optimal_Solutions, group_optimal_Costs);

	    construct_InstanceFromGroup(group_A_idx,
					robot_Groups,
					environment,
					start_arrangement,
					final_arrangement,
					group_start_Arrangements[group_A_idx],
					group_final_Arrangements[group_A_idx]);

	    group_start_Arrangements[group_B_idx] = *group_start_Arrangements.rbegin();
	    group_start_Arrangements.pop_back();
	    group_final_Arrangements[group_B_idx] = *group_final_Arrangements.rbegin();
	    group_final_Arrangements.pop_back();
	    	

#ifdef sVERBOSE
	    {
		print_RobotGroups(robot_Groups);
	    }
#endif

	    sRobotArrangement complement_start_arrangement;
	    sRobotGoal complement_final_arrangement;
	
	    construct_InstanceFromGroupComplement(group_A_idx,
						  robot_Groups,
						  environment,
						  start_arrangement,
						  final_arrangement,
						  complement_start_arrangement,
						  complement_final_arrangement);
	
	    sMultirobotSolution merge_blocking_solution;
	    merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, merge_blocking_solution);

	    Arrangements_vector unfolded_merge_blocking_Arrangements;
	
//				sASSERT(verify_Unfolding(complement_start_arrangement, blocking_solution, environment));
	    force_Solution(complement_start_arrangement, merge_blocking_solution, unfolded_merge_blocking_Arrangements);

	    int group_optimal_cost;
	    sMultirobotSolution group_optimal_solution;       

	    result = compute_CostOptimalSolution(group_start_Arrangements[group_A_idx],
						 group_final_Arrangements[group_A_idx],
						 environment,
						 sparse_environment,
						 MDD,
						 max_total_cost,
						 group_optimal_cost,
						 group_optimal_solution,
						 thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    sMultirobotSolution merge_avoiding_optimal_solution;
	    result = compute_CostOptimalSolution_avoid(group_start_Arrangements[group_A_idx],
						       group_final_Arrangements[group_A_idx],
						       environment,
						       sparse_environment,
						       unfolded_merge_blocking_Arrangements,
						       group_optimal_cost,
						       merge_avoiding_optimal_solution,
						       thread_id);

	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = merge_avoiding_optimal_solution;
		group_optimal_Costs[group_A_idx] = group_optimal_cost;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = group_optimal_solution;
		group_optimal_Costs[group_A_idx] = group_optimal_cost;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
		
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    break;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::inresolve_GroupDependency(Glucose::Solver            **solver,
								     RobotGroups_vector         &robot_Groups,
								     int                         group_A_idx,
								     int                         group_B_idx,
								     const sRobotArrangement    &start_arrangement,
								     const sRobotGoal           &final_arrangement,
								     const sUndirectedGraph     &environment,
								     const sUndirectedGraph     &sparse_environment,
								     const sMultirobotInstance::MDD_vector &MDD,
								     int                         max_total_cost,								   
								     RobotArrangements_vector   &group_start_Arrangements,
								     RobotGoals_vector          &group_final_Arrangements,
								     MultirobotSolutions_vector &group_optimal_Solutions,
								     SolutionCosts_vector       &group_optimal_Costs,
								     int                         thread_id)
    {
	sResult result;
	
	sRobotArrangement complement_start_arrangement;
	sRobotGoal complement_final_arrangement;
	
	construct_InstanceFromGroupComplement(group_A_idx,
					      robot_Groups,
					      environment,
					      start_arrangement,
					      final_arrangement,
					      complement_start_arrangement,
					      complement_final_arrangement);
	
	sMultirobotSolution blocking_solution;
	merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, blocking_solution);
	
	Arrangements_vector unfolded_blocking_Arrangements;       
	force_Solution(complement_start_arrangement, blocking_solution, unfolded_blocking_Arrangements);
				
#ifdef sVEROBE
	{
	    printf("Group optimal costs:\n");
	    for (int g = 0; g < group_optimal_Costs.size(); ++g)
	    {
		printf("%d\n", group_optimal_Costs[g]);
	    }
	}
#endif
		
	sMultirobotSolution avoiding_optimal_solution;
	result = incompute_CostOptimalSolution_avoid(solver,
						     group_start_Arrangements[group_A_idx],
						     group_final_Arrangements[group_A_idx],
						     environment,
						     sparse_environment,
						     unfolded_blocking_Arrangements,
						     group_optimal_Costs[group_A_idx],
						     avoiding_optimal_solution,
						     thread_id);	    

	if (sFAILED(result))
	{
	    return result;
	}	
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    group_optimal_Solutions[group_A_idx] = avoiding_optimal_solution;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	{
	    merge_RobotGroups(group_A_idx, group_B_idx, robot_Groups, group_optimal_Solutions, group_optimal_Costs);

	    construct_InstanceFromGroup(group_A_idx,
					robot_Groups,
					environment,
					start_arrangement,
					final_arrangement,
					group_start_Arrangements[group_A_idx],
					group_final_Arrangements[group_A_idx]);

	    group_start_Arrangements[group_B_idx] = *group_start_Arrangements.rbegin();
	    group_start_Arrangements.pop_back();
	    group_final_Arrangements[group_B_idx] = *group_final_Arrangements.rbegin();
	    group_final_Arrangements.pop_back();
	    	

#ifdef sVERBOSE
	    {
		print_RobotGroups(robot_Groups);
	    }
#endif

	    sRobotArrangement complement_start_arrangement;
	    sRobotGoal complement_final_arrangement;
	
	    construct_InstanceFromGroupComplement(group_A_idx,
						  robot_Groups,
						  environment,
						  start_arrangement,
						  final_arrangement,
						  complement_start_arrangement,
						  complement_final_arrangement);
	
	    sMultirobotSolution merge_blocking_solution;
	    merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, merge_blocking_solution);

	    Arrangements_vector unfolded_merge_blocking_Arrangements;
	
//				sASSERT(verify_Unfolding(complement_start_arrangement, blocking_solution, environment));
	    force_Solution(complement_start_arrangement, merge_blocking_solution, unfolded_merge_blocking_Arrangements);

	    int group_optimal_cost;
	    sMultirobotSolution group_optimal_solution;       

	    result = incompute_CostOptimalSolution(solver,
						   group_start_Arrangements[group_A_idx],
						   group_final_Arrangements[group_A_idx],
						   environment,
						   sparse_environment,
						   MDD,
						   max_total_cost,
						   group_optimal_cost,
						   group_optimal_solution,
						   thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    sMultirobotSolution merge_avoiding_optimal_solution;
	    result = incompute_CostOptimalSolution_avoid(solver,
							 group_start_Arrangements[group_A_idx],
							 group_final_Arrangements[group_A_idx],
							 environment,
							 sparse_environment,
							 unfolded_merge_blocking_Arrangements,
							 group_optimal_cost,
							 merge_avoiding_optimal_solution,
							 thread_id);

	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = merge_avoiding_optimal_solution;
		group_optimal_Costs[group_A_idx] = group_optimal_cost;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = group_optimal_solution;
		group_optimal_Costs[group_A_idx] = group_optimal_cost;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
		
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    break;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::resolve_MakespanGroupDependency(RobotGroups_vector         &robot_Groups,
									   int                         group_A_idx,
									   int                         group_B_idx,
									   const sRobotArrangement    &start_arrangement,
									   const sRobotGoal           &final_arrangement,
									   const sUndirectedGraph     &environment,
									   const sUndirectedGraph     &sparse_environment,
									   int                         max_makespan,								   
									   RobotArrangements_vector   &group_start_Arrangements,
									   RobotGoals_vector          &group_final_Arrangements,
									   MultirobotSolutions_vector &group_optimal_Solutions,
									   SolutionMakespans_vector   &group_optimal_Makespans,
									   int                         thread_id)
    {
	sResult result;
	
	sRobotArrangement complement_start_arrangement;
	sRobotGoal complement_final_arrangement;
	
	construct_InstanceFromGroupComplement(group_A_idx,
					      robot_Groups,
					      environment,
					      start_arrangement,
					      final_arrangement,
					      complement_start_arrangement,
					      complement_final_arrangement);
	
	sMultirobotSolution blocking_solution;
	merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, blocking_solution);
	
	Arrangements_vector unfolded_blocking_Arrangements;       
	force_Solution(complement_start_arrangement, blocking_solution, unfolded_blocking_Arrangements);
						
	sMultirobotSolution avoiding_optimal_solution;
	result = compute_MakespanOptimalSolution_avoid(group_start_Arrangements[group_A_idx],
						       group_final_Arrangements[group_A_idx],
						       environment,
						       sparse_environment,
						       unfolded_blocking_Arrangements,
						       group_optimal_Makespans[group_A_idx],
						       avoiding_optimal_solution,
						       thread_id);

	if (sFAILED(result))
	{
	    return result;
	}	
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    group_optimal_Solutions[group_A_idx] = avoiding_optimal_solution;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	{
	    merge_RobotGroups(group_A_idx, group_B_idx, robot_Groups, group_optimal_Solutions, group_optimal_Makespans);

	    construct_InstanceFromGroup(group_A_idx,
					robot_Groups,
					environment,
					start_arrangement,
					final_arrangement,
					group_start_Arrangements[group_A_idx],
					group_final_Arrangements[group_A_idx]);

	    group_start_Arrangements[group_B_idx] = *group_start_Arrangements.rbegin();
	    group_start_Arrangements.pop_back();
	    group_final_Arrangements[group_B_idx] = *group_final_Arrangements.rbegin();
	    group_final_Arrangements.pop_back();
	    	

#ifdef sVERBOSE
	    {
		print_RobotGroups(robot_Groups);
	    }
#endif

	    sRobotArrangement complement_start_arrangement;
	    sRobotGoal complement_final_arrangement;
	
	    construct_InstanceFromGroupComplement(group_A_idx,
						  robot_Groups,
						  environment,
						  start_arrangement,
						  final_arrangement,
						  complement_start_arrangement,
						  complement_final_arrangement);
	
	    sMultirobotSolution merge_blocking_solution;
	    merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, merge_blocking_solution);

	    Arrangements_vector unfolded_merge_blocking_Arrangements;
	
//				sASSERT(verify_Unfolding(complement_start_arrangement, blocking_solution, environment));
	    force_Solution(complement_start_arrangement, merge_blocking_solution, unfolded_merge_blocking_Arrangements);

	    int group_optimal_makespan;
	    sMultirobotSolution group_optimal_solution;       

	    result = compute_OptimalSolution_(group_start_Arrangements[group_A_idx],
					      group_final_Arrangements[group_A_idx],
					      environment,
					      sparse_environment,
					      max_makespan,
					      group_optimal_makespan,
					      group_optimal_solution,
					      thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    sMultirobotSolution merge_avoiding_optimal_solution;
	    result = compute_MakespanOptimalSolution_avoid(group_start_Arrangements[group_A_idx],
							   group_final_Arrangements[group_A_idx],
							   environment,
							   sparse_environment,
							   unfolded_merge_blocking_Arrangements,
							   group_optimal_makespan,
							   merge_avoiding_optimal_solution,
							   thread_id);

	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = merge_avoiding_optimal_solution;
		group_optimal_Makespans[group_A_idx] = group_optimal_makespan;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;		
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    break;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::inresolve_MakespanGroupDependency(Glucose::Solver            **solver,
									     RobotGroups_vector         &robot_Groups,
									     int                         group_A_idx,
									     int                         group_B_idx,
									     const sRobotArrangement    &start_arrangement,
									     const sRobotGoal           &final_arrangement,
									     const sUndirectedGraph     &environment,
									     const sUndirectedGraph     &sparse_environment,
									     int                         max_makespan,								   
									     RobotArrangements_vector   &group_start_Arrangements,
									     RobotGoals_vector          &group_final_Arrangements,
									     MultirobotSolutions_vector &group_optimal_Solutions,
									     SolutionMakespans_vector   &group_optimal_Makespans,
									     int                         thread_id)
    {
	sResult result;
	
	sRobotArrangement complement_start_arrangement;
	sRobotGoal complement_final_arrangement;
	
	construct_InstanceFromGroupComplement(group_A_idx,
					      robot_Groups,
					      environment,
					      start_arrangement,
					      final_arrangement,
					      complement_start_arrangement,
					      complement_final_arrangement);
	
	sMultirobotSolution blocking_solution;
	merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, blocking_solution);
	
	Arrangements_vector unfolded_blocking_Arrangements;       
	force_Solution(complement_start_arrangement, blocking_solution, unfolded_blocking_Arrangements);
						
	sMultirobotSolution avoiding_optimal_solution;
	result = incompute_MakespanOptimalSolution_avoid(solver,
							 group_start_Arrangements[group_A_idx],
							 group_final_Arrangements[group_A_idx],
							 environment,
							 sparse_environment,
							 unfolded_blocking_Arrangements,
							 group_optimal_Makespans[group_A_idx],
							 avoiding_optimal_solution,
							 thread_id);

	if (sFAILED(result))
	{
	    return result;
	}	
	switch (result)
	{
	case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	{
	    group_optimal_Solutions[group_A_idx] = avoiding_optimal_solution;
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	{
	    merge_RobotGroups(group_A_idx, group_B_idx, robot_Groups, group_optimal_Solutions, group_optimal_Makespans);

	    construct_InstanceFromGroup(group_A_idx,
					robot_Groups,
					environment,
					start_arrangement,
					final_arrangement,
					group_start_Arrangements[group_A_idx],
					group_final_Arrangements[group_A_idx]);

	    group_start_Arrangements[group_B_idx] = *group_start_Arrangements.rbegin();
	    group_start_Arrangements.pop_back();
	    group_final_Arrangements[group_B_idx] = *group_final_Arrangements.rbegin();
	    group_final_Arrangements.pop_back();
	    	

#ifdef sVERBOSE
	    {
		print_RobotGroups(robot_Groups);
	    }
#endif

	    sRobotArrangement complement_start_arrangement;
	    sRobotGoal complement_final_arrangement;
	
	    construct_InstanceFromGroupComplement(group_A_idx,
						  robot_Groups,
						  environment,
						  start_arrangement,
						  final_arrangement,
						  complement_start_arrangement,
						  complement_final_arrangement);
	
	    sMultirobotSolution merge_blocking_solution;
	    merge_GroupRobotSolutionsComplement(group_A_idx, robot_Groups, group_optimal_Solutions, merge_blocking_solution);

	    Arrangements_vector unfolded_merge_blocking_Arrangements;
	
//				sASSERT(verify_Unfolding(complement_start_arrangement, blocking_solution, environment));
	    force_Solution(complement_start_arrangement, merge_blocking_solution, unfolded_merge_blocking_Arrangements);

	    int group_optimal_makespan;
	    sMultirobotSolution group_optimal_solution;       

	    result = incompute_OptimalSolution_(solver,
						group_start_Arrangements[group_A_idx],
						group_final_Arrangements[group_A_idx],
						environment,
						sparse_environment,
						max_makespan,
						group_optimal_makespan,
						group_optimal_solution,
						thread_id);
	    if (sFAILED(result))
	    {
		return result;
	    }

	    sMultirobotSolution merge_avoiding_optimal_solution;
	    result = incompute_MakespanOptimalSolution_avoid(solver,
							     group_start_Arrangements[group_A_idx],
							     group_final_Arrangements[group_A_idx],
							     environment,
							     sparse_environment,
							     unfolded_merge_blocking_Arrangements,
							     group_optimal_makespan,
							     merge_avoiding_optimal_solution,
							     thread_id);

	    switch (result)
	    {
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
	    {
		group_optimal_Solutions[group_A_idx] = merge_avoiding_optimal_solution;
		group_optimal_Makespans[group_A_idx] = group_optimal_makespan;
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;		
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	    }
	    case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    break;
	}
	case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	return sRESULT_SUCCESS;
    }        

    
    typedef std::multimap<int, int, std::greater<int>> SortedGroups_mmap;
    
    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolutionAD(const sRobotArrangement                        &start_arrangement,
									 const sRobotGoal                               &final_arrangement,
									 const sUndirectedGraph                         &environment,
									 const sUndirectedGraph                         &sparse_environment,
									 const sMultirobotInstance::MDD_vector          &MDD,
									 int                                             max_total_cost,
									 int                                            &optimal_cost,
									 sMultirobotSolution                            &optimal_solution,
									 int                                             thread_id)
    {
	sResult result;

	optimal_cost = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged = false;
	MultirobotSolutions_vector group_optimal_Solutions;
	SolutionCosts_vector group_optimal_Costs;	

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());
	group_optimal_Costs.resize(robot_Groups.size(), -1);	

	do
	{

	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		{
		    printf("Initiating instance for group %d\n", g);
		}
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		{
		    printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
		}
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = compute_CostOptimalSolution(group_start_arrangement,
							     group_final_arrangement,
							     environment,
							     sparse_environment,
							     MDD,
							     max_total_cost,
							     group_optimal_cost,
							     group_optimal_solution,
							     thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
		    group_optimal_Costs[g] = group_optimal_cost;
			
#ifdef sVERBOSE
		    {
			printf("Computed group solution cost: %d\n", group_optimal_cost);
		    }
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

	    SortedGroups_mmap sorted_Groups;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
		sorted_Groups.insert(SortedGroups_mmap::value_type(group_optimal_Solutions[g].get_StepCount(), g));
	    }
	    
	    RobotGroups_vector sort_robot_Groups;
	    SolutionCosts_vector sort_group_optimal_Costs;
	    MultirobotSolutions_vector sort_group_optimal_Solutions;
	    RobotArrangements_vector sort_group_start_Arrangements;
	    RobotGoals_vector sort_group_final_Arrangements;
	    
	    for (SortedGroups_mmap::const_iterator group = sorted_Groups.begin(); group != sorted_Groups.end(); ++group)
	    {
		sort_robot_Groups.push_back(robot_Groups[group->second]);
		sort_group_optimal_Costs.push_back(group_optimal_Costs[group->second]);
		sort_group_optimal_Solutions.push_back(group_optimal_Solutions[group->second]);
		sort_group_start_Arrangements.push_back(group_start_Arrangements[group->second]);
		sort_group_final_Arrangements.push_back(group_final_Arrangements[group->second]);
	    }
	    robot_Groups = sort_robot_Groups;
	    group_optimal_Costs = sort_group_optimal_Costs;
	    group_optimal_Solutions = sort_group_optimal_Solutions;
	    group_start_Arrangements = sort_group_start_Arrangements;
	    group_final_Arrangements = sort_group_final_Arrangements;
	    		     
#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    merged = false;

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{		    
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
			
			printf("Checking dependence between groups: %d x %d\n", gA, gB);
			group_optimal_Solutions[gA].to_Screen();
			group_optimal_Solutions[gB].to_Screen();
			
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			result = resolve_GroupDependency(robot_Groups,
							 gA,
							 gB,
							 start_arrangement,
							 final_arrangement,
							 environment,
							 sparse_environment,
							 MDD,
							 max_total_cost,
							 group_start_Arrangements,
							 group_final_Arrangements,
							 group_optimal_Solutions,
							 group_optimal_Costs,						
							 thread_id);

			if (sFAILED(result))
			{
			    return result;
			}
		       
			switch (result)
			{			    			   
			case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
			{
			    gB = robot_Groups.size();
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
			case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
			{
			    gB = robot_Groups.size();			    
			    merged = true;
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
			{
			    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		}
	    }
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);	
	optimal_solution.to_Screen();	
	print_RobotGroups(robot_Groups);

	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);
	optimal_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolutionAD(Glucose::Solver                                **solver,
									   const sRobotArrangement                        &start_arrangement,
									   const sRobotGoal                               &final_arrangement,
									   const sUndirectedGraph                         &environment,
									   const sUndirectedGraph                         &sparse_environment,
									   const sMultirobotInstance::MDD_vector          &MDD,
									   int                                             max_total_cost,
									   int                                            &optimal_cost,
									   sMultirobotSolution                            &optimal_solution,
									   int                                             thread_id)
    {
	sResult result;

	optimal_cost = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged = false;
	MultirobotSolutions_vector group_optimal_Solutions;
	SolutionCosts_vector group_optimal_Costs;	

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());
	group_optimal_Costs.resize(robot_Groups.size(), -1);	

	do
	{

	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		{
		    printf("Initiating instance for group %d\n", g);
		}
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		{
		    printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
		}
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = incompute_CostOptimalSolution(solver,
							       group_start_arrangement,
							       group_final_arrangement,
							       environment,
							       sparse_environment,
							       MDD,
							       max_total_cost,
							       group_optimal_cost,
							       group_optimal_solution,
							       thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
		    group_optimal_Costs[g] = group_optimal_cost;
			
#ifdef sVERBOSE
		    {
			printf("Computed group solution cost: %d\n", group_optimal_cost);
		    }
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

	    SortedGroups_mmap sorted_Groups;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
		sorted_Groups.insert(SortedGroups_mmap::value_type(group_optimal_Solutions[g].get_StepCount(), g));
	    }
	    
	    RobotGroups_vector sort_robot_Groups;
	    SolutionCosts_vector sort_group_optimal_Costs;
	    MultirobotSolutions_vector sort_group_optimal_Solutions;
	    RobotArrangements_vector sort_group_start_Arrangements;
	    RobotGoals_vector sort_group_final_Arrangements;
	    
	    for (SortedGroups_mmap::const_iterator group = sorted_Groups.begin(); group != sorted_Groups.end(); ++group)
	    {
		sort_robot_Groups.push_back(robot_Groups[group->second]);
		sort_group_optimal_Costs.push_back(group_optimal_Costs[group->second]);
		sort_group_optimal_Solutions.push_back(group_optimal_Solutions[group->second]);
		sort_group_start_Arrangements.push_back(group_start_Arrangements[group->second]);
		sort_group_final_Arrangements.push_back(group_final_Arrangements[group->second]);
	    }
	    robot_Groups = sort_robot_Groups;
	    group_optimal_Costs = sort_group_optimal_Costs;
	    group_optimal_Solutions = sort_group_optimal_Solutions;
	    group_start_Arrangements = sort_group_start_Arrangements;
	    group_final_Arrangements = sort_group_final_Arrangements;
	    		     
#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    merged = false;

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{		    
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
			
			printf("Checking dependence between groups: %d x %d\n", gA, gB);
			group_optimal_Solutions[gA].to_Screen();
			group_optimal_Solutions[gB].to_Screen();
			
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			result = inresolve_GroupDependency(solver,
							   robot_Groups,
							   gA,
							   gB,
							   start_arrangement,
							   final_arrangement,
							   environment,
							   sparse_environment,
							   MDD,
							   max_total_cost,
							   group_start_Arrangements,
							   group_final_Arrangements,
							   group_optimal_Solutions,
							   group_optimal_Costs,						
							   thread_id);
			
			if (sFAILED(result))
			{
			    return result;
			}
		       
			switch (result)
			{			    			   
			case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
			{
			    gB = robot_Groups.size();
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
			case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
			{
			    gB = robot_Groups.size();			    
			    merged = true;
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
			{
			    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		}
	    }
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);	
	optimal_solution.to_Screen();	
	print_RobotGroups(robot_Groups);

	sMultirobotSolutionAnalyzer solution_analyzer;
	sUndirectedGraph environment_copy(environment);
	optimal_cost = solution_analyzer.calc_TotalCost(optimal_solution, start_arrangement, environment_copy);

	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotSolutionCompressor::compute_MakespanOptimalSolutionID(const sRobotArrangement                        &start_arrangement,
									     const sRobotGoal                               &final_arrangement,
									     const sUndirectedGraph                         &environment,
									     const sUndirectedGraph                         &sparse_environment,
									     int                                             max_makespan,
									     int                                            &optimal_makespan,
									     sMultirobotSolution                            &optimal_solution,
									     int                                             thread_id)
    {
	sResult result;

	optimal_makespan = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);

	sUndirectedGraph environment_work(environment);

	bool merged;
	MultirobotSolutions_vector group_optimal_Solutions;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());

	do
	{
	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		printf("Initiating instance for group %d\n", g);
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = compute_OptimalSolution_(group_start_arrangement,
							  group_final_arrangement,
							  environment,
							  sparse_environment,
							  max_makespan,
							  group_optimal_cost,
							  group_optimal_solution,
							  thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
			
#ifdef sVERBOSE
		    printf("Computed group solution cost: %d\n", group_optimal_cost);
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    AdjacencyMatrix_vector adjacency_Matrix;
	    initialize_AdjacencyMatrix(robot_Groups.size(), adjacency_Matrix);

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{
#ifdef sVERBOSE
		    printf("Checking dependence between groups: %d x %d\n", gA, gB);
		    group_optimal_Solutions[gA].to_Screen();
		    group_optimal_Solutions[gB].to_Screen();
#endif
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			set_Adjacent(gA, gB, 1, adjacency_Matrix);
		    }
		}
	    }
	    RobotGroups_vector merged_robot_Groups;
	    merged = merge_RobotGroups_(robot_Groups,
					group_optimal_Solutions,
					adjacency_Matrix,
					merged_robot_Groups,
					merged_group_optimal_Solutions);

#ifdef sVERBOSE
	    print_RobotGroups(merged_robot_Groups);
#endif

	    robot_Groups = merged_robot_Groups;
	    group_optimal_Solutions = merged_group_optimal_Solutions;
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);
	optimal_solution.to_Screen();
	print_RobotGroups(robot_Groups);

	optimal_makespan = optimal_solution.get_StepCount();

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_MakespanOptimalSolutionID(Glucose::Solver                                **solver,
									       const sRobotArrangement                        &start_arrangement,
									       const sRobotGoal                               &final_arrangement,
									       const sUndirectedGraph                         &environment,
									       const sUndirectedGraph                         &sparse_environment,
									       int                                             max_makespan,
									       int                                            &optimal_makespan,
									       sMultirobotSolution                            &optimal_solution,
									       int                                             thread_id)
    {
	sResult result;

	optimal_makespan = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);

	sUndirectedGraph environment_work(environment);

	bool merged;
	MultirobotSolutions_vector group_optimal_Solutions;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());

	do
	{
	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		printf("Initiating instance for group %d\n", g);
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_cost;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = incompute_OptimalSolution_(solver,
							    group_start_arrangement,
							    group_final_arrangement,
							    environment,
							    sparse_environment,
							    max_makespan,
							    group_optimal_cost,
							    group_optimal_solution,
							    thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_cost < 0) // cost undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
			
#ifdef sVERBOSE
		    printf("Computed group solution cost: %d\n", group_optimal_cost);
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    AdjacencyMatrix_vector adjacency_Matrix;
	    initialize_AdjacencyMatrix(robot_Groups.size(), adjacency_Matrix);

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{
#ifdef sVERBOSE
		    printf("Checking dependence between groups: %d x %d\n", gA, gB);
		    group_optimal_Solutions[gA].to_Screen();
		    group_optimal_Solutions[gB].to_Screen();
#endif
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			set_Adjacent(gA, gB, 1, adjacency_Matrix);
		    }
		}
	    }
	    RobotGroups_vector merged_robot_Groups;
	    merged = merge_RobotGroups_(robot_Groups,
					group_optimal_Solutions,
					adjacency_Matrix,
					merged_robot_Groups,
					merged_group_optimal_Solutions);

#ifdef sVERBOSE
	    print_RobotGroups(merged_robot_Groups);
#endif

	    robot_Groups = merged_robot_Groups;
	    group_optimal_Solutions = merged_group_optimal_Solutions;
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);
	optimal_solution.to_Screen();
	print_RobotGroups(robot_Groups);

	optimal_makespan = optimal_solution.get_StepCount();

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_MakespanOptimalSolutionAD(const sRobotArrangement &start_arrangement,
									     const sRobotGoal        &final_arrangement,
									     const sUndirectedGraph  &environment,
									     const sUndirectedGraph  &sparse_environment,
									     int                      max_makespan,
									     int                     &optimal_makespan,
									     sMultirobotSolution     &optimal_solution,
									     int                      thread_id)
    {
	sResult result;

	optimal_makespan = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged = false;
	MultirobotSolutions_vector group_optimal_Solutions;
	SolutionMakespans_vector group_optimal_Makespans;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());
	group_optimal_Makespans.resize(robot_Groups.size(), -1);	

	do
	{

	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		{
		    printf("Initiating instance for group %d\n", g);
		}
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		{
		    printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
		}
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_makespan;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = compute_OptimalSolution_(group_start_arrangement,
							  group_final_arrangement,
							  environment,
							  sparse_environment,
							  max_makespan,
							  group_optimal_makespan,
							  group_optimal_solution,
							  thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_makespan < 0) // makespan undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			int group_optimal_cost;
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
		    group_optimal_Makespans[g] = group_optimal_solution.get_StepCount();
#ifdef sVERBOSE
		    {
			printf("Computed group solution makespan: %d\n", group_optimal_makespan);
		    }
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

	    SortedGroups_mmap sorted_Groups;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
		sorted_Groups.insert(SortedGroups_mmap::value_type(group_optimal_Solutions[g].get_StepCount(), g));
	    }
	    
	    RobotGroups_vector sort_robot_Groups;
	    SolutionMakespans_vector sort_group_optimal_Makespans;
	    MultirobotSolutions_vector sort_group_optimal_Solutions;
	    RobotArrangements_vector sort_group_start_Arrangements;
	    RobotGoals_vector sort_group_final_Arrangements;
	    
	    for (SortedGroups_mmap::const_iterator group = sorted_Groups.begin(); group != sorted_Groups.end(); ++group)
	    {
		sort_robot_Groups.push_back(robot_Groups[group->second]);
		sort_group_optimal_Makespans.push_back(group_optimal_Makespans[group->second]);
		sort_group_optimal_Solutions.push_back(group_optimal_Solutions[group->second]);
		sort_group_start_Arrangements.push_back(group_start_Arrangements[group->second]);
		sort_group_final_Arrangements.push_back(group_final_Arrangements[group->second]);
	    }
	    robot_Groups = sort_robot_Groups;
	    group_optimal_Makespans = sort_group_optimal_Makespans;
	    group_optimal_Solutions = sort_group_optimal_Solutions;
	    group_start_Arrangements = sort_group_start_Arrangements;
	    group_final_Arrangements = sort_group_final_Arrangements;
	    		     
#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    merged = false;

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{		    
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
			
			printf("Checking dependence between groups: %d x %d\n", gA, gB);
			group_optimal_Solutions[gA].to_Screen();
			group_optimal_Solutions[gB].to_Screen();
			
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			result = resolve_MakespanGroupDependency(robot_Groups,
								 gA,
								 gB,
								 start_arrangement,
								 final_arrangement,
								 environment,
								 sparse_environment,
								 max_makespan,
								 group_start_Arrangements,
								 group_final_Arrangements,
								 group_optimal_Solutions,
								 group_optimal_Makespans,						
								 thread_id);

			if (sFAILED(result))
			{
			    return result;
			}
		       
			switch (result)
			{			    			   
			case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
			{
			    gB = robot_Groups.size();
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
			case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
			{
			    gB = robot_Groups.size();			    
			    merged = true;
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
			{
			    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		}
	    }
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);	
	optimal_solution.to_Screen();	
	print_RobotGroups(robot_Groups);

	optimal_makespan = optimal_solution.get_StepCount();

	return sRESULT_SUCCESS;	
    }

    
    sResult sMultirobotSolutionCompressor::incompute_MakespanOptimalSolutionAD(Glucose::Solver         **solver,
									       const sRobotArrangement &start_arrangement,
									       const sRobotGoal        &final_arrangement,
									       const sUndirectedGraph  &environment,
									       const sUndirectedGraph  &sparse_environment,
									       int                      max_makespan,
									       int                     &optimal_makespan,
									       sMultirobotSolution     &optimal_solution,
									       int                      thread_id)
    {
	sResult result;

	optimal_makespan = MAKESPAN_UNDEFINED;

	RobotGroups_vector robot_Groups;
	int N_Robots = start_arrangement.get_RobotCount();
	initialize_RobotGroups(N_Robots, robot_Groups);
	sASSERT(check_RobotGroups(N_Robots, robot_Groups));

	sUndirectedGraph environment_work(environment);

	bool merged = false;
	MultirobotSolutions_vector group_optimal_Solutions;
	SolutionMakespans_vector group_optimal_Makespans;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	group_optimal_Solutions.resize(robot_Groups.size(), sMultirobotSolution());
	group_optimal_Makespans.resize(robot_Groups.size(), -1);	

	do
	{

	    MultirobotSolutions_vector merged_group_optimal_Solutions;
	    RobotArrangements_vector group_start_Arrangements;
	    RobotGoals_vector group_final_Arrangements;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
#ifdef sVERBOSE
		{
		    printf("Initiating instance for group %d\n", g);
		}
#endif

		sRobotArrangement group_start_arrangement;
		sRobotGoal group_final_arrangement;
	 
		construct_InstanceFromGroup(g,
					    robot_Groups,
					    environment,
					    start_arrangement,
					    final_arrangement,
					    group_start_arrangement,
					    group_final_arrangement);
		
		group_start_Arrangements.push_back(group_start_arrangement);
		group_final_Arrangements.push_back(group_final_arrangement);

#ifdef sVERBOSE
		{
		    printf("Computing solution for group %d (size:%ld) ...\n", g, robot_Groups[g].size());
		}
#endif
		if (group_optimal_Solutions[g].is_Null())
		{
		    int group_optimal_makespan;
		    sMultirobotSolution group_optimal_solution;

		    if (group_start_arrangement.get_RobotCount() > 1)
		    {    
			result = incompute_OptimalSolution_(solver,
							    group_start_arrangement,
							    group_final_arrangement,
							    environment,
							    sparse_environment,
							    max_makespan,
							    group_optimal_makespan,
							    group_optimal_solution,
							    thread_id);
		    
			if (sFAILED(result))
			{
			    return result;
			}
			if (group_optimal_makespan < 0) // makespan undefined
			{
			    return sRESULT_SUCCESS;
			}
		    }
		    else
		    {
			int group_optimal_cost;
			compute_SingleCostOptimalSolution(group_start_arrangement,
							  group_final_arrangement,
							  environment_work,
							  group_optimal_cost,
							  group_optimal_solution);
		    }
		    group_optimal_Solutions[g] = group_optimal_solution;
		    group_optimal_Makespans[g] = group_optimal_solution.get_StepCount();
#ifdef sVERBOSE
		    {
			printf("Computed group solution makespan: %d\n", group_optimal_makespan);
		    }
		    group_optimal_solution.to_Screen();
#endif
		    finish_seconds = sGet_CPU_Seconds();

		    if (finish_seconds - start_seconds  > m_total_timeout)
		    {
			return sRESULT_SUCCESS;
		    }
		}
	    }

	    SortedGroups_mmap sorted_Groups;
	    
	    for (int g = 0; g < robot_Groups.size(); ++g)
	    {
		sorted_Groups.insert(SortedGroups_mmap::value_type(group_optimal_Solutions[g].get_StepCount(), g));
	    }
	    
	    RobotGroups_vector sort_robot_Groups;
	    SolutionMakespans_vector sort_group_optimal_Makespans;
	    MultirobotSolutions_vector sort_group_optimal_Solutions;
	    RobotArrangements_vector sort_group_start_Arrangements;
	    RobotGoals_vector sort_group_final_Arrangements;
	    
	    for (SortedGroups_mmap::const_iterator group = sorted_Groups.begin(); group != sorted_Groups.end(); ++group)
	    {
		sort_robot_Groups.push_back(robot_Groups[group->second]);
		sort_group_optimal_Makespans.push_back(group_optimal_Makespans[group->second]);
		sort_group_optimal_Solutions.push_back(group_optimal_Solutions[group->second]);
		sort_group_start_Arrangements.push_back(group_start_Arrangements[group->second]);
		sort_group_final_Arrangements.push_back(group_final_Arrangements[group->second]);
	    }
	    robot_Groups = sort_robot_Groups;
	    group_optimal_Makespans = sort_group_optimal_Makespans;
	    group_optimal_Solutions = sort_group_optimal_Solutions;
	    group_start_Arrangements = sort_group_start_Arrangements;
	    group_final_Arrangements = sort_group_final_Arrangements;
	    		     
#ifdef sVERBOSE
	    print_RobotGroups(robot_Groups);
#endif	    
	    merged = false;

	    for (int gA = 0; gA < robot_Groups.size() - 1; ++gA)
	    {
		for (int gB = gA + 1; gB < robot_Groups.size(); ++gB)
		{		    
		    bool dependent = check_GroupDependence(group_start_Arrangements[gA],
							   group_start_Arrangements[gB],
							   group_optimal_Solutions[gA],
							   group_optimal_Solutions[gB]);

#ifdef sVERBOSE
		    if (dependent)
		    {
			printf("----> Groups DEPENDENT\n");
			
			printf("Checking dependence between groups: %d x %d\n", gA, gB);
			group_optimal_Solutions[gA].to_Screen();
			group_optimal_Solutions[gB].to_Screen();
			
		    }
		    else
		    {
			printf("----> Groups INDEPENDENT\n");
		    }
#endif
		    if (dependent)
		    {
			result = inresolve_MakespanGroupDependency(solver,
								   robot_Groups,
								   gA,
								   gB,
								   start_arrangement,
								   final_arrangement,
								   environment,
								   sparse_environment,
								   max_makespan,
								   group_start_Arrangements,
								   group_final_Arrangements,
								   group_optimal_Solutions,
								   group_optimal_Makespans,						
								   thread_id);

			if (sFAILED(result))
			{
			    return result;
			}
		       
			switch (result)
			{			    			   
			case sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO:
			{
			    gB = robot_Groups.size();
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO:
			case sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO:
			{
			    gB = robot_Groups.size();			    
			    merged = true;
			    break;
			}
			case sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO:
			{
			    return sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO;
			}
			default:
			{
			    sASSERT(false);
			    break;
			}
			}
		    }
		}
	    }
	}
	while (merged);

	merge_GroupRobotSolutions(robot_Groups, group_optimal_Solutions, optimal_solution);	
	optimal_solution.to_Screen();	
	print_RobotGroups(robot_Groups);

	optimal_makespan = optimal_solution.get_StepCount();

	return sRESULT_SUCCESS;	
    }        
    

    sResult sMultirobotSolutionCompressor::compute_CostOptimalSolution_binary(const sRobotArrangement                        &start_arrangement,
									      const sRobotGoal                               &final_arrangement,
									      sUndirectedGraph                               &environment,
									      const sUndirectedGraph                         &sparse_environment,
									      const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
									      int                                             max_total_cost,
									      int                                            &optimal_cost,
									      sMultirobotSolution                            &optimal_solution,
									      int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = compute_OptimalCost_binary(start_arrangement, final_arrangement, environment, sparse_environment, max_total_cost, optimal_cost, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

		int extra_cost;
		sMultirobotInstance::MDD_vector MDD, extra_MDD;
		
		instance.construct_MDD(final_encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);

		result = extract_ComputedRXMddSolution(start_arrangement,
						       environment,
						       MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution,
						       thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_CostOptimalSolution_binary(Glucose::Solver                                **solver,
										const sRobotArrangement                        &start_arrangement,
										const sRobotGoal                               &final_arrangement,
										sUndirectedGraph                               &environment,
										const sUndirectedGraph                         &sparse_environment,
										const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
										int                                             max_total_cost,
										int                                            &optimal_cost,
										sMultirobotSolution                            &optimal_solution,
										int                                             thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = incompute_OptimalCost_binary(solver, start_arrangement, final_arrangement, environment, sparse_environment, max_total_cost, optimal_cost, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}
	if (optimal_cost != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_RXMDD:
	    case ENCODING_RXMDD_BINARY:
	    {
		sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

		int extra_cost;
		sMultirobotInstance::MDD_vector MDD, extra_MDD;
		
		instance.construct_MDD(final_encoding_context.m_max_total_cost, MDD, extra_cost, extra_MDD);

		result = intract_ComputedRXMddSolution(*solver,
						       start_arrangement,
						       environment,
						       MDD,
						       optimal_cost,
						       final_encoding_context,
						       optimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_BestCostSolution(const sRobotArrangement                        &start_arrangement,
								    const sRobotGoal                               &final_arrangement,
								    sUndirectedGraph                               &environment,
								    const sUndirectedGraph                         &sparse_environment,
								    const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
								    int                                             max_total_cost,
								    int                                            &optimal_cost,
								    sMultirobotSolution                            &optimal_solution,
								    int                                             thread_id)
    {
	sResult result;
	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = compute_BestCost(start_arrangement, final_arrangement, environment, sparse_environment, max_total_cost, optimal_cost, optimal_solution, final_encoding_context, thread_id);

	if (sFAILED(result))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_BestCostSolution(Glucose::Solver                               **solver,
								      const sRobotArrangement                        &start_arrangement,
								      const sRobotGoal                               &final_arrangement,
								      sUndirectedGraph                               &environment,
								      const sUndirectedGraph                         &sparse_environment,
								      const sMultirobotInstance::MDD_vector          &sUNUSED(MDD),
								      int                                             max_total_cost,
								      int                                            &optimal_cost,
								      sMultirobotSolution                            &optimal_solution,
								      int                                             thread_id)
    {
	sResult result;
	sMultirobotEncodingContext_CNFsat final_encoding_context;

	result = incompute_BestCost(solver, start_arrangement, final_arrangement, environment, sparse_environment, max_total_cost, optimal_cost, optimal_solution, final_encoding_context, thread_id);

	if (sFAILED(result))
	{
	    return result;
	}
	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/

    sResult sMultirobotSolutionCompressor::compute_UnirobotMakespan(const sRobotArrangement           &start_arrangement,
								    const sRobotGoal                  &final_arrangement,
								    const sUndirectedGraph            &environment,
								    const sUndirectedGraph            &sparse_environment,
								    int                                layer_upper_bound,
								    int                               &unirobot_makespan,
								    sMultirobotEncodingContext_CNFsat &final_encoding_context,
								    int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	sString cnf_filename, cnf_out_filename, output_filename;
	unirobot_makespan = MAKESPAN_UNDEFINED;

	int N_Layers = 1;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

#ifdef sVERBOSE
	    printf("Solving layer %d ...\n", N_Layers);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    cnf_out_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "_out.cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}
		
		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);

		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

#ifdef PREPROCESS
	    sString preprocess_call;
	    preprocess_call = "../../pre/HyPre/hypre -v 0 -o " + cnf_out_filename + " " + cnf_filename +  " 1>/dev/null";
	    
	    int preprocess_result = system(preprocess_call.c_str());
	    
	    if (preprocess_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    FILE *fro;
	    if ((fro = fopen(cnf_out_filename.c_str(), "r")) == NULL)
	    {
		cnf_out_filename = cnf_filename;
	    }
	    else
	    {
		fclose(fro);
	    }
	    
#else
	    cnf_out_filename = cnf_filename;
#endif

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_out_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());
	    
	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
	    
#ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
#endif
	
	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }
	    
	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);
	    
#ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	    }
#endif
	    
	    if (strcmp(answer, "UNSAT") == 0)
	    {
#ifdef sSTATISTICS
		{
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		unirobot_makespan = MAKESPAN_UNDEFINED;
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		unirobot_makespan = MAKESPAN_UNDEFINED;
		
#ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
#endif
		return sRESULT_SUCCESS;
	    }
	    else /* if (strcmp(answer, "SAT") == 0) */
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		unirobot_makespan = N_Layers;
		break;
	    }
	    if (m_encoding == ENCODING_SINGULAR)
	    {
		break;		
	    }
	    else
	    {
		if (m_encoding == ENCODING_PLURAL || m_encoding == ENCODING_PLURAL2)
		{
		    if (++N_Layers > layer_upper_bound)
		    {
			break;
		    }
		}
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_UnirobotMakespan(Glucose::Solver                   **solver,
								      const sRobotArrangement           &start_arrangement,
								      const sRobotGoal                  &final_arrangement,
								      const sUndirectedGraph            &environment,
								      const sUndirectedGraph            &sparse_environment,
								      int                                layer_upper_bound,
								      int                               &unirobot_makespan,
								      sMultirobotEncodingContext_CNFsat &final_encoding_context,
								      int                                sUNUSED(thread_id))
    {
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	unirobot_makespan = MAKESPAN_UNDEFINED;

	int N_Layers = 1;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

#ifdef sVERBOSE
	    printf("Solving layer %d ...\n", N_Layers);
#endif
	    
	    switch (m_encoding)
	    {
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		unirobot_makespan = N_Layers;		
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		unirobot_makespan = MAKESPAN_UNDEFINED;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		unirobot_makespan = MAKESPAN_UNDEFINED;		
	    }
	    if (m_encoding == ENCODING_SINGULAR)
	    {
		break;		
	    }
	    else
	    {
		if (m_encoding == ENCODING_PLURAL || m_encoding == ENCODING_PLURAL2)
		{
		    if (++N_Layers > layer_upper_bound)
		    {
			break;
		    }
		}
	    }

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_UnirobotSolution(int                      unirobot_id,
								    const sRobotArrangement &start_arrangement,
								    const sRobotGoal        &final_arrangement,
								    const sUndirectedGraph  &environment,
								    const sUndirectedGraph  &sparse_environment,
								    int                      layer_upper_bound,
								    int                     &unirobot_makespan,
								    sMultirobotSolution     &unirobot_solution,
								    int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

//	int N_Robots = start_arrangement.get_RobotCount();

	sRobotArrangement unirobot_final_arrangement(start_arrangement);
	sRobotGoal::Vertices_set unirobot_Vertices = final_arrangement.get_RobotGoal(unirobot_id);
	sASSERT(unirobot_Vertices.size() == 1);

	int occup_robot_id = unirobot_final_arrangement.get_VertexOccupancy(*unirobot_Vertices.begin());

	if (occup_robot_id == 0)
	{
	    unirobot_final_arrangement.remove_Robot(unirobot_id);
	    unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());

/* all towards goals one step
	    int N_Robots = start_arrangement.get_RobotCount();
	    for (int robot_id = unirobot_id + 1; robot_id <= N_Robots; ++robot_id)
	    {
		{
		    int vertex_id = unirobot_final_arrangement.get_RobotLocation(robot_id);
		    const sVertex::Neighbors_list &Neighbors =  environment.get_Vertex(vertex_id)->m_Neighbors;
		    sASSERT(!Neighbors.empty());

		    sVertex::Neighbors_list::const_iterator nearest_neighbor = Neighbors.end();
		    int nearest_neighbor_dist = sINT_32_MAX;

		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			sRobotGoal::Vertices_set robot_Vertices = final_arrangement.get_RobotGoal(unirobot_id);
			sASSERT(robot_Vertices.size() == 1);
			
			if (all_pairs_Distances[neighbor_id][*robot_Vertices.begin()] < nearest_neighbor_dist && unirobot_final_arrangement.get_VertexOccupancy(neighbor_id) == 0)
			{
			    nearest_neighbor_dist = all_pairs_Distances[neighbor_id][*robot_Vertices.begin()];
			    nearest_neighbor = neighbor;
			}
		    }
		    if (nearest_neighbor_dist < sINT_32_MAX)
		    {
			unirobot_final_arrangement.remove_Robot(robot_id);
			unirobot_final_arrangement.place_Robot(robot_id, (*nearest_neighbor)->m_target->m_id);
		    }
		}
	    }
*/
	}
	else if (occup_robot_id == unirobot_id)
	{
	    return sRESULT_SUCCESS;
	}
	else
	{
	    printf("----------------------------------->\n");
	    int original_unirobot_vertex_id = unirobot_final_arrangement.get_RobotLocation(unirobot_id);

	    sRobotGoal::Vertices_set occup_robot_Vertices = final_arrangement.get_RobotGoal(occup_robot_id);
	    sASSERT(occup_robot_Vertices.size() == 1);
	    
	    int occup_robot_id_2 = unirobot_final_arrangement.get_VertexOccupancy(*occup_robot_Vertices.begin());

	    if (occup_robot_id_2 == 0)
	    {
		printf("============================================>\n");
		unirobot_final_arrangement.remove_Robot(unirobot_id);
		unirobot_final_arrangement.remove_Robot(occup_robot_id);

		unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());
		unirobot_final_arrangement.place_Robot(occup_robot_id, *occup_robot_Vertices.begin());
	    }
	    else
	    {
		unirobot_final_arrangement.remove_Robot(unirobot_id);
		unirobot_final_arrangement.remove_Robot(occup_robot_id);	    

		unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());
		unirobot_final_arrangement.place_Robot(occup_robot_id, original_unirobot_vertex_id);
	    }
	}
	sRobotGoal unirobot_final_goal(unirobot_final_arrangement);
/*
	int N_Robots = start_arrangement.get_RobotCount();
	for (int robot_id = unirobot_id + 1; robot_id <= N_Robots; ++robot_id)
	{
	    unirobot_final_goal.assign_Goal(*unirobot_final_goal.get_RobotGoal(robot_id).begin(), unirobot_id + 1);
	}
*/
	result = compute_UnirobotMakespan(start_arrangement, unirobot_final_goal, environment, sparse_environment, layer_upper_bound, unirobot_makespan, final_encoding_context, thread_id);

	if (sFAILED(result))
	{
	    return result;
	}
	if (unirobot_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, unirobot_makespan, final_encoding_context, unirobot_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, unirobot_makespan, final_encoding_context, unirobot_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_UnirobotSolution(Glucose::Solver        **solver,
								      int                      unirobot_id,
								      const sRobotArrangement &start_arrangement,
								      const sRobotGoal        &final_arrangement,
								      const sUndirectedGraph  &environment,
								      const sUndirectedGraph  &sparse_environment,
								      int                      layer_upper_bound,
								      int                     &unirobot_makespan,
								      sMultirobotSolution     &unirobot_solution,
								      int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;

//	int N_Robots = start_arrangement.get_RobotCount();

	sRobotArrangement unirobot_final_arrangement(start_arrangement);
	sRobotGoal::Vertices_set unirobot_Vertices = final_arrangement.get_RobotGoal(unirobot_id);
	sASSERT(unirobot_Vertices.size() == 1);

	int occup_robot_id = unirobot_final_arrangement.get_VertexOccupancy(*unirobot_Vertices.begin());

	if (occup_robot_id == 0)
	{
	    unirobot_final_arrangement.remove_Robot(unirobot_id);
	    unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());
	}
	else if (occup_robot_id == unirobot_id)
	{
	    return sRESULT_SUCCESS;
	}
	else
	{
	    printf("----------------------------------->\n");
	    int original_unirobot_vertex_id = unirobot_final_arrangement.get_RobotLocation(unirobot_id);

	    sRobotGoal::Vertices_set occup_robot_Vertices = final_arrangement.get_RobotGoal(occup_robot_id);
	    sASSERT(occup_robot_Vertices.size() == 1);
	    
	    int occup_robot_id_2 = unirobot_final_arrangement.get_VertexOccupancy(*occup_robot_Vertices.begin());

	    if (occup_robot_id_2 == 0)
	    {
		printf("============================================>\n");
		unirobot_final_arrangement.remove_Robot(unirobot_id);
		unirobot_final_arrangement.remove_Robot(occup_robot_id);

		unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());
		unirobot_final_arrangement.place_Robot(occup_robot_id, *occup_robot_Vertices.begin());
	    }
	    else
	    {
		unirobot_final_arrangement.remove_Robot(unirobot_id);
		unirobot_final_arrangement.remove_Robot(occup_robot_id);	    

		unirobot_final_arrangement.place_Robot(unirobot_id, *unirobot_Vertices.begin());
		unirobot_final_arrangement.place_Robot(occup_robot_id, original_unirobot_vertex_id);
	    }
	}
	sRobotGoal unirobot_final_goal(unirobot_final_arrangement);

	result = incompute_UnirobotMakespan(solver, start_arrangement, unirobot_final_goal, environment, sparse_environment, layer_upper_bound, unirobot_makespan, final_encoding_context, thread_id);

	if (sFAILED(result))
	{
	    return result;
	}
	if (unirobot_makespan != MAKESPAN_UNDEFINED)
	{	
	    switch (m_encoding)
	    {
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, unirobot_makespan, final_encoding_context, unirobot_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, unirobot_makespan, final_encoding_context, unirobot_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


    typedef std::multimap<int, int, std::less<int> > Robots_map;

    sResult sMultirobotSolutionCompressor::compute_UnirobotsSolution(const sRobotArrangement &start_arrangement,
								     const sRobotGoal        &final_arrangement,
								     const sUndirectedGraph  &environment,
								     const sUndirectedGraph  &sparse_environment,
								     int                      layer_upper_bound,
								     int                     &unirobots_makespan,
								     sMultirobotSolution     &unirobots_solution,
								     int                      thread_id)
    {
	sResult result;


	sRobotArrangement unirobot_start_arrangement(start_arrangement);
	int N_Robots = start_arrangement.get_RobotCount();

	unirobots_makespan = 0;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	sMultirobotInstance::VertexIDs_vector source_IDs, goal_IDs;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    source_IDs.push_back(start_arrangement.get_RobotLocation(robot_id));
	    sASSERT(final_arrangement.get_RobotGoal(robot_id).size() == 1);
	    
	    goal_IDs.push_back(*final_arrangement.get_RobotGoal(robot_id).begin());
	}	
	sUndirectedGraph dist_environment = environment;
	dist_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = dist_environment.get_SourceShortestPaths();

	Robots_map Robots;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sRobotGoal::Vertices_set robot_Vertices = final_arrangement.get_RobotGoal(robot_id);
	    sASSERT(robot_Vertices.size() == 1);

	    printf("%d\n", source_Distances[start_arrangement.get_RobotLocation(robot_id)][*robot_Vertices.begin()]);
//	    Robots.insert(Robots_map::value_type(*robot_Vertices.begin(), robot_id));
	    Robots.insert(Robots_map::value_type(source_Distances[start_arrangement.get_RobotLocation(robot_id)][*robot_Vertices.begin()], robot_id));
	}

	int progress = 1;
	for (Robots_map::const_iterator unirobot = Robots.begin(); unirobot != Robots.end(); ++unirobot)
//	for (int unirobot_id = 1; unirobot_id <= N_Robots; ++unirobot_id)
	{
	    int unirobot_id = unirobot->second;

	    int unirobot_makespan = 0;
	    sMultirobotSolution unirobot_solution;
	    sMultirobotSolution unirobot_pre_solution;

	    printf("Processing robot: %d (%.3f%%)\n", unirobot_id, 100.0 * ((double)progress / N_Robots));
	    ++progress;

	    result = compute_UnirobotSolution(unirobot_id,
					      unirobot_start_arrangement,
					      final_arrangement,
					      environment,
					      sparse_environment,
					      layer_upper_bound,
					      unirobot_makespan,
					      unirobot_pre_solution,
					      thread_id);

	    if (sFAILED(result))
	    {
		return result;
	    }
	    sRobotGoal final_filter_arrangement = final_arrangement;
	    int N_Vertices = environment.get_VertexCount();

	    Robots_map::const_iterator robot = unirobot;
	    for (++robot; robot != Robots.end(); ++robot)
//	    for (int robot_id = unirobot_id + 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_id = robot->second;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    final_filter_arrangement.charge_Robot(robot_id, vertex_id);
		}
	    }
	    unirobot_pre_solution.filter_Solution(unirobot_start_arrangement,
						  final_filter_arrangement,
						  unirobot_solution);
//	    unirobot_solution = unirobot_pre_solution;
	    
	    sRobotArrangement next_unirobot_start_arrangement;
	    unirobot_solution.execute_Solution(unirobot_start_arrangement, next_unirobot_start_arrangement);

	    unirobot_start_arrangement = next_unirobot_start_arrangement;

	    sMultirobotSolution next_unirobots_subsolution(unirobots_solution.get_StepCount(), unirobot_solution);
	    sMultirobotSolution next_unirobots_solution(unirobots_solution, next_unirobots_subsolution);
	    unirobots_solution = next_unirobots_solution;

	    unirobots_makespan += unirobot_makespan;

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	sMultirobotSolution critical_unirobots_solution;
	unirobots_solution.criticalize_Solution(critical_unirobots_solution);

	unirobots_solution = critical_unirobots_solution;


	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_UnirobotsSolution(Glucose::Solver         **solver,
								       const sRobotArrangement  &start_arrangement,
								       const sRobotGoal         &final_arrangement,
								       const sUndirectedGraph   &environment,
								       const sUndirectedGraph   &sparse_environment,
								       int                       layer_upper_bound,
								       int                      &unirobots_makespan,
								       sMultirobotSolution      &unirobots_solution,
								       int                       thread_id)
    {
	sResult result;


	sRobotArrangement unirobot_start_arrangement(start_arrangement);
	int N_Robots = start_arrangement.get_RobotCount();

	unirobots_makespan = 0;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	sMultirobotInstance::VertexIDs_vector source_IDs, goal_IDs;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    source_IDs.push_back(start_arrangement.get_RobotLocation(robot_id));
	    sASSERT(final_arrangement.get_RobotGoal(robot_id).size() == 1);
	    
	    goal_IDs.push_back(*final_arrangement.get_RobotGoal(robot_id).begin());
	}	
	sUndirectedGraph dist_environment = environment;
	dist_environment.calc_SourceGoalShortestPaths(source_IDs, goal_IDs);

	const sUndirectedGraph::Distances_2d_vector &source_Distances = dist_environment.get_SourceShortestPaths();

	Robots_map Robots;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sRobotGoal::Vertices_set robot_Vertices = final_arrangement.get_RobotGoal(robot_id);
	    sASSERT(robot_Vertices.size() == 1);

	    printf("%d\n", source_Distances[start_arrangement.get_RobotLocation(robot_id)][*robot_Vertices.begin()]);
//	    Robots.insert(Robots_map::value_type(*robot_Vertices.begin(), robot_id));
	    Robots.insert(Robots_map::value_type(source_Distances[start_arrangement.get_RobotLocation(robot_id)][*robot_Vertices.begin()], robot_id));
	}

	int progress = 1;
	for (Robots_map::const_iterator unirobot = Robots.begin(); unirobot != Robots.end(); ++unirobot)
//	for (int unirobot_id = 1; unirobot_id <= N_Robots; ++unirobot_id)
	{
	    int unirobot_id = unirobot->second;

	    int unirobot_makespan = 0;
	    sMultirobotSolution unirobot_solution;
	    sMultirobotSolution unirobot_pre_solution;

	    printf("Processing robot: %d (%.3f%%)\n", unirobot_id, 100.0 * ((double)progress / N_Robots));
	    ++progress;

	    result = incompute_UnirobotSolution(solver,
						unirobot_id,
						unirobot_start_arrangement,
						final_arrangement,
						environment,
						sparse_environment,
						layer_upper_bound,
						unirobot_makespan,
						unirobot_pre_solution,
					      thread_id);

	    if (sFAILED(result))
	    {
		return result;
	    }
	    sRobotGoal final_filter_arrangement = final_arrangement;
	    int N_Vertices = environment.get_VertexCount();

	    Robots_map::const_iterator robot = unirobot;
	    for (++robot; robot != Robots.end(); ++robot)
//	    for (int robot_id = unirobot_id + 1; robot_id <= N_Robots; ++robot_id)
	    {
		int robot_id = robot->second;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    final_filter_arrangement.charge_Robot(robot_id, vertex_id);
		}
	    }
	    unirobot_pre_solution.filter_Solution(unirobot_start_arrangement,
						  final_filter_arrangement,
						  unirobot_solution);
//	    unirobot_solution = unirobot_pre_solution;
	    
	    sRobotArrangement next_unirobot_start_arrangement;
	    unirobot_solution.execute_Solution(unirobot_start_arrangement, next_unirobot_start_arrangement);

	    unirobot_start_arrangement = next_unirobot_start_arrangement;

	    sMultirobotSolution next_unirobots_subsolution(unirobots_solution.get_StepCount(), unirobot_solution);
	    sMultirobotSolution next_unirobots_solution(unirobots_solution, next_unirobots_subsolution);
	    unirobots_solution = next_unirobots_solution;

	    unirobots_makespan += unirobot_makespan;

	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}
	sMultirobotSolution critical_unirobots_solution;
	unirobots_solution.criticalize_Solution(critical_unirobots_solution);

	unirobots_solution = critical_unirobots_solution;


	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/

    sResult sMultirobotSolutionCompressor::compute_SuboptimalMakespan(const sRobotArrangement &start_arrangement,
								      const sRobotArrangement &final_arrangement,
								      const sUndirectedGraph  &environment,
								      const sUndirectedGraph  &sparse_environment,
								      int                      makespan_lower_bound,
								      int                      makespan_upper_bound,
								      int                     &suboptimal_makespan,
								      int                      thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return compute_SuboptimalMakespan(start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, suboptimal_makespan, dummy_final_encoding_context, thread_id);
    }


    sResult sMultirobotSolutionCompressor::incompute_SuboptimalMakespan(Glucose::Solver         **solver,
									const sRobotArrangement  &start_arrangement,
									const sRobotArrangement  &final_arrangement,
									const sUndirectedGraph   &environment,
									const sUndirectedGraph   &sparse_environment,
									int                       makespan_lower_bound,
									int                       makespan_upper_bound,
									int                      &suboptimal_makespan,
									int                       thread_id)
    {
	sMultirobotEncodingContext_CNFsat dummy_final_encoding_context;

	return incompute_SuboptimalMakespan(solver, start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, suboptimal_makespan, dummy_final_encoding_context, thread_id);
    }    


    sResult sMultirobotSolutionCompressor::compute_SuboptimalMakespan(const sRobotArrangement           &start_arrangement,
								      const sRobotArrangement           &final_arrangement,
								      const sUndirectedGraph            &environment,
								      const sUndirectedGraph            &sparse_environment,
								      int                                makespan_lower_bound,
								      int                                makespan_upper_bound,
								      int                               &suboptimal_makespan,
								      sMultirobotEncodingContext_CNFsat &final_encoding_context,
								      int                                thread_id)
    {
	sResult result;
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	suboptimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	int first_unsatisfiable_makespan = makespan_lower_bound - 1;
	int last_satisfiable_makespan = makespan_upper_bound + 1;

	while (last_satisfiable_makespan - first_unsatisfiable_makespan > 1)
	{
	    int makespan_try = last_satisfiable_makespan - (last_satisfiable_makespan - first_unsatisfiable_makespan) / 2;
	    int N_Layers = makespan_try + 1;

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);
	    sString cnf_filename, output_filename;

	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_INVERSE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_InverseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicAdvancedCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDifferentialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicBijectionCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_BITWISE_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_BitwiseCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_FLOW_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_FlowCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicMatchingCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_DirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_ANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_AnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_GANO_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_GAnoCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_RELAXED_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_RelaxedMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_TOKEN_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_TokenMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_TokenEmptyMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_PermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_CapacitatedPermutationMmddCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_MMDD_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_MMDD_plus_plus_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_MmddPlusPlusCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicDirectCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_SimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_HeuristicSimplicialCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_SINGULAR_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_SingularCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_PLURAL_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_PluralCNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + "#" + sInt_32_to_String(thread_id) + ".txt";
		}
		else
		{
		    cnf_filename = CNF_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".cnf";
		    output_filename = OUTPUT_PLURAL2_FILENAME_PREFIX + "_" + sInt_32_to_String(N_Layers) + "-" + sInt_32_to_String(getpid()) + ".txt";
		}

		result = instance.to_File_Plural2CNFsat(cnf_filename, encoding_context, "", false);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }

	    sString system_call;
	    if (m_minisat_timeout != MINISAT_TIMEOUT_UNDEFINED)
	    {
		system_call = m_minisat_path + " -cpu-lim=" + sInt_32_to_String(m_minisat_timeout) + " " + cnf_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    else
	    {
		system_call = m_minisat_path + " " + cnf_filename + " " + output_filename +  " 1>/dev/null";
	    }
	    int system_result = system(system_call.c_str());

	    if (system_result < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR;
	    }
            #ifdef sSTATISTICS
	    {
		++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
	    }
	    #endif

	    FILE *fr;
	    if ((fr = fopen(output_filename.c_str(), "r")) == NULL)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR;
	    }

	    char answer[32];
	    answer[0] = '\0';

	    fscanf(fr, "%s\n", answer);
	    fclose(fr);

            #ifndef sDEBUG
	    {
		if (unlink(cnf_filename.c_str()) < 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		}
	
	    }
            #endif

	    if (strcmp(answer, "UNSAT") == 0)
	    {
		printf("UNSAT\n");
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
                #endif
		first_unsatisfiable_makespan = makespan_try;

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
	    }
	    else if (strcmp(answer, "INDET") == 0)
	    {
		printf("INDET\n");
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
                #endif
		//		first_unsatisfiable_makespan = makespan_try;
		suboptimal_makespan = MAKESPAN_UNDEFINED;

                #ifndef sDEBUG
		{
		    if (unlink(output_filename.c_str()) < 0)
		    {
			return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
		    }
		}
                #endif
		return sRESULT_SUCCESS;
	    }
	    else /*if (strcmp(answer, "SAT") == 0)*/
	    {
		printf("SAT\n");
                #ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_SAT_sat_solver_Calls;
		}
                #endif
		final_encoding_context = encoding_context;
		suboptimal_makespan = makespan_try;
		return sRESULT_SUCCESS;
	    }	    
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}

	if (last_satisfiable_makespan <= makespan_upper_bound)
	{
	    suboptimal_makespan = last_satisfiable_makespan;
	}
	else
	{
	    suboptimal_makespan = MAKESPAN_UNDEFINED;
	}

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::incompute_SuboptimalMakespan(Glucose::Solver                  **solver,
									const sRobotArrangement           &start_arrangement,
									const sRobotArrangement           &final_arrangement,
									const sUndirectedGraph            &environment,
									const sUndirectedGraph            &sparse_environment,
									int                                makespan_lower_bound,
									int                                makespan_upper_bound,
									int                               &suboptimal_makespan,
									sMultirobotEncodingContext_CNFsat &final_encoding_context,
									int                                sUNUSED(thread_id))
    {
	sMultirobotInstance instance(environment, sparse_environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	suboptimal_makespan = MAKESPAN_UNDEFINED;

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	int first_unsatisfiable_makespan = makespan_lower_bound - 1;
	int last_satisfiable_makespan = makespan_upper_bound + 1;

	while (last_satisfiable_makespan - first_unsatisfiable_makespan > 1)
	{
	    if (*solver != NULL)
	    {
		delete *solver;
	    }
	    *solver = new Glucose::Solver;
	    
	    int makespan_try = last_satisfiable_makespan - (last_satisfiable_makespan - first_unsatisfiable_makespan) / 2;
	    int N_Layers = makespan_try + 1;

	    sMultirobotEncodingContext_CNFsat encoding_context(N_Layers);

	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		instance.to_Memory_InverseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		instance.to_Memory_AdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		instance.to_Memory_DifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		instance.to_Memory_BijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		instance.to_Memory_HeuristicAdvancedCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		instance.to_Memory_HeuristicDifferentialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		instance.to_Memory_HeuristicBijectionCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		instance.to_Memory_BitwiseCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_FLOW:
	    {
		instance.to_Memory_FlowCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		instance.to_Memory_MatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		instance.to_Memory_HeuristicMatchingCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		instance.to_Memory_DirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD:
	    {
		instance.to_Memory_MmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_ANO:
	    {
		instance.to_Memory_AnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_GANO:
	    {
		instance.to_Memory_GAnoCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		instance.to_Memory_RelaxedMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		instance.to_Memory_TokenMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		instance.to_Memory_TokenEmptyMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		instance.to_Memory_PermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		instance.to_Memory_CapacitatedPermutationMmddCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		instance.to_Memory_MmddPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		instance.to_Memory_MmddPlusPlusCNFsat(*solver, encoding_context, "", false);
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		instance.to_Memory_HeuristicDirectCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		instance.to_Memory_SimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		instance.to_Memory_HeuristicSimplicialCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		instance.to_Memory_SingularCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		instance.to_Memory_PluralCNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		instance.to_Memory_Plural2CNFsat(*solver, encoding_context, "", false);
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }


	    if (!(*solver)->simplify())
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif		
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO;
	    }

	    Glucose::vec<Glucose::Lit> dummy;
	    Glucose::lbool ret = (*solver)->solveLimited(dummy);
	    
	    if (ret == l_True)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_total_sat_solver_Calls;
		}
#endif
		final_encoding_context = encoding_context;
		suboptimal_makespan = makespan_try;
		
		return sRESULT_SUCCESS;
	    }
	    else if (ret == l_False)
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_UNSAT_sat_solver_Calls;
		}
#endif
		first_unsatisfiable_makespan = makespan_try;
	    }
	    else
	    {
#ifdef sSTATISTICS
		{
		    ++s_GlobalPhaseStatistics.get_CurrentPhase().m_INDET_sat_solver_Calls;
		}
#endif
		suboptimal_makespan = MAKESPAN_UNDEFINED;
	    }
	    
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	}

	if (last_satisfiable_makespan <= makespan_upper_bound)
	{
	    suboptimal_makespan = last_satisfiable_makespan;
	}
	else
	{
	    suboptimal_makespan = MAKESPAN_UNDEFINED;
	}

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotSolutionCompressor::compute_SuboptimalSolution(const sRobotArrangement &start_arrangement,
								      const sRobotArrangement &final_arrangement,
								      const sUndirectedGraph  &environment,
								      const sUndirectedGraph  &sparse_environment,
								      int                      makespan_lower_bound,
								      int                      makespan_upper_bound,
								      int                     &suboptimal_makespan,
								      sMultirobotSolution     &suboptimal_solution,
								      int                      thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = compute_SuboptimalMakespan(start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, suboptimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (suboptimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		result = extract_ComputedInverseSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = extract_ComputedAdvancedSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		result = extract_ComputedDifferentialSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = extract_ComputedBijectionSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = extract_ComputedHeuristicAdvancedSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = extract_ComputedHeuristicDifferentialSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = extract_ComputedHeuristicBijectionSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = extract_ComputedBitwiseSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = extract_ComputedFlowSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = extract_ComputedMatchingSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = extract_ComputedHeuristicMatchingSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = extract_ComputedDirectSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedAnoSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedGAnoSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedRelaxedMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedTokenMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedTokenEmptyMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedPermutationMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedCapacitatedPermutationMmddSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedMmddPlusSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = extract_ComputedMmddPlusPlusSolution(start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = extract_ComputedHeuristicDirectSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = extract_ComputedSimplicialSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = extract_ComputedHeuristicSimplicialSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = extract_ComputedSingularSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = extract_ComputedPluralSolution(start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = extract_ComputedPlural2Solution(start_arrangement, environment, sparse_environment, suboptimal_makespan, final_encoding_context, suboptimal_solution, thread_id);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotSolutionCompressor::incompute_SuboptimalSolution(Glucose::Solver         **solver,
									const sRobotArrangement  &start_arrangement,
									const sRobotArrangement  &final_arrangement,
									const sUndirectedGraph   &environment,
									const sUndirectedGraph   &sparse_environment,
									int                       makespan_lower_bound,
									int                       makespan_upper_bound,
									int                      &suboptimal_makespan,
									sMultirobotSolution      &suboptimal_solution,
									int                       thread_id)
    {
	sResult result;

	sMultirobotEncodingContext_CNFsat final_encoding_context;
	sMultirobotInstance instance(environment, start_arrangement, final_arrangement, m_ratio, m_robustness, m_range);

	result = incompute_SuboptimalMakespan(solver, start_arrangement, final_arrangement, environment, sparse_environment, makespan_lower_bound, makespan_upper_bound, suboptimal_makespan, final_encoding_context, thread_id);
	if (sFAILED(result))
	{
	    return result;
	}

	if (suboptimal_makespan != MAKESPAN_UNDEFINED)
	{
	    switch (m_encoding)
	    {
	    case ENCODING_INVERSE:
	    {
		result = intract_ComputedInverseSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ADVANCED:
	    {
		result = intract_ComputedAdvancedSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIFFERENTIAL:
	    {
		result = intract_ComputedDifferentialSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BIJECTION:
	    {
		result = intract_ComputedBijectionSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_ADVANCED:
	    {
		result = intract_ComputedHeuristicAdvancedSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_DIFFERENTIAL:
	    {
		result = intract_ComputedHeuristicDifferentialSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_BIJECTION:
	    {
		result = intract_ComputedHeuristicBijectionSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_BITWISE:
	    {
		result = intract_ComputedBitwiseSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_FLOW:
	    {
		result = intract_ComputedFlowSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MATCHING:
	    {
		result = intract_ComputedMatchingSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_MATCHING:
	    {
		result = intract_ComputedHeuristicMatchingSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_DIRECT:
	    {
		result = intract_ComputedDirectSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_ANO:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedAnoSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_GANO:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedGAnoSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_RELAXED_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedRelaxedMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedTokenMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_TOKEN_EMPTY_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedTokenEmptyMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    
	    case ENCODING_PERMUTATION_MMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedPermutationMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PERMUTATION_CMMDD:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedCapacitatedPermutationMmddSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    	    	    
	    case ENCODING_MMDD_plus:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedMmddPlusSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_MMDD_plus_plus:
	    {
		sMultirobotInstance::MDD_vector MDD;
		instance.construct_MakespanMDD(suboptimal_makespan, MDD);

		result = intract_ComputedMmddPlusPlusSolution(*solver, start_arrangement, environment, MDD, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }	    	    
	    case ENCODING_HEURISTIC_DIRECT:
	    {
		result = intract_ComputedHeuristicDirectSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SIMPLICIAL:
	    {
		result = intract_ComputedSimplicialSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_HEURISTIC_SIMPLICIAL:
	    {
		result = intract_ComputedHeuristicSimplicialSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_SINGULAR:
	    {
		result = intract_ComputedSingularSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL:
	    {
		result = intract_ComputedPluralSolution(*solver, start_arrangement, environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    case ENCODING_PLURAL2:
	    {
		result = intract_ComputedPlural2Solution(*solver, start_arrangement, environment, sparse_environment, suboptimal_makespan, final_encoding_context, suboptimal_solution);
		if (sFAILED(result))
		{
		    return result;
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
	    }
	    }
	}

	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/

    sResult sMultirobotSolutionCompressor::shorten_Solution(const sRobotArrangement   &initial_arrangement,
							    const sMultirobotSolution &original_solution,
							    sUndirectedGraph          &environment,
							    const sUndirectedGraph    &sparse_environment,
							    sMultirobotSolution       &shortened_solution,
							    int                        thread_id)
    {
        #ifdef sSTATISTICS
	{
	    if (thread_id == THREAD_ID_UNDEFINED)
	    {
		s_GlobalPhaseStatistics.enter_Phase("shortening");
	    }
	}
        #endif

	sResult result;
	Arrangements_vector unfolded_Arrangements;
	sMultirobotSolution sub_solution;

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	int original_makespan = unfolded_Arrangements.size() - 1;
	int start_step = 0;

	while (start_step < original_makespan)
	{
	    int final_step_lower_bound = start_step;
	    int final_step_upper_bound = original_makespan + 1;
	    sMultirobotSolution best_sub_solution = original_solution.extract_Subsolution(start_step, final_step_lower_bound);

	    while (final_step_upper_bound - final_step_lower_bound > 1)
	    {
		int final_step = final_step_lower_bound + (final_step_upper_bound - final_step_lower_bound) / 2;
		int makespan_lower_bound = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);

		if (makespan_lower_bound <= m_makespan_upper_bound)
		{
		    int makespan_upper_bound = sMIN(m_makespan_upper_bound, final_step - start_step + 1);

		    int computed_makespan;
		    sMultirobotSolution computed_solution;

		    result = compute_OptimalSolution(unfolded_Arrangements[start_step],
						     unfolded_Arrangements[final_step],
						     environment,
						     sparse_environment,
						     makespan_lower_bound,
						     makespan_upper_bound,
						     computed_makespan,
						     computed_solution,
						     thread_id);
		    
		    if (sFAILED(result))
		    {
			return result;
		    }

		    if (computed_makespan != MAKESPAN_UNDEFINED)
		    {
			best_sub_solution = sMultirobotSolution(start_step, computed_solution);
			final_step_lower_bound = final_step;
		    }
		    else
		    {
			final_step_upper_bound = final_step;
		    }
		}
		else
		{
		    final_step_upper_bound = final_step;
		}
	    }

	    sub_solution = sMultirobotSolution(sub_solution, best_sub_solution);
	    start_step = final_step_lower_bound;

            #ifdef sVERBOSE
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    printf("Thread %d shortening progress = %.2f%% (current ratio = %.2f%%)\n",
			   thread_id,
			   100 * ((double)start_step / original_makespan),
			   100 * ((double)(sub_solution.get_StepCount() - sub_solution.calc_EmptySteps())  / start_step)
			);
		}
		else
		{
		    printf("Shortening progress = %.2f%% (current ratio = %.2f%%)\n",
			   100 * ((double)start_step / original_makespan),
			   100 * ((double)(sub_solution.get_StepCount() - sub_solution.calc_EmptySteps()) / start_step));
		}
	    }
            #endif
	}
	shortened_solution = sub_solution;
	shortened_solution.remove_EmptySteps();

        #ifdef sSTATISTICS
	{
	    if (thread_id == THREAD_ID_UNDEFINED)
	    {
		s_GlobalPhaseStatistics.leave_Phase();
	    }
	}
        #endif
       
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::shorten_Solution_mt(const sRobotArrangement   &initial_arrangement,
							       const sMultirobotSolution &original_solution,
							       sUndirectedGraph          &environment,
							       const sUndirectedGraph    &sUNUSED(sparse_environment),
							       sMultirobotSolution       &shortened_solution)

    {
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("shortening_MT");
	}
        #endif

	#ifdef sVERBOSE
	{
	    printf("Solution shortening(mt) commenced ...\n");
	}
	#endif

	Arrangements_vector unfolded_Arrangements;
	sMultirobotSolution sub_solution;

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	int original_makespan = unfolded_Arrangements.size() - 1;

	int N_required_Threads = sMIN(original_makespan / m_makespan_upper_bound, m_N_parallel_Threads);
	N_required_Threads = (N_required_Threads < m_N_parallel_Threads && original_makespan % m_makespan_upper_bound > 0) ? N_required_Threads + 1 : N_required_Threads;

	ProcessingArguments_vector processing_Arguments;
	processing_Arguments.resize(m_N_parallel_Threads);

	int last_finish_step = 0;
	int remaining_N_Steps = original_makespan - 1;
	
	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    processing_Arguments[thread_id].m_ctx_arg.m_compressor = this;

	    int sub_solution_N_Steps = sMIN(sMAX(m_makespan_upper_bound, remaining_N_Steps / (N_required_Threads - thread_id)), remaining_N_Steps);

	    processing_Arguments[thread_id].m_in_arg.m_original_solution = original_solution.extract_Subsolution(last_finish_step, last_finish_step + sub_solution_N_Steps);
	    processing_Arguments[thread_id].m_in_arg.m_original_solution.remove_EmptySteps();
	    processing_Arguments[thread_id].m_in_arg.m_initial_arrangement = unfolded_Arrangements[last_finish_step];
	    processing_Arguments[thread_id].m_in_arg.m_environment = &environment;
	    last_finish_step += (sub_solution_N_Steps + 1);
	    remaining_N_Steps -= (sub_solution_N_Steps + 1);

	    pthread_attr_t thread_attr;
	    if (pthread_attr_init(&thread_attr) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
	    }

	    processing_Arguments[thread_id].m_ctx_arg.m_thread_id = thread_id;

	    if (pthread_create(&processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, &thread_attr, s_shorten_Solution_mt, &processing_Arguments[thread_id]) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_CREATE_ERROR;
	    }
	    if (pthread_attr_destroy(&thread_attr) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
	    }
	}

	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    if (pthread_join(processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, NULL))
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_JOIN_ERROR;
	    }
	}

	last_finish_step = 0;

	shortened_solution = sMultirobotSolution();
	
	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    if (sFAILED(processing_Arguments[thread_id].m_out_arg.m_result))
	    {
		return processing_Arguments[thread_id].m_out_arg.m_result;
	    }
	    sMultirobotSolution sub_solution(last_finish_step, processing_Arguments[thread_id].m_out_arg.m_processed_solution);
	    last_finish_step += processing_Arguments[thread_id].m_out_arg.m_processed_solution.get_StepCount();
	    shortened_solution = sMultirobotSolution(shortened_solution, sub_solution);
	}

	#ifdef sVERBOSE
	{
	    printf("Solution shortening(mt) finished\n");
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::primeShorten_Solution(const sRobotArrangement   &initial_arrangement,
								 const sMultirobotSolution &original_solution,
								 sUndirectedGraph          &environment,
								 const sUndirectedGraph    &sparse_environment,
								 sMultirobotSolution       &shortened_solution,
								 int                        thread_id)
    {
        #ifdef sSTATISTICS
	{
	    if (thread_id == THREAD_ID_UNDEFINED)
	    {
		s_GlobalPhaseStatistics.enter_Phase("prime_shortening");
	    }
	}
        #endif

	sResult result;
	Arrangements_vector unfolded_Arrangements;
	sMultirobotSolution sub_solution;

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	int original_makespan = unfolded_Arrangements.size() - 1;
	int start_step = 0;

	while (start_step < original_makespan)
	{
	    int final_step_lower_bound = start_step;
	    int final_step_upper_bound = original_makespan + 1;

	    int best_final_step = final_step_lower_bound;
	    sMultirobotSolution best_sub_solution = original_solution.extract_Subsolution(start_step, final_step_lower_bound);

	    bool optimized = false;

	    while (final_step_upper_bound - final_step_lower_bound > 1)
	    {
		int final_step = final_step_lower_bound + (final_step_upper_bound - final_step_lower_bound) / 2;
		int makespan_lower_bound = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);
		int subsolution_original_makespan = final_step - start_step;
		int makespan_upper_bound = sMIN(m_makespan_upper_bound, subsolution_original_makespan);

		if (makespan_lower_bound < makespan_upper_bound)
		{
		    int computed_makespan;
		    sMultirobotSolution computed_solution;

		    if (m_attempt_Database.find(AttemptDatabaseRecord(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step])) != m_attempt_Database.end())
		    {
			computed_makespan = MAKESPAN_UNDEFINED;
		    }
		    else
		    {
			result = compute_OptimalSolution(unfolded_Arrangements[start_step],
							 unfolded_Arrangements[final_step],
							 environment,
							 sparse_environment,
							 makespan_lower_bound,
							 makespan_upper_bound,
							 computed_makespan,
							 computed_solution,
							 thread_id);
			if (sFAILED(result))
			{
			    return result;
			}
		    }

                    #ifdef sDEBUG
		    {
			//printf("Computed solution:%d\n", computed_makespan);
			//computed_solution.to_Screen();

			//printf("Original solution\n");
			//original_solution.extract_Subsolution(start_step, final_step - 1).to_Screen();
		    }
                    #endif
		    
		    if (computed_makespan != MAKESPAN_UNDEFINED)
		    {
			if (computed_makespan < subsolution_original_makespan)
			{
			    best_sub_solution = sMultirobotSolution(start_step, computed_solution);
			    best_final_step = final_step;
			    final_step_lower_bound = final_step;
			    optimized = true;
			}
			else
			{
			    m_attempt_Database.insert(AttemptDatabaseRecord(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step]));
			    final_step_upper_bound = final_step;
			}
		    }
		    else
		    {
			final_step_upper_bound = final_step;
		    }
		}
		else
		{
		    if (makespan_lower_bound > m_makespan_upper_bound)
		    {
			final_step_upper_bound = final_step;
		    }
		    else
		    {
			final_step_lower_bound = final_step;
		    }
		}
	    }

	    if (optimized)
	    {
		sub_solution = sMultirobotSolution(sub_solution, best_sub_solution);
		//		start_step = final_step_lower_bound;
		start_step = best_final_step;
	    }
	    else
	    {
		sub_solution = sMultirobotSolution(sub_solution, original_solution.extract_Subsolution(start_step, start_step));
		start_step += 1;
	    }

	    #ifdef sDEBUG
	    {
		//printf("Start:%d Current subsolution:\n", start_step);
		//sub_solution.to_Screen();
	    }
	    #endif

            #ifdef sVERBOSE
	    {
		if (thread_id != THREAD_ID_UNDEFINED)
		{
		    printf("Thread %d shortening progress = %.2f%% (current ratio = %.2f%%)\n",
			   thread_id,
			   100 * ((double)start_step / original_makespan),
			   100 * ((double)(sub_solution.get_StepCount() - sub_solution.calc_EmptySteps())  / start_step)
			);
		}
		else
		{
		    printf("Shortening progress = %.2f%% (current ratio = %.2f%%)\n",
			   100 * ((double)start_step / original_makespan),
			   100 * ((double)(sub_solution.get_StepCount() - sub_solution.calc_EmptySteps()) / start_step));
		}
	    }
            #endif
	}
	shortened_solution = sub_solution;
	shortened_solution.remove_EmptySteps();

        #ifdef sSTATISTICS
	{
	    if (thread_id == THREAD_ID_UNDEFINED)
	    {
		s_GlobalPhaseStatistics.leave_Phase();
	    }
	}
        #endif

        #ifdef sDEBUG
	{
	    //	    printf("Shortened solution:\n");
	    //sub_solution.to_Screen();
	}
        #endif
       
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::primeShorten_Solution_mt2(const sRobotArrangement   &initial_arrangement,
								     const sMultirobotSolution &original_solution,
								     sUndirectedGraph          &environment,
								     const sUndirectedGraph    &sUNUSED(sparse_environment),
								     sMultirobotSolution       &shortened_solution)
    {
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("prime_shortening_MT");
	}
        #endif

	#ifdef sVERBOSE
	{
	    printf("Solution shortening(mt) commenced ...\n");
	}
	#endif

	Arrangements_vector unfolded_Arrangements;
	sMultirobotSolution sub_solution;

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	//	std::vector<int> Potentials;
	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	int N_Arrangements = unfolded_Arrangements.size();
	for (int i = 0; i < N_Arrangements - m_makespan_upper_bound; ++i)
	{
//	    int start_step = i;
//	    int final_step = start_step + m_makespan_upper_bound;

//	    int dist = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);

	}

	int original_makespan = unfolded_Arrangements.size() - 1;

	int N_required_Threads = sMIN(original_makespan / m_makespan_upper_bound, m_N_parallel_Threads);
	N_required_Threads = (N_required_Threads < m_N_parallel_Threads && original_makespan % m_makespan_upper_bound > 0) ? N_required_Threads + 1 : N_required_Threads;

	ProcessingArguments_vector processing_Arguments;
	processing_Arguments.resize(m_N_parallel_Threads + 1);

	int last_finish_step = 0;
	int remaining_N_Steps = original_makespan - 1;
	
	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    processing_Arguments[thread_id].m_ctx_arg.m_compressor = this;

	    int sub_solution_N_Steps = sMIN(sMAX(m_makespan_upper_bound, remaining_N_Steps / (N_required_Threads - thread_id)), remaining_N_Steps);

	    processing_Arguments[thread_id].m_in_arg.m_original_solution = original_solution.extract_Subsolution(last_finish_step, last_finish_step + sub_solution_N_Steps);
	    processing_Arguments[thread_id].m_in_arg.m_original_solution.remove_EmptySteps();
	    processing_Arguments[thread_id].m_in_arg.m_initial_arrangement = unfolded_Arrangements[last_finish_step];
	    processing_Arguments[thread_id].m_in_arg.m_environment = &environment;
	    last_finish_step += (sub_solution_N_Steps + 1);
	    remaining_N_Steps -= (sub_solution_N_Steps + 1);

	    pthread_attr_t thread_attr;
	    if (pthread_attr_init(&thread_attr) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
	    }

	    processing_Arguments[thread_id].m_ctx_arg.m_thread_id = thread_id;

	    if (pthread_create(&processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, &thread_attr, s_prime_shorten_Solution_mt, &processing_Arguments[thread_id]) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_CREATE_ERROR;
	    }
	    if (pthread_attr_destroy(&thread_attr) != 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
	    }
	}

	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    if (pthread_join(processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, NULL))
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_JOIN_ERROR;
	    }
	}

	last_finish_step = 0;

	shortened_solution = sMultirobotSolution();
	
	for (int thread_id = 0; thread_id < N_required_Threads; ++thread_id)
	{
	    if (sFAILED(processing_Arguments[thread_id].m_out_arg.m_result))
	    {
		return processing_Arguments[thread_id].m_out_arg.m_result;
	    }
	    sMultirobotSolution sub_solution(last_finish_step, processing_Arguments[thread_id].m_out_arg.m_processed_solution);
	    last_finish_step += processing_Arguments[thread_id].m_out_arg.m_processed_solution.get_StepCount();
	    shortened_solution = sMultirobotSolution(shortened_solution, sub_solution);
	}

	#ifdef sVERBOSE
	{
	    printf("Solution prime shortening(mt) finished\n");
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::primeShorten_Solution_mt(const sRobotArrangement   &initial_arrangement,
								    const sMultirobotSolution &original_solution,
								    sUndirectedGraph          &environment,
								    const sUndirectedGraph    &sUNUSED(sparse_environment),
								    sMultirobotSolution       &shortened_solution)
    {
        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("prime_shortening_MT");
	}
        #endif

	#ifdef sVERBOSE
	{
	    printf("Solution shortening(mt) commenced ...\n");
	}
	#endif

	Arrangements_vector unfolded_Arrangements;
	sMultirobotSolution sub_solution;

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	#ifdef sDEBUG
	{
	    int N_Arrangements = unfolded_Arrangements.size();
	    for (int i = 0; i < N_Arrangements - m_makespan_upper_bound; ++i)
	    {
		int start_step = i;
		int final_step = start_step + m_makespan_upper_bound;
		
		int dist = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);

		printf("%d: step potential: %d,%d: --> %d <-- (%d,%d,%d)\n", i, final_step - start_step, dist, final_step - start_step - dist, start_step, final_step, dist);
	    }
	}
	#endif

	int original_makespan = unfolded_Arrangements.size() - 1;

	ProcessingArguments_vector processing_Arguments;
	processing_Arguments.resize(m_N_parallel_Threads);

	int last_final_step = 0;

	ProcessedSolutionRecords_set processed_solution_Records;
	int last_thread_id = 0;

	while (last_final_step < original_makespan)
	{
	    int start_step = last_final_step;
	    int final_step = start_step;

	    int distance = 0;

	    while (final_step < original_makespan)
	    {
		 ++final_step;
		 distance = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);

		 if (distance >= m_makespan_upper_bound)
		 {
		     break;
		 }
	    }

	    if (final_step - start_step > m_makespan_upper_bound)
	    {
		if (last_thread_id >= m_N_parallel_Threads)
		{
		    for (int thread_id = 0; thread_id < last_thread_id; ++thread_id)
		    {
			if (pthread_join(processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, NULL))
			{
			    return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_JOIN_ERROR;
			}
		    }
		    for (int thread_id = 0; thread_id < last_thread_id; ++thread_id)
		    {
			if (sFAILED(processing_Arguments[thread_id].m_out_arg.m_result))
			{
			    return processing_Arguments[thread_id].m_out_arg.m_result;
			}
			ProcessedSolutionRecord processed_solution_record(processing_Arguments[thread_id].m_in_arg.m_start_step,
									  processing_Arguments[thread_id].m_in_arg.m_final_step,
									  processing_Arguments[thread_id].m_out_arg.m_processed_solution);
			processed_solution_Records.insert(processed_solution_record);
		    }
		    last_thread_id = 0;
		}
		
		processing_Arguments[last_thread_id].m_ctx_arg.m_compressor = this;
		
		processing_Arguments[last_thread_id].m_in_arg.m_start_step = start_step;
		processing_Arguments[last_thread_id].m_in_arg.m_final_step = final_step;
		
		processing_Arguments[last_thread_id].m_in_arg.m_original_solution = original_solution.extract_Subsolution(start_step, final_step - 1);
		processing_Arguments[last_thread_id].m_in_arg.m_original_solution.remove_EmptySteps();
		processing_Arguments[last_thread_id].m_in_arg.m_initial_arrangement = unfolded_Arrangements[start_step];
		processing_Arguments[last_thread_id].m_in_arg.m_environment = &environment;
		
		pthread_attr_t thread_attr;
		if (pthread_attr_init(&thread_attr) != 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
		}
		processing_Arguments[last_thread_id].m_ctx_arg.m_thread_id = last_thread_id;
		
		if (pthread_create(&processing_Arguments[last_thread_id].m_ctx_arg.m_pthread_handle,
				   &thread_attr,
				   s_prime_shorten_Solution_mt,
				   &processing_Arguments[last_thread_id]) != 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_CREATE_ERROR;
		}
		
		if (pthread_attr_destroy(&thread_attr) != 0)
		{
		    return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR;
		}
		++last_thread_id;
	    }
	    else
	    {
		final_step = last_final_step + 1;
	    }
	    last_final_step = final_step;
	}
	for (int thread_id = 0; thread_id < last_thread_id; ++thread_id)
	{
	    if (pthread_join(processing_Arguments[thread_id].m_ctx_arg.m_pthread_handle, NULL))
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_JOIN_ERROR;
	    }
	}
	for (int thread_id = 0; thread_id < last_thread_id; ++thread_id)
	{
	    if (sFAILED(processing_Arguments[thread_id].m_out_arg.m_result))
	    {
		return processing_Arguments[thread_id].m_out_arg.m_result;
	    }
	    ProcessedSolutionRecord processed_solution_record(processing_Arguments[thread_id].m_in_arg.m_start_step,
							      processing_Arguments[thread_id].m_in_arg.m_final_step,
							      processing_Arguments[thread_id].m_out_arg.m_processed_solution);
	    processed_solution_Records.insert(processed_solution_record);
	}

	shortened_solution = sMultirobotSolution();
	last_final_step = 0;
	for (ProcessedSolutionRecords_set::const_iterator processed_solution =  processed_solution_Records.begin(); processed_solution !=  processed_solution_Records.end(); ++processed_solution)
	{
	    sMultirobotSolution original_part = original_solution.extract_Subsolution(last_final_step, processed_solution->m_start_step - 1);
	    sMultirobotSolution processed_part = sMultirobotSolution(processed_solution->m_start_step, processed_solution->m_processed_solution);

	    shortened_solution = sMultirobotSolution(shortened_solution, original_part);
	    shortened_solution = sMultirobotSolution(shortened_solution, processed_part);

	    last_final_step = processed_solution->m_final_step;
	}
	sMultirobotSolution final_original_part = sMultirobotSolution(last_final_step, original_solution.extract_Subsolution(last_final_step, original_makespan - 1));
	shortened_solution = sMultirobotSolution(shortened_solution, final_original_part);
	shortened_solution.remove_EmptySteps();

	#ifdef sVERBOSE
	{
	    printf("Solution prime shortening(mt) finished\n");
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::optimize_Solution(const sRobotArrangement   &initial_arrangement,
							     const sMultirobotSolution &original_solution,
							     sUndirectedGraph          &environment,
							     const sUndirectedGraph    &sparse_environment,
							     sMultirobotSolution       &optimized_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution = original_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("optimization");
	}
        #endif

	#ifdef sVERBOSE
	static int pass_cnt;
	pass_cnt = 1;
	{
	    printf("Solution optimization commenced ...\n");
	}
	#endif

	double start_seconds = sGet_WC_Seconds();
	double finish_seconds = sGet_WC_Seconds();

	while (true)
	{
            #ifdef sVERBOSE
	    {
		printf("Solution shortening pass %d ...\n", pass_cnt);
		++pass_cnt;
	    }
	    #endif

	    result = shorten_Solution(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    #ifdef sVERBOSE
	    {
		printf("Solution shortening pass %d finished with ratio %.2f%%\n", pass_cnt, 100 * ((double)next_solution.get_StepCount() / previous_solution.get_StepCount()));
		printf("Intermediate statistics:\n");
		sMultirobotSolutionAnalyzer solution_analyzer;
		solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		solution_analyzer.analyze_Solution(next_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		s_GlobalPhaseStatistics.to_Screen();
	    }
            #endif

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    optimized_solution = previous_solution;
		}
		else
		{
		    optimized_solution = next_solution;
		}
		break;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    previous_solution = next_solution;
	}

	#ifdef sVERBOSE
	{
	    printf("Solution optimization finished with ratio %.2f%%\n", 100 * ((double)optimized_solution.get_StepCount() / original_solution.get_StepCount()));
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::optimize_Solution_mt(const sRobotArrangement   &initial_arrangement,
								const sMultirobotSolution &original_solution,
								sUndirectedGraph          &environment,
								const sUndirectedGraph    &sparse_environment,
								sMultirobotSolution       &optimized_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution = original_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("optimization_MT");
	}
        #endif

	#ifdef sVERBOSE
	static int pass_cnt;
	pass_cnt = 0;
	{
	    printf("Solution optimization(mt) commenced ...\n");
	}
	#endif

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
            #ifdef sVERBOSE
	    {
		printf("Solution shortening(mt) pass %d ...\n", ++pass_cnt);
	    }
	    #endif

	    result = shorten_Solution_mt(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    #ifdef sVERBOSE
	    {
		printf("Solution shortening pass %d finished with ratio %.2f%%\n", pass_cnt, 100 * ((double)next_solution.get_StepCount() / previous_solution.get_StepCount()));
		printf("Intermediate statistics:\n");
		sMultirobotSolutionAnalyzer solution_analyzer;
		solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		solution_analyzer.analyze_Solution(next_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		s_GlobalPhaseStatistics.to_Screen();
	    }
            #endif

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    optimized_solution = previous_solution;
		}
		else
		{
		    optimized_solution = next_solution;
		}
		break;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    previous_solution = next_solution;
	}

	#ifdef sVERBOSE
	{
	    printf("Solution optimization finished with ratio %.2f%%\n", 100 * ((double)optimized_solution.get_StepCount() / original_solution.get_StepCount()));
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::primeOptimize_Solution(const sRobotArrangement   &initial_arrangement,
								  const sMultirobotSolution &original_solution,
								  sUndirectedGraph          &environment,
								  const sUndirectedGraph    &sparse_environment,
								  sMultirobotSolution       &optimized_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution = original_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("prime_optimization");
	}
        #endif

	#ifdef sVERBOSE
	static int pass_cnt;
	pass_cnt = 1;
	{
	    printf("Solution prime optimization commenced ...\n");
	}
	#endif

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
            #ifdef sVERBOSE
	    {
		printf("Solution prime shortening pass %d ...\n", pass_cnt);
		++pass_cnt;
	    }
	    #endif

	    result = primeShorten_Solution(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    #ifdef sVERBOSE
	    {
		printf("Solution prime shortening pass %d finished with ratio %.2f%%\n", pass_cnt, 100 * ((double)next_solution.get_StepCount() / previous_solution.get_StepCount()));
		printf("Intermediate statistics:\n");
		sMultirobotSolutionAnalyzer solution_analyzer;
		solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		solution_analyzer.analyze_Solution(next_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		s_GlobalPhaseStatistics.to_Screen();
	    }
            #endif

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    optimized_solution = previous_solution;
		}
		else
		{
		    optimized_solution = next_solution;
		}
		break;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }
	    previous_solution = next_solution;
	}

	#ifdef sVERBOSE
	{
	    printf("Solution prime optimization finished with ratio %.2f%%\n", 100 * ((double)optimized_solution.get_StepCount() / original_solution.get_StepCount()));
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::primeOptimize_Solution_mt(const sRobotArrangement   &initial_arrangement,
								     const sMultirobotSolution &original_solution,
								     sUndirectedGraph          &environment,
								     const sUndirectedGraph    &sparse_environment,
								     sMultirobotSolution       &optimized_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution = original_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("prime_optimization_MT");
	}
        #endif

	#ifdef sVERBOSE
	static int pass_cnt;
	pass_cnt = 0;
	{
	    printf("Solution prime optimization(mt) commenced ...\n");
	}
	#endif

	double start_seconds = sGet_CPU_Seconds();
	double finish_seconds = sGet_CPU_Seconds();

	while (true)
	{
            #ifdef sVERBOSE
	    {
		printf("Solution prime shortening(mt) pass %d ...\n", ++pass_cnt);
	    }
	    #endif

	    result = primeShorten_Solution_mt(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    #ifdef sVERBOSE
	    {
		printf("Solution prime shortening pass %d finished with ratio %.2f%%\n", pass_cnt, 100 * ((double)next_solution.get_StepCount() / previous_solution.get_StepCount()));
		printf("Intermediate statistics:\n");
		sMultirobotSolutionAnalyzer solution_analyzer;
		solution_analyzer.analyze_Solution(original_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		solution_analyzer.analyze_Solution(next_solution, initial_arrangement, environment);
		solution_analyzer.to_Screen();
		s_GlobalPhaseStatistics.to_Screen();
	    }
            #endif

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    optimized_solution = previous_solution;
		}
		else
		{
		    optimized_solution = next_solution;
		}
		break;
	    }
	    finish_seconds = sGet_CPU_Seconds();

	    if (finish_seconds - start_seconds  > m_total_timeout)
	    {
		break;
	    }

	    previous_solution = next_solution;
	}

	#ifdef sVERBOSE
	{
	    printf("Solution prime optimization finished with ratio %.2f%%\n", 100 * ((double)optimized_solution.get_StepCount() / original_solution.get_StepCount()));
	}
	#endif

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::compress_Solution(const sRobotArrangement   &initial_arrangement,
							     const sMultirobotSolution &original_solution,
							     sUndirectedGraph          &environment,
							     const sUndirectedGraph    &sparse_environment,
							     sMultirobotSolution       &compressed_solution)
    {
	sResult result;
	Arrangements_vector unfolded_Arrangements;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("compression");
	}
        #endif

	sASSERT(verify_Unfolding(initial_arrangement, original_solution, environment));
	unfold_Solution(initial_arrangement, original_solution, unfolded_Arrangements);

	sUndirectedGraph::Distances_2d_vector all_pairs_Distances;
	environment.calc_AllPairsShortestPaths(all_pairs_Distances);

	CompressionMatrix_2d_vector compression_Matrix;

	int original_makespan = unfolded_Arrangements.size() - 1;
	compression_Matrix.resize(original_makespan + 1);

	for (CompressionMatrix_2d_vector::iterator matrix_row = compression_Matrix.begin(); matrix_row != compression_Matrix.end(); ++matrix_row)
	{
	    matrix_row->resize(original_makespan + 1);
	}

	for (int start_step = 0; start_step < original_makespan; ++start_step)
	{
	    int final_step = start_step + 1;
	    CompressionRecord compression_record(1, start_step, final_step, NULL, NULL, original_solution.extract_Subsolution(start_step, final_step - 1));

	    compression_Matrix[start_step][final_step] = compression_record;
	}

	for (int start_step = 0; start_step < original_makespan - 1; ++start_step)
	{
	    int final_step = start_step + 2;

	    int optimal_makespan;
	    sMultirobotSolution optimal_solution;

	    result = compute_OptimalSolution(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], environment, sparse_environment, 3, optimal_makespan, optimal_solution);
	    if (sFAILED(result))
	    {
		return result;
	    }	    
	    CompressionRecord compression_record(optimal_makespan, start_step, final_step, NULL, NULL, sMultirobotSolution(start_step, optimal_solution));

	    compression_Matrix[start_step][final_step] = compression_record;
	}
	
	for (int makespan_distance = 3; makespan_distance <= original_makespan; ++makespan_distance)
	{
	    for (int start_step = 0; start_step < original_makespan - makespan_distance + 1; ++start_step)
	    {
		int final_step = start_step + makespan_distance;
		CompressionRecord compression_record(start_step, final_step);

		for (int left_component_span = 1; left_component_span < makespan_distance; ++left_component_span)
		{
		    int left_start_step = start_step;
		    int left_final_step = start_step + left_component_span;

		    int right_start_step = left_final_step;
		    int right_final_step = final_step;

		    int combined_makespan = compression_Matrix[left_start_step][left_final_step].m_makespan + compression_Matrix[right_start_step][right_final_step].m_makespan;

		    if (compression_record.m_makespan == MAKESPAN_UNDEFINED || combined_makespan < compression_record.m_makespan)
		    {
			compression_record.m_makespan = combined_makespan;
			
			compression_record.m_left_component = &compression_Matrix[left_start_step][left_final_step];
			compression_record.m_right_component = &compression_Matrix[right_start_step][right_final_step];
		    }
		}
		compression_record.m_compressed_sub_solution = sMultirobotSolution(compression_record.m_left_component->m_compressed_sub_solution,
										   compression_record.m_right_component->m_compressed_sub_solution);


		int makespan_lower_bound = calc_MakespanLowerBound(unfolded_Arrangements[start_step], unfolded_Arrangements[final_step], all_pairs_Distances);

		if (makespan_lower_bound <= m_makespan_upper_bound)
		{
		    int makespan_upper_bound = sMIN(m_makespan_upper_bound, final_step - start_step + 1);

		    int optimal_makespan;
		    sMultirobotSolution optimal_solution;	      

		    result = compute_OptimalSolution(unfolded_Arrangements[start_step],
						     unfolded_Arrangements[final_step],
						     environment,
						     sparse_environment,
						     makespan_lower_bound,
						     makespan_upper_bound,
						     optimal_makespan,
						     optimal_solution);
		    if (sFAILED(result))
		    {
			return result;
		    }
		    if (optimal_makespan != MAKESPAN_UNDEFINED && optimal_makespan < compression_record.m_makespan)
		    {
			compression_record.m_makespan = optimal_makespan;
			
			compression_record.m_left_component = NULL;
			compression_record.m_right_component = NULL;
			
			compression_record.m_compressed_sub_solution = sMultirobotSolution(start_step, optimal_solution);
		    }
		}

		compression_Matrix[start_step][final_step] = compression_record;
	    }
	}
	compressed_solution = compression_Matrix[0][original_makespan].m_compressed_sub_solution;
	compressed_solution.remove_EmptySteps();

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif
       
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::deflate_Solution(const sRobotArrangement   &initial_arrangement,
							    const sMultirobotSolution &original_solution,
							    sUndirectedGraph          &environment,
							    const sUndirectedGraph    &sparse_environment,
							    sMultirobotSolution       &deflated_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("deflation");
	}
        #endif

	result = optimize_Solution(initial_arrangement, original_solution, environment, sparse_environment, previous_solution);

	if (sFAILED(result))
	{
	    return result;
	}
	while (true)
	{
	    result = compress_Solution(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    deflated_solution = previous_solution;
		}
		else
		{
		    deflated_solution = next_solution;
		}
		break;
	    }
	    previous_solution = next_solution;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotSolutionCompressor::deflate_Solution_mt(const sRobotArrangement   &initial_arrangement,
							       const sMultirobotSolution &original_solution,
							       sUndirectedGraph          &environment,
							       const sUndirectedGraph    &sparse_environment,
							       sMultirobotSolution       &deflated_solution)
    {
	sResult result;
	sMultirobotSolution previous_solution;
	sMultirobotSolution next_solution;

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.enter_Phase("deflation");
	}
        #endif

	result = optimize_Solution_mt(initial_arrangement, original_solution, environment, sparse_environment, previous_solution);

	if (sFAILED(result))
	{
	    return result;
	}
	while (true)
	{
	    result = compress_Solution(initial_arrangement, previous_solution, environment, sparse_environment, next_solution);

	    if (sFAILED(result))
	    {
		return result;
	    }

	    if (previous_solution.get_StepCount() <= next_solution.get_StepCount())
	    {
		if (previous_solution.get_MoveCount() <= next_solution.get_MoveCount())
		{
		    deflated_solution = previous_solution;
		}
		else
		{
		    deflated_solution = next_solution;
		}
		break;
	    }
	    previous_solution = next_solution;
	}

        #ifdef sSTATISTICS
	{
	    s_GlobalPhaseStatistics.leave_Phase();
	}
        #endif

	return sRESULT_SUCCESS;
    }


    void sMultirobotSolutionCompressor::unfold_Solution(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, Arrangements_vector &unfolded_Arrangements)
    {
	unfolded_Arrangements.push_back(initial_arrangement);
	sRobotArrangement current_arrangement = initial_arrangement;

	for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		current_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	    }
	    unfolded_Arrangements.push_back(current_arrangement);
	}
	
    }

    
    void sMultirobotSolutionCompressor::force_Solution(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, Arrangements_vector &unfolded_Arrangements)
    {
	unfolded_Arrangements.push_back(initial_arrangement);
	sRobotArrangement current_arrangement = initial_arrangement;

	for (sMultirobotSolution::Steps_vector::const_iterator step =  solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		current_arrangement.force_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	    }
	    unfolded_Arrangements.push_back(current_arrangement);
	}
	
    }    


    bool sMultirobotSolutionCompressor::verify_Unfolding(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, const sUndirectedGraph &graph)
    {
	sRobotArrangement current_arrangement = initial_arrangement;

	for (sMultirobotSolution::Steps_vector::const_iterator step = solution.m_Steps.begin(); step != solution.m_Steps.end(); ++step)
	{
	    for (sMultirobotSolution::Moves_list::const_iterator move = step->m_Moves.begin(); move != step->m_Moves.end(); ++move)
	    {
		if (!current_arrangement.verify_Move(move->m_robot_id, move->m_dest_vrtx_id, graph))
		{
		    return false;
		}
		current_arrangement.move_Robot(move->m_robot_id, move->m_dest_vrtx_id);
	    }
	}

	return true;
    }


    void sMultirobotSolutionCompressor::to_Screen_arrangements(const Arrangements_vector &unfolded_Arrangements, const sString &indent) const
    {      
	to_Stream_arrangements(stdout, unfolded_Arrangements, indent);
    }


    void sMultirobotSolutionCompressor::to_Stream_arrangements(FILE *fw, const Arrangements_vector &unfolded_Arrangements, const sString &indent) const
    {
	fprintf(fw, "%sRobot arrangements [\n", indent.c_str());

	for (Arrangements_vector::const_iterator arrangement = unfolded_Arrangements.begin(); arrangement != unfolded_Arrangements.end(); ++arrangement)
	{
	    arrangement->to_Stream(fw, indent + sRELOC_INDENT);
	}

	fprintf(fw, "]\n");
    }


/*----------------------------------------------------------------------------*/
// Global functions

    void* s_shorten_Solution_mt(void *arg)
    {
	sMultirobotSolutionCompressor::ProcessingArgument &processing_argument = *reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(arg);

        #ifdef sVERBOSE
	{
	    printf("Thread %d started ...\n", processing_argument.m_ctx_arg.m_thread_id);
        }
        #endif

	processing_argument.m_out_arg.m_result = processing_argument.m_ctx_arg.m_compressor->shorten_Solution(processing_argument.m_in_arg.m_initial_arrangement,
													      processing_argument.m_in_arg.m_original_solution,
													      *processing_argument.m_in_arg.m_environment,
													      *processing_argument.m_in_arg.m_sparse_environment,
													      processing_argument.m_out_arg.m_processed_solution,
													      processing_argument.m_ctx_arg.m_thread_id);
	if (sFAILED(processing_argument.m_out_arg.m_result))
	{
	    return (reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(&processing_argument));
	}
	processing_argument.m_out_arg.m_result = sRESULT_SUCCESS;

        #ifdef sVERBOSE
	{
	    printf("Thread %d finished with ratio %.2f%%\n",
		   processing_argument.m_ctx_arg.m_thread_id,
		   100 * (double)processing_argument.m_out_arg.m_processed_solution.get_StepCount() / processing_argument.m_in_arg.m_original_solution.get_StepCount());
        }
        #endif

	return (reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(&processing_argument));
    }


    void* s_prime_shorten_Solution_mt(void *arg)
    {
	sMultirobotSolutionCompressor::ProcessingArgument &processing_argument = *reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(arg);

        #ifdef sVERBOSE
	{
	    printf("Thread %d started ...\n", processing_argument.m_ctx_arg.m_thread_id);
        }
        #endif

	processing_argument.m_out_arg.m_result = processing_argument.m_ctx_arg.m_compressor->primeShorten_Solution(processing_argument.m_in_arg.m_initial_arrangement,
														   processing_argument.m_in_arg.m_original_solution,
														   *processing_argument.m_in_arg.m_environment,
														   *processing_argument.m_in_arg.m_sparse_environment,
														   processing_argument.m_out_arg.m_processed_solution,
														   processing_argument.m_ctx_arg.m_thread_id);
	if (sFAILED(processing_argument.m_out_arg.m_result))
	{
	    return (reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(&processing_argument));
	}
	processing_argument.m_out_arg.m_result = sRESULT_SUCCESS;

        #ifdef sVERBOSE
	{
	    printf("Thread %d finished with ratio %.2f%%\n",
		   processing_argument.m_ctx_arg.m_thread_id,
		   100 * (double)processing_argument.m_out_arg.m_processed_solution.get_StepCount() / processing_argument.m_in_arg.m_original_solution.get_StepCount());
        }
        #endif

	return (reinterpret_cast<sMultirobotSolutionCompressor::ProcessingArgument*>(&processing_argument));
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc
