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
/* compress.h / 0.22-robik_103                                                */
/*----------------------------------------------------------------------------*/
//
// Compression tools for relocation problem solutions.
//
/*----------------------------------------------------------------------------*/


#ifndef __COMPRESS_H__
#define __COMPRESS_H__

#include <pthread.h>

#include <vector>

#include "types.h"
#include "result.h"
#include "multirobot.h"


using namespace std;
using namespace sReloc;

#ifdef sUSE_GLUCOSE
    #define sSATSolverBool_Type Glucose::lbool
#endif


#ifdef sUSE_MAPLE
    #define sSATSolverBool_Type Maple::lbool
#endif


#ifdef sUSE_COMSPS
    #define sSATSolverBool_Type Comsps::lbool
#endif


/*----------------------------------------------------------------------------*/

namespace sReloc
{    


/*----------------------------------------------------------------------------*/
// sMultirobotSolutionCompressor

    class sMultirobotSolutionCompressor
    {
    public:
	static const int MAKESPAN_UNDEFINED = -1;
	static const int MINISAT_TIMEOUT_UNDEFINED = -1;
	static const int TOTAL_TIMEOUT_UNDEFINED = -1;
	static const int THREAD_ID_UNDEFINED = -1;

	static const sString CNF_INVERSE_FILENAME_PREFIX;
	static const sString CNF_ADVANCED_FILENAME_PREFIX;
	static const sString CNF_DIFFERENTIAL_FILENAME_PREFIX;
	static const sString CNF_BIJECTION_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_ADVANCED_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_BIJECTION_FILENAME_PREFIX;
	static const sString CNF_BITWISE_FILENAME_PREFIX;
	static const sString CNF_FLOW_FILENAME_PREFIX;
	static const sString CNF_MATCHING_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_MATCHING_FILENAME_PREFIX;
	static const sString CNF_DIRECT_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_DIRECT_FILENAME_PREFIX;
	static const sString CNF_SIMPLICIAL_FILENAME_PREFIX;
	static const sString CNF_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX;
	static const sString CNF_SINGULAR_FILENAME_PREFIX;
	static const sString CNF_PLURAL_FILENAME_PREFIX;
	static const sString CNF_PLURAL2_FILENAME_PREFIX;
	static const sString CNF_HEIGHTED_FILENAME_PREFIX;
	static const sString CNF_MDD_FILENAME_PREFIX;
	static const sString CNF_MDD_UMTEX_FILENAME_PREFIX;
	static const sString CNF_MDD_MUTEX_FILENAME_PREFIX;		
	static const sString CNF_GMDD_FILENAME_PREFIX;
	static const sString CNF_GEMDD_FILENAME_PREFIX;
	static const sString CNF_ANO_FILENAME_PREFIX;
	static const sString CNF_GANO_FILENAME_PREFIX;	
	static const sString CNF_WATER_MDD_FILENAME_PREFIX;	
	static const sString CNF_RELAXED_MDD_FILENAME_PREFIX;
	static const sString CNF_TOKEN_MDD_FILENAME_PREFIX;
	static const sString CNF_TOKEN_EMPTY_MDD_FILENAME_PREFIX;	
	static const sString CNF_PERMUTATION_MDD_FILENAME_PREFIX;
	static const sString CNF_PERMUTATION_CMDD_FILENAME_PREFIX;	
	static const sString CNF_MMDD_FILENAME_PREFIX;
	static const sString CNF_RELAXED_MMDD_FILENAME_PREFIX;
	static const sString CNF_TOKEN_MMDD_FILENAME_PREFIX;
	static const sString CNF_TOKEN_EMPTY_MMDD_FILENAME_PREFIX;	
	static const sString CNF_PERMUTATION_MMDD_FILENAME_PREFIX;
	static const sString CNF_PERMUTATION_CMMDD_FILENAME_PREFIX;		
	static const sString CNF_MDD_plus_FILENAME_PREFIX;
	static const sString CNF_MMDD_plus_FILENAME_PREFIX;
	static const sString CNF_MDD_plus_plus_FILENAME_PREFIX;
	static const sString CNF_MDD_plus_plus_mutex_FILENAME_PREFIX;	
	static const sString CNF_MDD_plus_plus_fuel_FILENAME_PREFIX;	
	static const sString CNF_LMDD_plus_plus_FILENAME_PREFIX;	
	static const sString CNF_MDD_star_FILENAME_PREFIX;	
	static const sString CNF_MMDD_plus_plus_FILENAME_PREFIX;		
	static const sString CNF_RXMDD_FILENAME_PREFIX;
	static const sString CNF_NOMDD_FILENAME_PREFIX;
	static const sString CNF_RXNOMDD_FILENAME_PREFIX;
	static const sString CNF_ID_MDD_FILENAME_PREFIX;
	static const sString CNF_ID_WATER_MDD_FILENAME_PREFIX;	

	static const sString OUTPUT_INVERSE_FILENAME_PREFIX;
	static const sString OUTPUT_ADVANCED_FILENAME_PREFIX;
	static const sString OUTPUT_DIFFERENTIAL_FILENAME_PREFIX;
	static const sString OUTPUT_BIJECTION_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_ADVANCED_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_DIFFERENTIAL_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_BIJECTION_FILENAME_PREFIX;
	static const sString OUTPUT_BITWISE_FILENAME_PREFIX;
	static const sString OUTPUT_FLOW_FILENAME_PREFIX;
	static const sString OUTPUT_MATCHING_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_MATCHING_FILENAME_PREFIX;
	static const sString OUTPUT_DIRECT_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_DIRECT_FILENAME_PREFIX;
	static const sString OUTPUT_SIMPLICIAL_FILENAME_PREFIX;
	static const sString OUTPUT_HEURISTIC_SIMPLICIAL_FILENAME_PREFIX;
	static const sString OUTPUT_SINGULAR_FILENAME_PREFIX;
	static const sString OUTPUT_PLURAL_FILENAME_PREFIX;
	static const sString OUTPUT_PLURAL2_FILENAME_PREFIX;
	static const sString OUTPUT_HEIGHTED_FILENAME_PREFIX;
	static const sString OUTPUT_MDD_FILENAME_PREFIX;
	static const sString OUTPUT_MDD_UMTEX_FILENAME_PREFIX;
	static const sString OUTPUT_MDD_MUTEX_FILENAME_PREFIX;		
	static const sString OUTPUT_GMDD_FILENAME_PREFIX;
	static const sString OUTPUT_GEMDD_FILENAME_PREFIX;
	static const sString OUTPUT_ANO_FILENAME_PREFIX;
	static const sString OUTPUT_GANO_FILENAME_PREFIX;
	static const sString OUTPUT_WATER_MDD_FILENAME_PREFIX;	
	static const sString OUTPUT_RELAXED_MDD_FILENAME_PREFIX;
	static const sString OUTPUT_TOKEN_MDD_FILENAME_PREFIX;
	static const sString OUTPUT_TOKEN_EMPTY_MDD_FILENAME_PREFIX;	
	static const sString OUTPUT_PERMUTATION_MDD_FILENAME_PREFIX;
	static const sString OUTPUT_PERMUTATION_CMDD_FILENAME_PREFIX;				
	static const sString OUTPUT_MMDD_FILENAME_PREFIX;
	static const sString OUTPUT_RELAXED_MMDD_FILENAME_PREFIX;
	static const sString OUTPUT_TOKEN_MMDD_FILENAME_PREFIX;
	static const sString OUTPUT_TOKEN_EMPTY_MMDD_FILENAME_PREFIX;	
	static const sString OUTPUT_PERMUTATION_MMDD_FILENAME_PREFIX;
	static const sString OUTPUT_PERMUTATION_CMMDD_FILENAME_PREFIX;		
	static const sString OUTPUT_MDD_plus_FILENAME_PREFIX;
	static const sString OUTPUT_MMDD_plus_FILENAME_PREFIX;
	static const sString OUTPUT_MDD_plus_plus_FILENAME_PREFIX;
	static const sString OUTPUT_MDD_plus_plus_mutex_FILENAME_PREFIX;	
	static const sString OUTPUT_MDD_plus_plus_fuel_FILENAME_PREFIX;	
	static const sString OUTPUT_LMDD_plus_plus_FILENAME_PREFIX;	
	static const sString OUTPUT_MDD_star_FILENAME_PREFIX;	
	static const sString OUTPUT_MMDD_plus_plus_FILENAME_PREFIX;		
	static const sString OUTPUT_RXMDD_FILENAME_PREFIX;
	static const sString OUTPUT_NOMDD_FILENAME_PREFIX;
	static const sString OUTPUT_RXNOMDD_FILENAME_PREFIX;
	static const sString OUTPUT_ID_MDD_FILENAME_PREFIX;
	static const sString OUTPUT_ID_WATER_MDD_FILENAME_PREFIX;

    public:
	enum Encoding
	{
	    ENCODING_UNDEFINED,
	    ENCODING_UNUSED,
	    ENCODING_INVERSE,
	    ENCODING_ADVANCED,
	    ENCODING_DIFFERENTIAL,
	    ENCODING_BIJECTION,
	    ENCODING_HEURISTIC_ADVANCED,
	    ENCODING_HEURISTIC_DIFFERENTIAL,
	    ENCODING_HEURISTIC_BIJECTION,
	    ENCODING_PUZZLE,
	    ENCODING_BITWISE,
	    ENCODING_FLOW,
	    ENCODING_MATCHING,
	    ENCODING_HEURISTIC_MATCHING,
	    ENCODING_DIRECT,
	    ENCODING_HEURISTIC_DIRECT,
	    ENCODING_SIMPLICIAL,
	    ENCODING_HEURISTIC_SIMPLICIAL,
	    ENCODING_SINGULAR,
	    ENCODING_PLURAL,
	    ENCODING_PLURAL2,
	    ENCODING_HEIGHTED,
	    ENCODING_MDD,
	    ENCODING_MDD_UMTEX,
	    ENCODING_MDD_MUTEX,
	    ENCODING_GMDD,
	    ENCODING_GEMDD,
	    ENCODING_ANO,
	    ENCODING_GANO,	    
	    ENCODING_WATER_MDD,
	    ENCODING_RELAXED_MDD,
	    ENCODING_TOKEN_MDD,
	    ENCODING_TOKEN_EMPTY_MDD,	    
	    ENCODING_PERMUTATION_MDD,	    	    	    
	    ENCODING_MMDD,
	    ENCODING_RELAXED_MMDD,
	    ENCODING_TOKEN_MMDD,
	    ENCODING_TOKEN_EMPTY_MMDD,	    
	    ENCODING_PERMUTATION_MMDD,	    	    	    
	    ENCODING_MDD_plus,
	    ENCODING_MMDD_plus,
	    ENCODING_MDD_plus_plus,
	    ENCODING_MDD_plus_plus_mutex,	    
	    ENCODING_MDD_plus_plus_fuel,	    
	    ENCODING_LMDD_plus_plus,	    
	    ENCODING_MDD_star,	    
	    ENCODING_MMDD_plus_plus,
	    ENCODING_RXMDD,
	    ENCODING_RXMDD_BINARY,
	    ENCODING_NOMDD,
	    ENCODING_RXNOMDD,
	    ENCODING_BMDD,
	    ENCODING_BCMDD,
	    ENCODING_BNOMDD,
	    ENCODING_BCNOMDD,
	    ENCODING_ID_MDD,
	    ENCODING_ID_WATER_MDD,	    
	    ENCODING_ID_MDD_plus,
	    ENCODING_ID_MDD_plus_plus,
	    ENCODING_ID_MDD_star,	    
	    ENCODING_AD_MDD,
	    ENCODING_AD_WATER_MDD,
	    ENCODING_AD_MDD_plus,
	    ENCODING_AD_MDD_plus_plus,
	    ENCODING_AD_MDD_star,
	    ENCODING_AD_MMDD,
	    ENCODING_AD_MMDD_plus,
	    ENCODING_AD_MMDD_plus_plus,
	    ENCODING_PERMUTATION_CMDD,
	    ENCODING_PERMUTATION_CMMDD	    
	};

	typedef std::vector<sRobotArrangement> Arrangements_vector;

	struct CompressionRecord
	{
	    CompressionRecord();
	    CompressionRecord(int start_step, int final_step);

	    CompressionRecord(int                        makespan,
			      int                        start_step,
			      int                        final_step,
			      CompressionRecord         *left_component,
			      CompressionRecord         *right_component,
			      const sMultirobotSolution &compressed_sub_solution);

	    int m_makespan;
	    int m_start_step;
	    int m_final_step;

	    CompressionRecord *m_left_component;
	    CompressionRecord *m_right_component;

	    sMultirobotSolution m_compressed_sub_solution;
	};

	typedef std::vector<std::vector<CompressionRecord> > CompressionMatrix_2d_vector;

	struct ProcessingArgument_IN
	{
	    ProcessingArgument_IN();
	    ProcessingArgument_IN(const sRobotArrangement   &initial_arrangement,
				  const sMultirobotSolution &original_solution,
				  sUndirectedGraph          &environment,
				  const sUndirectedGraph    &sparse_environment);

	    int m_start_step;
	    int m_final_step;

	    sRobotArrangement m_initial_arrangement;
	    sMultirobotSolution m_original_solution;
	    sUndirectedGraph *m_environment;
	    const sUndirectedGraph *m_sparse_environment;
	};

	struct ProcessingArgument_OUT
	{
	    ProcessingArgument_OUT();

	    sResult m_result;
	    sMultirobotSolution m_processed_solution;
	};

	struct ProcessingArgument_CTX
	{
	    ProcessingArgument_CTX();
	    ProcessingArgument_CTX(sMultirobotSolutionCompressor &compressor);

	    sMultirobotSolutionCompressor *m_compressor;
	    int m_thread_id;
	    pthread_t m_pthread_handle;
	};

	struct ProcessingArgument
	{
	    ProcessingArgument();
	    ProcessingArgument(const ProcessingArgument_CTX &ctx_arg, const ProcessingArgument_IN &in_arg);

	    ProcessingArgument_CTX m_ctx_arg;
	    ProcessingArgument_IN m_in_arg;
	    ProcessingArgument_OUT m_out_arg;
	};

	typedef std::vector<ProcessingArgument> ProcessingArguments_vector;

	struct ProcessedSolutionRecord
	{
	    ProcessedSolutionRecord();
	    ProcessedSolutionRecord(int start_step, int final_step, const sMultirobotSolution &short_solution);

	    bool operator==(const ProcessedSolutionRecord &processed_record) const;
	    bool operator<(const ProcessedSolutionRecord &processed_record) const;
	    
	    int m_start_step;
	    int m_final_step;

	    sMultirobotSolution m_processed_solution;
	};

	typedef std::set<ProcessedSolutionRecord, std::less<ProcessedSolutionRecord> > ProcessedSolutionRecords_set;

	struct AttemptDatabaseRecord
	{
	    AttemptDatabaseRecord();
	    AttemptDatabaseRecord(const sRobotArrangement &initial_arrangement, const sRobotArrangement &goal_arrangement);

	    bool operator==(const AttemptDatabaseRecord &attempt_record) const;
	    bool operator<(const AttemptDatabaseRecord &attempt_record) const;

	    sRobotArrangement m_initial_arrangement;
	    sRobotArrangement m_goal_arrangement;
	};

	typedef std::set<AttemptDatabaseRecord, std::less<AttemptDatabaseRecord> > AttemptDatabaseRecords_set;

	enum Solvability
	{
	    SOLVABILITY_UNDEFINED,
	    SOLVABILITY_SAT,
	    SOLVABILITY_UNSAT,
	    SOLVABILITY_INDET
	};

	typedef std::vector<int> RobotIDs_vector;
	typedef std::vector<RobotIDs_vector> RobotGroups_vector;

	typedef std::vector<sRobotArrangement> RobotArrangements_vector;
	typedef std::vector<sRobotGoal> RobotGoals_vector;
	typedef std::vector<sMultirobotSolution> MultirobotSolutions_vector;
	
	typedef std::vector<int> AdjacencyRow_vector;
	typedef std::vector<AdjacencyRow_vector> AdjacencyMatrix_vector;

	typedef std::vector<int> SolutionCosts_vector;
	typedef std::vector<int> SolutionMakespans_vector;
	
    public:
	sMultirobotSolutionCompressor(const sString &minisat_path,
				      int            minisat_timeout       = MINISAT_TIMEOUT_UNDEFINED,
				      int            total_timeout         = TOTAL_TIMEOUT_UNDEFINED,
				      int            makespan_upper_bound  = sDEFAULT_MINISAT_UPPER_BOUND,
	                              int            N_parallel_Threads    = sDEFAULT_N_PARALLEL_THREADS,
				      Encoding       encoding              = ENCODING_INVERSE);

	void set_Ratio(double ratio);
	void set_Robustness(int robustness);
	void set_Range(int range);	

	int calc_MakespanLowerBound(const sRobotArrangement                     &start_arrangement,
				    const sRobotArrangement                     &final_arrangement,
				    const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances);

	int calc_MakespanLowerBound(const sRobotArrangement                     &start_arrangement,
				    const sRobotGoal                            &robot_goal,
				    const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances);

	sResult compute_OptimalMakespan(sMultirobotInstance     &instance,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult compute_SpecifiedMakespan(sMultirobotInstance     &instance,
					  int                     &specified_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult incompute_OptimalMakespan(sSATSolver_Type        **solver,
					  sMultirobotInstance     &instance,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SpecifiedMakespan(sSATSolver_Type        **solver,
					    sMultirobotInstance     &instance,
					    int                     &specified_makespan,
					    int                      thread_id = THREAD_ID_UNDEFINED);
	
	sResult compute_OptimalMakespan(const sRobotArrangement &start_arrangement,
					const sRobotGoal        &final_arrangement,
					const sUndirectedGraph  &environment,
					const sUndirectedGraph  &sparse_environment,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult compute_SpecifiedMakespan(const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                     &specified_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult incompute_OptimalMakespan(sSATSolver_Type        **solver,
					  const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type        **solver,
					  const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                     &specified_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);		

	sResult compute_OptimalMakespan_(sMultirobotInstance     &instance,
					 int                      makespan_upper_bound,
					 int                     &optimal_makespan,
					 int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_(sSATSolver_Type        **solver,
					   sMultirobotInstance     &instance,
					   int                      makespan_upper_bound,
					   int                     &optimal_makespan,
					   int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalMakespan_(const sRobotArrangement &start_arrangement,
					 const sRobotGoal        &final_arrangement,
					 const sUndirectedGraph  &environment,
					 const sUndirectedGraph  &sparse_environment,
					 int                      makespan_upper_bound,
					 int                     &optimal_makespan,
					 int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_(sSATSolver_Type        **solver,
					   const sRobotArrangement &start_arrangement,
					   const sRobotGoal        &final_arrangement,
					   const sUndirectedGraph  &environment,
					   const sUndirectedGraph  &sparse_environment,
					   int                      makespan_upper_bound,
					   int                     &optimal_makespan,
					   int                      thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalMakespan(sMultirobotInstance     &instance,
					int                      makespan_lower_bound,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type        **solver,
					  sMultirobotInstance     &instance,
					  int                      makespan_lower_bound,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalMakespan(const sRobotArrangement &start_arrangement,
					const sRobotGoal        &final_arrangement,
					const sUndirectedGraph  &environment,
					const sUndirectedGraph  &sparse_environment,
					int                      makespan_lower_bound,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type        **solver,
					  const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                      makespan_lower_bound,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalMakespan(sMultirobotInstance               &instance,
					int                                makespan_upper_bound,
					int                               &optimal_makespan,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id = THREAD_ID_UNDEFINED);

	sResult compute_SpecifiedMakespan(sMultirobotInstance               &instance,
					  int                               &specified_makespan,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult incompute_OptimalMakespan(sSATSolver_Type                  **solver,
					  sMultirobotInstance               &instance,
					  int                                makespan_upper_bound,
					  int                               &optimal_makespan,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SpecifiedMakespan(sSATSolver_Type                  **solver,
					    sMultirobotInstance               &instance,
					    int                               &specified_makespan,
					    sMultirobotEncodingContext_CNFsat &final_encoding_context,
					    int                                thread_id = THREAD_ID_UNDEFINED);		
	
	sResult compute_OptimalMakespan(const sRobotArrangement           &start_arrangement,
					const sRobotGoal                  &final_arrangement,
					const sUndirectedGraph            &environment,
					const sUndirectedGraph            &sparse_environment,
					int                                makespan_upper_bound,
					int                               &optimal_makespan,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type                  **solver,
					  const sRobotArrangement           &start_arrangement,
					  const sRobotGoal                  &final_arrangement,
					  const sUndirectedGraph            &environment,
					  const sUndirectedGraph            &sparse_environment,
					  int                                makespan_upper_bound,
					  int                               &optimal_makespan,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalMakespan_(sMultirobotInstance               &instance,
					 int                                makespan_upper_bound,
					 int                               &optimal_makespan,
					 sMultirobotEncodingContext_CNFsat &final_encoding_context,
					 int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_(sSATSolver_Type                  **solver,
					   sMultirobotInstance               &instance,
					   int                                makespan_upper_bound,
					   int                               &optimal_makespan,
					   sMultirobotEncodingContext_CNFsat &final_encoding_context,
					   int                                thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalMakespan_(const sRobotArrangement           &start_arrangement,
					 const sRobotGoal                  &final_arrangement,
					 const sUndirectedGraph            &environment,
					 const sUndirectedGraph            &sparse_environment,
					 int                                makespan_upper_bound,
					 int                               &optimal_makespan,
					 sMultirobotEncodingContext_CNFsat &final_encoding_context,
					 int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_(sSATSolver_Type                  **solver,
					   const sRobotArrangement           &start_arrangement,
					   const sRobotGoal                  &final_arrangement,
					   const sUndirectedGraph            &environment,
					   const sUndirectedGraph            &sparse_environment,
					   int                                makespan_upper_bound,
					   int                               &optimal_makespan,
					   sMultirobotEncodingContext_CNFsat &final_encoding_context,
					   int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalMakespan(sMultirobotInstance               &instance,
					int                                makespan_lower_bound,
					int                                makespan_upper_bound,
					int                               &optimal_makespan,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type                  **solver,
					  sMultirobotInstance               &instance,
					  int                                makespan_lower_bound,
					  int                                makespan_upper_bound,
					  int                               &optimal_makespan,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalMakespan(const sRobotArrangement           &start_arrangement,
					const sRobotGoal                  &final_arrangement,
					const sUndirectedGraph            &environment,
					const sUndirectedGraph            &sparse_environment,
					int                                makespan_lower_bound,
					int                                makespan_upper_bound,
					int                               &optimal_makespan,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan(sSATSolver_Type                  **solver,
					  const sRobotArrangement           &start_arrangement,
					  const sRobotGoal                  &final_arrangement,
					  const sUndirectedGraph            &environment,
					  const sUndirectedGraph            &sparse_environment,
					  int                                makespan_lower_bound,
					  int                                makespan_upper_bound,
					  int                               &optimal_makespan,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);

	
/*----------------------------------------------------------------------------*/

	sResult compute_OptimalSolution(const sRobotArrangement &start_arrangement,
					const sRobotGoal        &final_arrangement,
					const sUndirectedGraph  &environment,
					const sUndirectedGraph  &sparse_environment,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					sMultirobotSolution     &optimal_solution,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult compute_SpecifiedSolution(const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                     &specified_makespan,
					  sMultirobotSolution     &specified_solution,
					  int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult incompute_OptimalSolution(sSATSolver_Type        **solver,
					  const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  sMultirobotSolution     &optimal_solution,
					  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SpecifiedSolution(sSATSolver_Type        **solver,
					    const sRobotArrangement &start_arrangement,
					    const sRobotGoal        &final_arrangement,
					    const sUndirectedGraph  &environment,
					    const sUndirectedGraph  &sparse_environment,
					    int                     &specified_makespan,
					    sMultirobotSolution     &specified_solution,
					    int                      thread_id = THREAD_ID_UNDEFINED);		
	
	sResult compute_OptimalSolution_(const sRobotArrangement &start_arrangement,
					 const sRobotGoal        &final_arrangement,
					 const sUndirectedGraph  &environment,
					 const sUndirectedGraph  &sparse_environment,
					 int                      makespan_upper_bound,
					 int                     &optimal_makespan,
					 sMultirobotSolution     &optimal_solution,
					 int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalSolution_(sSATSolver_Type        **solver,
					   const sRobotArrangement &start_arrangement,
					   const sRobotGoal        &final_arrangement,
					   const sUndirectedGraph  &environment,
					   const sUndirectedGraph  &sparse_environment,
					   int                      makespan_upper_bound,
					   int                     &optimal_makespan,
					   sMultirobotSolution     &optimal_solution,
					   int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalSolution(const sRobotArrangement &start_arrangement,
					const sRobotGoal        &final_arrangement,
					const sUndirectedGraph  &environment,
					const sUndirectedGraph  &sparse_environment,
					int                      makespan_lower_bound,
					int                      makespan_upper_bound,
					int                     &optimal_makespan,
					sMultirobotSolution     &optimal_solution,
					int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalSolution(sSATSolver_Type        **solver,
					  const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                      makespan_lower_bound,
					  int                      makespan_upper_bound,
					  int                     &optimal_makespan,
					  sMultirobotSolution     &optimal_solution,
					  int                      thread_id = THREAD_ID_UNDEFINED);	

/*----------------------------------------------------------------------------*/

	sResult compute_OrtoOptimalMakespan(const sRobotArrangement           &start_arrangement,
					    const sRobotGoal                  &final_arrangement,
					    const sUndirectedGraph            &environment,
					    const sUndirectedGraph            &sparse_environment,
					    int                                layer_upper_bound,
					    int                               &optimal_makespan,
					    sMultirobotEncodingContext_CNFsat &final_encoding_context,
					    int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OrtoOptimalMakespan(sSATSolver_Type                  **solver,
					      const sRobotArrangement           &start_arrangement,
					      const sRobotGoal                  &final_arrangement,
					      const sUndirectedGraph            &environment,
					      const sUndirectedGraph            &sparse_environment,
					      int                                layer_upper_bound,
					      int                               &optimal_makespan,
					      sMultirobotEncodingContext_CNFsat &final_encoding_context,
					      int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OrtoOptimalSolution(const sRobotArrangement &start_arrangement,
					    const sRobotGoal        &final_arrangement,
					    const sUndirectedGraph  &environment,
					    const sUndirectedGraph  &sparse_environment,
					    int                      layer_upper_bound,
					    int                     &optimal_makespan,
					    sMultirobotSolution     &optimal_solution,
					    int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OrtoOptimalSolution(sSATSolver_Type        **solver,
					      const sRobotArrangement &start_arrangement,
					      const sRobotGoal        &final_arrangement,
					      const sUndirectedGraph  &environment,
					      const sUndirectedGraph  &sparse_environment,
					      int                      layer_upper_bound,
					      int                     &optimal_makespan,
					      sMultirobotSolution     &optimal_solution,
					      int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalCost(const sRobotArrangement           &start_arrangement,
				    const sRobotGoal                  &final_arrangement,
				    const sUndirectedGraph            &environment,
				    const sUndirectedGraph            &sparse_environment,
				    int                                max_total_cost,
				    int                               &optimal_cost,
				    int                               &expansion_count,
				    sMultirobotEncodingContext_CNFsat &final_encoding_context,
				    int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalCost(sSATSolver_Type                  **solver,
				      const sRobotArrangement           &start_arrangement,
				      const sRobotGoal                  &final_arrangement,
				      const sUndirectedGraph            &environment,
				      const sUndirectedGraph            &sparse_environment,
				      int                                max_total_cost,
				      int                               &optimal_cost,
				      int                               &expansion_count,
				      sMultirobotEncodingContext_CNFsat &final_encoding_context,
				      int                                thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalCost(sMultirobotInstance               &instance,
				    int                                max_total_cost,
				    int                               &optimal_cost,
				    int                               &expansion_count,
				    sMultirobotEncodingContext_CNFsat &final_encoding_context,
				    int                                thread_id = THREAD_ID_UNDEFINED);
	
	sResult incompute_OptimalCost(sSATSolver_Type                  **solver,
				      sMultirobotInstance               &instance,
				      int                                max_total_cost,
				      int                               &optimal_cost,
				      int                               &expansion_count,
				      sMultirobotEncodingContext_CNFsat &final_encoding_context,
				      int                                thread_id = THREAD_ID_UNDEFINED);

	sResult compute_OptimalFuel(const sRobotArrangement           &start_arrangement,
				    const sRobotGoal                  &final_arrangement,
				    const sUndirectedGraph            &environment,
				    const sUndirectedGraph            &sparse_environment,
				    int                                max_total_fuel,
				    int                               &optimal_fuel,
				    int                               &fuel_makespan,
				    int                               &expansion_count,
				    sMultirobotEncodingContext_CNFsat &final_encoding_context,
				    int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalFuel(sSATSolver_Type                  **solver,
				      const sRobotArrangement           &start_arrangement,
				      const sRobotGoal                  &final_arrangement,
				      const sUndirectedGraph            &environment,
				      const sUndirectedGraph            &sparse_environment,
				      int                                max_total_fuel,
				      int                               &optimal_fuel,
				      int                               &fuel_makespan,
				      int                               &expansion_count,
				      sMultirobotEncodingContext_CNFsat &final_encoding_context,
				      int                                thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_OptimalFuel(sMultirobotInstance               &instance,
				    int                                max_total_fuel,
				    int                               &optimal_fuel,
				    int                               &fuel_makespan,
				    int                               &expansion_count,
				    sMultirobotEncodingContext_CNFsat &final_encoding_context,
				    int                                thread_id = THREAD_ID_UNDEFINED);
	
	sResult incompute_OptimalFuel(sSATSolver_Type                  **solver,
				      sMultirobotInstance               &instance,
				      int                                max_total_fuel,
				      int                               &optimal_fuel,
				      int                               &fuel_makespan,
				      int                               &expansion_count,
				      sMultirobotEncodingContext_CNFsat &final_encoding_context,
				      int                                thread_id = THREAD_ID_UNDEFINED);		

	sResult compute_OptimalCost_avoid(sMultirobotInstance               &instance,
					  const Arrangements_vector         &blocking_solution,
					  int                                optimal_cost,					  
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalCost_avoid(sSATSolver_Type                  **solver,
					    sMultirobotInstance               &instance,
					    const Arrangements_vector         &blocking_solution,
					    int                                optimal_cost,					  
					    sMultirobotEncodingContext_CNFsat &final_encoding_context,
					    int                                thread_id = THREAD_ID_UNDEFINED);		

	sResult compute_OptimalCost_avoid(const sRobotArrangement           &start_arrangement,
					  const sRobotGoal                  &final_arrangement,
					  const sUndirectedGraph            &environment,
					  const sUndirectedGraph            &sparse_environment,
					  const Arrangements_vector         &blocking_solution,
					  int                                optimal_cost,					  
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalCost_avoid(sSATSolver_Type                  **solver,
					    const sRobotArrangement           &start_arrangement,
					    const sRobotGoal                  &final_arrangement,
					    const sUndirectedGraph            &environment,
					    const sUndirectedGraph            &sparse_environment,
					    const Arrangements_vector         &blocking_solution,
					    int                                optimal_cost,					  
					    sMultirobotEncodingContext_CNFsat &final_encoding_context,
					    int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_OptimalMakespan_avoid(sMultirobotInstance               &instance,
					      const Arrangements_vector         &blocking_solution,
					      int                                optimal_makespan,					  
					      sMultirobotEncodingContext_CNFsat &final_encoding_context,
					      int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_avoid(sSATSolver_Type                  **solver,
						sMultirobotInstance               &instance,
						const Arrangements_vector         &blocking_solution,
						int                                optimal_makespan,					  
						sMultirobotEncodingContext_CNFsat &final_encoding_context,
						int                                thread_id = THREAD_ID_UNDEFINED);			
       
	sResult compute_OptimalMakespan_avoid(const sRobotArrangement           &start_arrangement,
					      const sRobotGoal                  &final_arrangement,
					      const sUndirectedGraph            &environment,
					      const sUndirectedGraph            &sparse_environment,
					      const Arrangements_vector         &blocking_solution,
					      int                                optimal_makespan,					  
					      sMultirobotEncodingContext_CNFsat &final_encoding_context,
					      int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalMakespan_avoid(sSATSolver_Type                  **solver,
						const sRobotArrangement           &start_arrangement,
						const sRobotGoal                  &final_arrangement,
						const sUndirectedGraph            &environment,
						const sUndirectedGraph            &sparse_environment,
						const Arrangements_vector         &blocking_solution,
						int                                optimal_makespan,					  
						sMultirobotEncodingContext_CNFsat &final_encoding_context,
						int                                thread_id = THREAD_ID_UNDEFINED);			

	sResult compute_OptimalCost_binary(const sRobotArrangement           &start_arrangement,
					   const sRobotGoal                  &final_arrangement,
					   sUndirectedGraph                  &environment,
					   const sUndirectedGraph            &sparse_environment,
					   int                                max_total_cost,
					   int                               &optimal_cost,
					   sMultirobotEncodingContext_CNFsat &final_encoding_context,
					   int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_OptimalCost_binary(sSATSolver_Type                  **solver,
					     const sRobotArrangement           &start_arrangement,
					     const sRobotGoal                  &final_arrangement,
					     sUndirectedGraph                  &environment,
					     const sUndirectedGraph            &sparse_environment,
					     int                                max_total_cost,
					     int                               &optimal_cost,
					     sMultirobotEncodingContext_CNFsat &final_encoding_context,
					     int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_CostSolvability(sMultirobotInstance               &instance,
					int                                total_cost,
					int                                extra_cost,
					Solvability                       &solvability,
					sMultirobotSolution               &solution,
					sMultirobotEncodingContext_CNFsat &encoding_context,
					int                                thread_id);

	sResult incompute_CostSolvability(sSATSolver_Type                  **solver,
					  sMultirobotInstance               &instance,
					  int                                total_cost,
					  int                                extra_cost,
					  Solvability                       &solvability,
					  sMultirobotSolution               &solution,
					  sMultirobotEncodingContext_CNFsat &encoding_context,
					  int                                thread_id);	

	sResult compute_CostSolvability(sMultirobotInstance               &instance,
					int                                total_cost,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id);

	sResult incompute_CostSolvability(sSATSolver_Type                  **solver,
					  sMultirobotInstance               &instance,
					  int                                total_cost,
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id);

	sResult compute_FuelSolvability(sMultirobotInstance               &instance,
					int                                total_fuel,
					int                                fuel_makespan,
					sMultirobotEncodingContext_CNFsat &final_encoding_context,
					int                                thread_id);

	sResult incompute_FuelSolvability(sSATSolver_Type                  **solver,
					  sMultirobotInstance               &instance,
					  int                                total_fuel,
					  int                                fuel_makespan,					  
					  sMultirobotEncodingContext_CNFsat &final_encoding_context,
					  int                                thread_id);		

	sResult compute_CostSolvability_avoid(sMultirobotInstance               &instance,
					      int                                total_cost,
					      const Arrangements_vector         &blocking_solution,
					      sMultirobotEncodingContext_CNFsat &final_encoding_context,
					      int                                thread_id);

	sResult incompute_CostSolvability_avoid(sSATSolver_Type                  **solver,
						sMultirobotInstance               &instance,
						int                                total_cost,
						const Arrangements_vector         &blocking_solution,
						sMultirobotEncodingContext_CNFsat &final_encoding_context,
						int                                thread_id);	

	sResult compute_MakespanSolvability_avoid(sMultirobotInstance               &instance,
						  int                                makespan,
						  const Arrangements_vector         &blocking_solution,
						  sMultirobotEncodingContext_CNFsat &final_encoding_context,
						  int                                thread_id);

	sResult incompute_MakespanSolvability_avoid(sSATSolver_Type                  **solver,
						    sMultirobotInstance               &instance,
						    int                                makespan,
						    const Arrangements_vector         &blocking_solution,
						    sMultirobotEncodingContext_CNFsat &final_encoding_context,
						    int                                thread_id);			
	
	sResult compute_BestExtraCost_linear(sMultirobotInstance               &instance,
					     int                                total_cost,
					     int                                lower_extra_cost,
					     int                                upper_extra_cost,
					     int                               &best_extra_cost,
					     sMultirobotSolution               &solution,
					     sMultirobotEncodingContext_CNFsat &encoding_context,
					     int                                thread_id);

	sResult incompute_BestExtraCost_linear(sSATSolver_Type                  **solver,
					       sMultirobotInstance               &instance,
					       int                                total_cost,
					       int                                lower_extra_cost,
					       int                                upper_extra_cost,
					       int                               &best_extra_cost,
					       sMultirobotSolution               &solution,
					       sMultirobotEncodingContext_CNFsat &encoding_context,
					       int                                thread_id);	
	
	sResult compute_BestExtraCost_binary(sMultirobotInstance               &instance,
					     int                                total_cost,
					     int                                lower_extra_cost,
					     int                                upper_extra_cost,
					     int                               &best_extra_cost,
					     sMultirobotSolution               &solution,
					     sMultirobotEncodingContext_CNFsat &encoding_context,
					     int                                thread_id);

	sResult incompute_BestExtraCost_binary(sSATSolver_Type                  **solver,
					       sMultirobotInstance               &instance,
					       int                                total_cost,
					       int                                lower_extra_cost,
					       int                                upper_extra_cost,
					       int                               &best_extra_cost,
					       sMultirobotSolution               &solution,
					       sMultirobotEncodingContext_CNFsat &encoding_context,
					       int                                thread_id);
	
	sResult compute_CostReduction(sMultirobotInstance &instance,
				      int                  max_total_cost,
				      int                 &cost_reduction,
				      int                  thread_id);

	sResult incompute_CostReduction(sSATSolver_Type    **solver,
					sMultirobotInstance &instance,
					int                  max_total_cost,
					int                 &cost_reduction,
					int                  thread_id);
	
	sResult compute_BestCost(const sRobotArrangement           &start_arrangement,
				 const sRobotGoal                  &final_arrangement,
				 sUndirectedGraph                  &environment,
				 const sUndirectedGraph            &sparse_environment,
				 int                                max_total_cost,
				 int                               &optimal_cost,
				 sMultirobotSolution               &optimal_solution,
				 sMultirobotEncodingContext_CNFsat &final_encoding_context,
				 int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_BestCost(sSATSolver_Type                  **solver,
				   const sRobotArrangement           &start_arrangement,
				   const sRobotGoal                  &final_arrangement,
				   sUndirectedGraph                  &environment,
				   const sUndirectedGraph            &sparse_environment,
				   int                                max_total_cost,
				   int                               &optimal_cost,
				   sMultirobotSolution               &optimal_solution,
				   sMultirobotEncodingContext_CNFsat &final_encoding_context,
				   int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_UpperCost(const sRobotArrangement           &start_arrangement,
				  const sRobotGoal                  &final_arrangement,
				  sUndirectedGraph                  &environment,
				  const sUndirectedGraph            &sparse_environment,
				  int                                max_total_cost,
				  int                               &optimal_cost,
				  sMultirobotEncodingContext_CNFsat &final_encoding_context,
				  int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_UpperCost(sSATSolver_Type                  **solver,
				    const sRobotArrangement           &start_arrangement,
				    const sRobotGoal                  &final_arrangement,
				    sUndirectedGraph                  &environment,
				    const sUndirectedGraph            &sparse_environment,
				    int                                max_total_cost,
				    int                               &optimal_cost,
				    sMultirobotEncodingContext_CNFsat &final_encoding_context,
				    int                                thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_CostOptimalSolution(const sRobotArrangement                        &start_arrangement,
					    const sRobotGoal                               &final_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sUndirectedGraph                         &sparse_environment,
					    const sMultirobotInstance::Environments_vector &heighted_Environments,
					    int                                             max_total_cost,
					    int                                            &optimal_cost,
					    sMultirobotSolution                            &optimal_solution,
					    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolution(sSATSolver_Type                               **solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sRobotGoal                               &final_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sUndirectedGraph                         &sparse_environment,
					      const sMultirobotInstance::Environments_vector &heighted_Environments,
					      int                                             max_total_cost,
					      int                                            &optimal_cost,
					      sMultirobotSolution                            &optimal_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_CostOptimalSolution(const sRobotArrangement                        &start_arrangement,
					    const sRobotGoal                               &final_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sUndirectedGraph                         &sparse_environment,
					    const sMultirobotInstance::MDD_vector          &MDD,
					    int                                             max_total_cost,
					    int                                            &optimal_cost,
					    sMultirobotSolution                            &optimal_solution,
					    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolution(sSATSolver_Type                               **solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sRobotGoal                               &final_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sUndirectedGraph                         &sparse_environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             max_total_cost,
					      int                                            &optimal_cost,
					      sMultirobotSolution                            &optimal_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult compute_FuelOptimalSolution(const sRobotArrangement                        &start_arrangement,
					    const sRobotGoal                               &final_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sUndirectedGraph                         &sparse_environment,
					    const sMultirobotInstance::MDD_vector          &MDD,
					    int                                             max_total_fuel,
					    int                                            &optimal_fuel,
					    int                                            &fuel_makespan,
					    sMultirobotSolution                            &optimal_solution,
					    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_FuelOptimalSolution(sSATSolver_Type                               **solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sRobotGoal                               &final_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sUndirectedGraph                         &sparse_environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             max_total_fuel,
					      int                                            &optimal_fuel,
					      int                                            &fuel_makespan,
					      sMultirobotSolution                            &optimal_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult compute_CostOptimalSolution_avoid(const sRobotArrangement                        &start_arrangement,
						  const sRobotGoal                               &final_arrangement,
						  const sUndirectedGraph                         &environment,
						  const sUndirectedGraph                         &sparse_environment,
						  const Arrangements_vector                      &blocked_solution,
						  int                                             optimal_cost,
						  sMultirobotSolution                            &optimal_solution,
						  int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolution_avoid(sSATSolver_Type                               **solver,
						    const sRobotArrangement                        &start_arrangement,
						    const sRobotGoal                               &final_arrangement,
						    const sUndirectedGraph                         &environment,
						    const sUndirectedGraph                         &sparse_environment,
						    const Arrangements_vector                      &blocked_solution,
						    int                                             optimal_cost,
						    sMultirobotSolution                            &optimal_solution,
						    int                                             thread_id = THREAD_ID_UNDEFINED);
	
	sResult compute_MakespanOptimalSolution_avoid(const sRobotArrangement                        &start_arrangement,
						      const sRobotGoal                               &final_arrangement,
						      const sUndirectedGraph                         &environment,
						      const sUndirectedGraph                         &sparse_environment,
						      const Arrangements_vector                      &blocked_solution,
						      int                                             optimal_makespan,
						      sMultirobotSolution                            &optimal_solution,
						      int                                             thread_id = THREAD_ID_UNDEFINED);


	sResult incompute_MakespanOptimalSolution_avoid(sSATSolver_Type                               **solver,
							const sRobotArrangement                        &start_arrangement,
							const sRobotGoal                               &final_arrangement,
							const sUndirectedGraph                         &environment,
							const sUndirectedGraph                         &sparse_environment,
							const Arrangements_vector                      &blocked_solution,
							int                                             optimal_makespan,
							sMultirobotSolution                            &optimal_solution,
							int                                             thread_id = THREAD_ID_UNDEFINED);
	
	sResult compute_CostOptimalSolutionID(const sRobotArrangement                        &start_arrangement,
					      const sRobotGoal                               &final_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sUndirectedGraph                         &sparse_environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             max_total_cost,
					      int                                            &optimal_cost,
					      sMultirobotSolution                            &optimal_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolutionID(sSATSolver_Type                               **solver,
						const sRobotArrangement                        &start_arrangement,
						const sRobotGoal                               &final_arrangement,
						const sUndirectedGraph                         &environment,
						const sUndirectedGraph                         &sparse_environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             max_total_cost,
						int                                            &optimal_cost,
						sMultirobotSolution                            &optimal_solution,
						int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult resolve_GroupDependency(RobotGroups_vector         &robot_Groups,
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
					int                         thread_id = THREAD_ID_UNDEFINED);

	sResult inresolve_GroupDependency(sSATSolver_Type           **solver,
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
					  int                         thread_id = THREAD_ID_UNDEFINED);	

	sResult resolve_MakespanGroupDependency(RobotGroups_vector         &robot_Groups,
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
						int                         thread_id = THREAD_ID_UNDEFINED);

	sResult inresolve_MakespanGroupDependency(sSATSolver_Type           **solver,
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
						int                         thread_id = THREAD_ID_UNDEFINED);		
	
	sResult compute_CostOptimalSolutionAD(const sRobotArrangement                        &start_arrangement,
					      const sRobotGoal                               &final_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sUndirectedGraph                         &sparse_environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             max_total_cost,
					      int                                            &optimal_cost,
					      sMultirobotSolution                            &optimal_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolutionAD(sSATSolver_Type                               **solver,
						const sRobotArrangement                        &start_arrangement,
						const sRobotGoal                               &final_arrangement,
						const sUndirectedGraph                         &environment,
						const sUndirectedGraph                         &sparse_environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             max_total_cost,
						int                                            &optimal_cost,
						sMultirobotSolution                            &optimal_solution,
						int                                             thread_id = THREAD_ID_UNDEFINED);		

	sResult compute_MakespanOptimalSolutionID(const sRobotArrangement &start_arrangement,
						  const sRobotGoal        &final_arrangement,
						  const sUndirectedGraph  &environment,
						  const sUndirectedGraph  &sparse_environment,
						  int                      max_makespan,
						  int                     &optimal_makespan,
						  sMultirobotSolution     &optimal_solution,
						  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_MakespanOptimalSolutionID(sSATSolver_Type        **solver,
						    const sRobotArrangement &start_arrangement,
						    const sRobotGoal        &final_arrangement,
						    const sUndirectedGraph  &environment,
						    const sUndirectedGraph  &sparse_environment,
						    int                      max_makespan,
						    int                     &optimal_makespan,
						    sMultirobotSolution     &optimal_solution,
						    int                      thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_MakespanOptimalSolutionAD(const sRobotArrangement &start_arrangement,
						  const sRobotGoal        &final_arrangement,
						  const sUndirectedGraph  &environment,
						  const sUndirectedGraph  &sparse_environment,
						  int                      max_makespan,
						  int                     &optimal_makespan,
						  sMultirobotSolution     &optimal_solution,
						  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_MakespanOptimalSolutionAD(sSATSolver_Type        **solver,
						    const sRobotArrangement &start_arrangement,
						    const sRobotGoal        &final_arrangement,
						    const sUndirectedGraph  &environment,
						    const sUndirectedGraph  &sparse_environment,
						    int                      max_makespan,
						    int                     &optimal_makespan,
						    sMultirobotSolution     &optimal_solution,
						    int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_CostOptimalSolution_binary(const sRobotArrangement                        &start_arrangement,
						   const sRobotGoal                               &final_arrangement,
						   sUndirectedGraph                               &environment,
						   const sUndirectedGraph                         &sparse_environment,
						   const sMultirobotInstance::MDD_vector          &MDD,
						   int                                             max_total_cost,
						   int                                            &optimal_cost,
						   sMultirobotSolution                            &optimal_solution,
						   int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_CostOptimalSolution_binary(sSATSolver_Type                               **solver,
						     const sRobotArrangement                        &start_arrangement,
						     const sRobotGoal                               &final_arrangement,
						     sUndirectedGraph                               &environment,
						     const sUndirectedGraph                         &sparse_environment,
						     const sMultirobotInstance::MDD_vector          &MDD,
						     int                                             max_total_cost,
						     int                                            &optimal_cost,
						     sMultirobotSolution                            &optimal_solution,
						     int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_BestCostSolution(const sRobotArrangement                        &start_arrangement,
					 const sRobotGoal                               &final_arrangement,
					 sUndirectedGraph                               &environment,
					 const sUndirectedGraph                         &sparse_environment,
					 const sMultirobotInstance::MDD_vector          &MDD,
					 int                                             max_total_cost,
					 int                                            &optimal_cost,
					 sMultirobotSolution                            &optimal_solution,
					 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_BestCostSolution(sSATSolver_Type                               **solver,
					   const sRobotArrangement                        &start_arrangement,
					   const sRobotGoal                               &final_arrangement,
					   sUndirectedGraph                               &environment,
					   const sUndirectedGraph                         &sparse_environment,
					   const sMultirobotInstance::MDD_vector          &MDD,
					   int                                             max_total_cost,
					   int                                            &optimal_cost,
					   sMultirobotSolution                            &optimal_solution,
					   int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_UnirobotMakespan(const sRobotArrangement           &start_arrangement,
					 const sRobotGoal                  &final_arrangement,
					 const sUndirectedGraph            &environment,
					 const sUndirectedGraph            &sparse_environment,
					 int                                layer_upper_bound,
					 int                               &unirobot_makespan,
					 sMultirobotEncodingContext_CNFsat &final_encoding_context,
					 int                                thread_id = THREAD_ID_UNDEFINED);


	sResult incompute_UnirobotMakespan(sSATSolver_Type                  **solver,
					   const sRobotArrangement           &start_arrangement,
					   const sRobotGoal                  &final_arrangement,
					   const sUndirectedGraph            &environment,
					   const sUndirectedGraph            &sparse_environment,
					   int                                layer_upper_bound,
					   int                               &unirobot_makespan,
					   sMultirobotEncodingContext_CNFsat &final_encoding_context,
					   int                                thread_id = THREAD_ID_UNDEFINED);
	
	sResult compute_UnirobotSolution(int                      unirobot_id,
					 const sRobotArrangement &start_arrangement,
					 const sRobotGoal        &final_arrangement,
					 const sUndirectedGraph  &environment,
					 const sUndirectedGraph  &sparse_environment,
					 int                      layer_upper_bound,
					 int                     &unirobot_makespan,
					 sMultirobotSolution     &unirobot_solution,
					 int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_UnirobotSolution(sSATSolver_Type        **solver,
					   int                      unirobot_id,
					   const sRobotArrangement &start_arrangement,
					   const sRobotGoal        &final_arrangement,
					   const sUndirectedGraph  &environment,
					   const sUndirectedGraph  &sparse_environment,
					   int                      layer_upper_bound,
					   int                     &unirobot_makespan,
					   sMultirobotSolution     &unirobot_solution,
					   int                      thread_id = THREAD_ID_UNDEFINED);	

	sResult compute_UnirobotsSolution(const sRobotArrangement &start_arrangement,
					  const sRobotGoal        &final_arrangement,
					  const sUndirectedGraph  &environment,
					  const sUndirectedGraph  &sparse_environment,
					  int                      layer_upper_bound,
					  int                     &unirobot_makespan,
					  sMultirobotSolution     &unirobot_solution,
					  int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_UnirobotsSolution(sSATSolver_Type        **solver,
					    const sRobotArrangement &start_arrangement,
					    const sRobotGoal        &final_arrangement,
					    const sUndirectedGraph  &environment,
					    const sUndirectedGraph  &sparse_environment,
					    int                      layer_upper_bound,
					    int                     &unirobot_makespan,
					    sMultirobotSolution     &unirobot_solution,
					    int                      thread_id = THREAD_ID_UNDEFINED);	

/*----------------------------------------------------------------------------*/

	sResult compute_SuboptimalMakespan(const sRobotArrangement &start_arrangement,
					   const sRobotArrangement &final_arrangement,
					   const sUndirectedGraph  &environment,
					   const sUndirectedGraph  &sparse_environment,
					   int                      makespan_lower_bound,
					   int                      makespan_upper_bound,
					   int                     &suboptimal_makespan,
					   int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SuboptimalMakespan(sSATSolver_Type        **solver,
					     const sRobotArrangement &start_arrangement,
					     const sRobotArrangement &final_arrangement,
					     const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     int                      makespan_lower_bound,
					     int                      makespan_upper_bound,
					     int                     &suboptimal_makespan,
					     int                      thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_SuboptimalMakespan(const sRobotArrangement           &start_arrangement,
					   const sRobotArrangement           &final_arrangement,
					   const sUndirectedGraph            &environment,
					   const sUndirectedGraph            &sparse_environment,
					   int                                makespan_lower_bound,
					   int                                makespan_upper_bound,
					   int                               &suboptimal_makespan,
					   sMultirobotEncodingContext_CNFsat &final_encoding_context,
					   int                                thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SuboptimalMakespan(sSATSolver_Type                  **solver,
					     const sRobotArrangement           &start_arrangement,
					     const sRobotArrangement           &final_arrangement,
					     const sUndirectedGraph            &environment,
					     const sUndirectedGraph            &sparse_environment,
					     int                                makespan_lower_bound,
					     int                                makespan_upper_bound,
					     int                               &suboptimal_makespan,
					     sMultirobotEncodingContext_CNFsat &final_encoding_context,
					     int                                thread_id = THREAD_ID_UNDEFINED);	
	
	sResult compute_SuboptimalSolution(const sRobotArrangement &start_arrangement,
					   const sRobotArrangement &final_arrangement,
					   const sUndirectedGraph  &environment,
					   const sUndirectedGraph  &sparse_environment,
					   int                      makespan_lower_bound,
					   int                      makespan_upper_bound,
					   int                     &suboptimal_makespan,
					   sMultirobotSolution     &suboptimal_solution,
					   int                      thread_id = THREAD_ID_UNDEFINED);

	sResult incompute_SuboptimalSolution(sSATSolver_Type        **solver,
					     const sRobotArrangement &start_arrangement,
					     const sRobotArrangement &final_arrangement,
					     const sUndirectedGraph  &environment,
					     const sUndirectedGraph  &sparse_environment,
					     int                      makespan_lower_bound,
					     int                      makespan_upper_bound,
					     int                     &suboptimal_makespan,
					     sMultirobotSolution     &suboptimal_solution,
					     int                      thread_id = THREAD_ID_UNDEFINED);	


/*----------------------------------------------------------------------------*/

	sResult extract_ComputedInverseSolution(const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						int                                      computed_makespan,
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						sMultirobotSolution                     &computed_solution,
						int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedInverseSolution(sSATSolver_Type                         *solver,
						const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						int                                      computed_makespan,
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,						
						sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedAdvancedSolution(const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						 sMultirobotSolution                     &computed_solution,
						 int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedAdvancedSolution(sSATSolver_Type                         *solver,
						 const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,						 
						 sMultirobotSolution                     &computed_solution);	

	sResult extract_ComputedDifferentialSolution(const sRobotArrangement                 &start_arrangement,
						     const sUndirectedGraph                  &environment,
						     int                                      computed_makespan,
						     const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						     sMultirobotSolution                     &computed_solution,
						     int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedDifferentialSolution(sSATSolver_Type                         *solver,
						     const sRobotArrangement                 &start_arrangement,
						     const sUndirectedGraph                  &environment,
						     int                                      computed_makespan,
						     const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						     sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedBijectionSolution(const sRobotArrangement                 &start_arrangement,
						  const sUndirectedGraph                  &environment,
						  int                                      computed_makespan,
						  const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						  sMultirobotSolution                     &computed_solution,
						  int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedBijectionSolution(sSATSolver_Type                         *solver,
						  const sRobotArrangement                 &start_arrangement,
						  const sUndirectedGraph                  &environment,
						  int                                      computed_makespan,
						  const sMultirobotEncodingContext_CNFsat &final_encoding_context,						  
						  sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeuristicAdvancedSolution(const sRobotArrangement                 &start_arrangement,
							  const sUndirectedGraph                  &environment,
							  int                                      computed_makespan,
							  const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							  sMultirobotSolution                     &computed_solution,
							  int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicAdvancedSolution(sSATSolver_Type                         *solver,
							  const sRobotArrangement                 &start_arrangement,
							  const sUndirectedGraph                  &environment,
							  int                                      computed_makespan,
							  const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							  sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeuristicDifferentialSolution(const sRobotArrangement                 &start_arrangement,
							      const sUndirectedGraph                  &environment,
							      int                                      computed_makespan,
							      const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							      sMultirobotSolution                     &computed_solution,
							      int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicDifferentialSolution(sSATSolver_Type                         *solver,
							      const sRobotArrangement                 &start_arrangement,
							      const sUndirectedGraph                  &environment,
							      int                                      computed_makespan,
							      const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							      sMultirobotSolution                     &computed_solution);
	
	sResult extract_ComputedHeuristicBijectionSolution(const sRobotArrangement                 &start_arrangement,
							   const sUndirectedGraph                  &environment,
							   int                                      computed_makespan,
							   const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							   sMultirobotSolution                     &computed_solution,
							   int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicBijectionSolution(sSATSolver_Type                         *solver,
							   const sRobotArrangement                 &start_arrangement,
							   const sUndirectedGraph                  &environment,
							   int                                      computed_makespan,
							   const sMultirobotEncodingContext_CNFsat &final_encoding_context,							   
							   sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedBitwiseSolution(const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						int                                      computed_makespan,
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						sMultirobotSolution                     &computed_solution,
						int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedBitwiseSolution(sSATSolver_Type                         *solver,
						const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						int                                      computed_makespan,
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedFlowSolution(const sRobotArrangement                 &start_arrangement,
					     const sUndirectedGraph                  &environment,
					     int                                      computed_makespan,
					     const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					     sMultirobotSolution                     &computed_solution,
					     int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedFlowSolution(sSATSolver_Type                         *solver,
					     const sRobotArrangement                 &start_arrangement,
					     const sUndirectedGraph                  &environment,
					     int                                      computed_makespan,
					     const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					     sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedMatchingSolution(const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,						 
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						 sMultirobotSolution                     &computed_solution,
						 int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMatchingSolution(sSATSolver_Type                         *solver,
						 const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						 sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeuristicMatchingSolution(const sRobotArrangement                 &start_arrangement,
							  const sUndirectedGraph                  &environment,
							  int                                      computed_makespan,
							  const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							  sMultirobotSolution                     &computed_solution,
							  int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicMatchingSolution(sSATSolver_Type                         *solver,
							  const sRobotArrangement                 &start_arrangement,
							  const sUndirectedGraph                  &environment,
							  int                                      computed_makespan,
							  const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							  sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedDirectSolution(const sRobotArrangement                 &start_arrangement,
					       const sUndirectedGraph                  &environment,
					       int                                      computed_makespan,
					       const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					       sMultirobotSolution                     &computed_solution,
					       int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedDirectSolution(sSATSolver_Type                         *solver,
					       const sRobotArrangement                 &start_arrangement,
					       const sUndirectedGraph                  &environment,
					       int                                      computed_makespan,					       
					       const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					       sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeuristicDirectSolution(const sRobotArrangement                 &start_arrangement,
							const sUndirectedGraph                  &environment,
							int                                      computed_makespan,
							const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							sMultirobotSolution                     &computed_solution,
							int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicDirectSolution(sSATSolver_Type                         *solver,
							const sRobotArrangement                 &start_arrangement,
							const sUndirectedGraph                  &environment,
							int                                      computed_makespan,							
							const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedSimplicialSolution(const sRobotArrangement                 &start_arrangement,
						   const sUndirectedGraph                  &environment,
						   int                                      computed_makespan,						   
						   const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						   sMultirobotSolution                     &computed_solution,
						   int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedSimplicialSolution(sSATSolver_Type                         *solver,
						   const sRobotArrangement                 &start_arrangement,
						   const sUndirectedGraph                  &environment,
						   int                                      computed_makespan,						   
						   const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						   sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeuristicSimplicialSolution(const sRobotArrangement                 &start_arrangement,
							    const sUndirectedGraph                  &environment,
							    int                                      computed_makespan,							    
							    const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							    sMultirobotSolution                     &computed_solution,
							    int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeuristicSimplicialSolution(sSATSolver_Type                         *solver,
							    const sRobotArrangement                 &start_arrangement,
							    const sUndirectedGraph                  &environment,
							    int                                      computed_makespan,							    
							    const sMultirobotEncodingContext_CNFsat &final_encoding_context,
							    sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedSingularSolution(const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						 sMultirobotSolution                     &computed_solution,
						 int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedSingularSolution(sSATSolver_Type                         *solver,
						 const sRobotArrangement                 &start_arrangement,
						 const sUndirectedGraph                  &environment,
						 int                                      computed_makespan,
						 const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						 sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedPluralSolution(const sRobotArrangement                 &start_arrangement,
					       const sUndirectedGraph                  &environment,
					       int                                      computed_makespan,
					       const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					       sMultirobotSolution                     &computed_solution,
					       int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedPluralSolution(sSATSolver_Type                         *solver,
					       const sRobotArrangement                 &start_arrangement,
					       const sUndirectedGraph                  &environment,
					       int                                      computed_makespan,
					       const sMultirobotEncodingContext_CNFsat &final_encoding_context,
					       sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedPlural2Solution(const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						const sUndirectedGraph                  &sparse_environment,
						int                                      computed_makespan,
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						sMultirobotSolution                     &computed_solution,
						int                                      thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedPlural2Solution(sSATSolver_Type                         *solver,
						const sRobotArrangement                 &start_arrangement,
						const sUndirectedGraph                  &environment,
						const sUndirectedGraph                  &sparse_environment,
						int                                      computed_makespan,						
						const sMultirobotEncodingContext_CNFsat &final_encoding_context,
						sMultirobotSolution                     &computed_solution);

	sResult extract_ComputedHeightedSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::Environments_vector &heighted_Environments,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedHeightedSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 int                                             computed_cost,
						 const sMultirobotInstance::Environments_vector &heighted_Environments,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMddSolution(const sRobotArrangement                        &start_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sMultirobotInstance::MDD_vector          &MDD,
					    int                                             computed_cost,
					    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					    sMultirobotSolution                            &computed_solution,
					    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMddSolution(sSATSolver_Type                                *solver,
					    const sRobotArrangement                        &start_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sMultirobotInstance::MDD_vector          &MDD,
					    int                                             computed_cost,
					    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					    sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMddUmtexSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMddUmtexSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMddMutexSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMddMutexSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);		

	sResult extract_ComputedGMddSolution(const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_cost,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution,
					     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedGMddSolution(sSATSolver_Type                                *solver,
					     const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_cost,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedGEMddSolution(const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedGEMddSolution(sSATSolver_Type                                *solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedAnoSolution(const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_cost,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution,
					     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedAnoSolution(sSATSolver_Type                                *solver,
					    const sRobotArrangement                        &start_arrangement,
					    const sUndirectedGraph                         &environment,
					    const sMultirobotInstance::MDD_vector          &MDD,
					    int                                             computed_cost,
					    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					    sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedGAnoSolution(const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_cost,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution,
					     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedGAnoSolution(sSATSolver_Type                                *solver,
					     const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_cost,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution);	

	sResult extract_ComputedWaterMddSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedWaterMddSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedRelaxedMddSolution(const sRobotArrangement                        &start_arrangement,
						   const sUndirectedGraph                         &environment,
						   const sMultirobotInstance::MDD_vector          &MDD,
						   int                                             computed_cost,
						   const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						   sMultirobotSolution                            &computed_solution,
						   int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedTokenMddSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedTokenEmptyMddSolution(const sRobotArrangement                        &start_arrangement,
						      const sUndirectedGraph                         &environment,
						      const sMultirobotInstance::MDD_vector          &MDD,
						      int                                             computed_cost,
						      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						      sMultirobotSolution                            &computed_solution,
						      int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult extract_ComputedPermutationMddSolution(const sRobotArrangement                        &start_arrangement,
						       const sUndirectedGraph                         &environment,
						       const sMultirobotInstance::MDD_vector          &MDD,
						       int                                             computed_cost,
						       const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						       sMultirobotSolution                            &computed_solution,
						       int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedCapacitatedPermutationMddSolution(const sRobotArrangement                        &start_arrangement,
								  const sUndirectedGraph                         &environment,
								  const sMultirobotInstance::MDD_vector          &MDD,
								  int                                             computed_cost,
								  const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
								  sMultirobotSolution                            &computed_solution,
								  int                                             thread_id = THREAD_ID_UNDEFINED);			

	sResult intract_ComputedRelaxedMddSolution(sSATSolver_Type                                *solver,
						   const sRobotArrangement                        &start_arrangement,
						   const sUndirectedGraph                         &environment,
						   const sMultirobotInstance::MDD_vector          &MDD,
						   int                                             computed_cost,
						   const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						   sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedTokenMddSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_cost,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedTokenEmptyMddSolution(sSATSolver_Type                                *solver,
						      const sRobotArrangement                        &start_arrangement,
						      const sUndirectedGraph                         &environment,
						      const sMultirobotInstance::MDD_vector          &MDD,
						      int                                             computed_cost,
						      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						      sMultirobotSolution                            &computed_solution);	

	sResult intract_ComputedPermutationMddSolution(sSATSolver_Type                                *solver,
						       const sRobotArrangement                        &start_arrangement,
						       const sUndirectedGraph                         &environment,
						       const sMultirobotInstance::MDD_vector          &MDD,
						       int                                             computed_cost,
						       const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						       sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedCapacitatedPermutationMddSolution(sSATSolver_Type                                *solver,
								  const sRobotArrangement                        &start_arrangement,
								  const sUndirectedGraph                         &environment,
								  const sMultirobotInstance::MDD_vector          &MDD,
								  int                                             computed_cost,
								  const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
								  sMultirobotSolution                            &computed_solution);			

	sResult extract_ComputedMmddSolution(const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_makespan,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution,
					     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMmddSolution(sSATSolver_Type                                *solver,
					     const sRobotArrangement                        &start_arrangement,
					     const sUndirectedGraph                         &environment,
					     const sMultirobotInstance::MDD_vector          &MDD,
					     int                                             computed_makespan,
					     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					     sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedRelaxedMmddSolution(const sRobotArrangement                        &start_arrangement,
						    const sUndirectedGraph                         &environment,
						    const sMultirobotInstance::MDD_vector          &MDD,
						    int                                             computed_makespan,
						    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						    sMultirobotSolution                            &computed_solution,
						    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedTokenMmddSolution(const sRobotArrangement                        &start_arrangement,
						  const sUndirectedGraph                         &environment,
						  const sMultirobotInstance::MDD_vector          &MDD,
						  int                                             computed_makespan,
						  const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						  sMultirobotSolution                            &computed_solution,
						  int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedTokenEmptyMmddSolution(const sRobotArrangement                        &start_arrangement,
						       const sUndirectedGraph                         &environment,
						       const sMultirobotInstance::MDD_vector          &MDD,
						       int                                             computed_makespan,
						       const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						       sMultirobotSolution                            &computed_solution,
						       int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult extract_ComputedPermutationMmddSolution(const sRobotArrangement                        &start_arrangement,
							const sUndirectedGraph                         &environment,
							const sMultirobotInstance::MDD_vector          &MDD,
							int                                             computed_makespan,
							const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							sMultirobotSolution                            &computed_solution,
							int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedCapacitatedPermutationMmddSolution(const sRobotArrangement                        &start_arrangement,
								   const sUndirectedGraph                         &environment,
								   const sMultirobotInstance::MDD_vector          &MDD,
								   int                                             computed_makespan,
								   const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
								   sMultirobotSolution                            &computed_solution,
								   int                                             thread_id = THREAD_ID_UNDEFINED);			

	sResult intract_ComputedRelaxedMmddSolution(sSATSolver_Type                                *solver,
						    const sRobotArrangement                        &start_arrangement,
						    const sUndirectedGraph                         &environment,
						    const sMultirobotInstance::MDD_vector          &MDD,
						    int                                             computed_makespan,
						    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						    sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedTokenMmddSolution(sSATSolver_Type                                *solver,
						  const sRobotArrangement                        &start_arrangement,
						  const sUndirectedGraph                         &environment,
						  const sMultirobotInstance::MDD_vector          &MDD,
						  int                                             computed_makespan,
						  const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						  sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedTokenEmptyMmddSolution(sSATSolver_Type                                *solver,
						       const sRobotArrangement                        &start_arrangement,
						       const sUndirectedGraph                         &environment,
						       const sMultirobotInstance::MDD_vector          &MDD,
						       int                                             computed_makespan,
						       const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						       sMultirobotSolution                            &computed_solution);	

	sResult intract_ComputedPermutationMmddSolution(sSATSolver_Type                                *solver,
							const sRobotArrangement                        &start_arrangement,
							const sUndirectedGraph                         &environment,
							const sMultirobotInstance::MDD_vector          &MDD,
							int                                             computed_makespan,
							const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedCapacitatedPermutationMmddSolution(sSATSolver_Type                                *solver,
								   const sRobotArrangement                        &start_arrangement,
								   const sUndirectedGraph                         &environment,
								   const sMultirobotInstance::MDD_vector          &MDD,
								   int                                             computed_makespan,
								   const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
								   sMultirobotSolution                            &computed_solution);			

	sResult extract_ComputedMddPlusSolution(const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution,
						int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMddPlusSolution(sSATSolver_Type                                *solver,
						const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMddPlusPlusSolution(const sRobotArrangement                        &start_arrangement,
						    const sUndirectedGraph                         &environment,
						    const sMultirobotInstance::MDD_vector          &MDD,
						    int                                             computed_cost,
						    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						    sMultirobotSolution                            &computed_solution,
						    int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult extract_ComputedMddPlusPlusMutexSolution(const sRobotArrangement                        &start_arrangement,
							 const sUndirectedGraph                         &environment,
							 const sMultirobotInstance::MDD_vector          &MDD,
							 int                                             computed_cost,
							 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							 sMultirobotSolution                            &computed_solution,
							 int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult extract_ComputedMddPlusPlusFuelSolution(const sRobotArrangement                        &start_arrangement,
							const sUndirectedGraph                         &environment,
							const sMultirobotInstance::MDD_vector          &MDD,
							int                                             computed_cost,
							int                                             fuel_makespan,
							const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							sMultirobotSolution                            &computed_solution,
							int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult intract_ComputedMddPlusPlusSolution(sSATSolver_Type                                *solver,
						    const sRobotArrangement                        &start_arrangement,
						    const sUndirectedGraph                         &environment,
						    const sMultirobotInstance::MDD_vector          &MDD,
						    int                                             computed_cost,
						    const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						    sMultirobotSolution                            &computed_solution);

	sResult intract_ComputedMddPlusPlusMutexSolution(sSATSolver_Type                                *solver,
							 const sRobotArrangement                        &start_arrangement,
							 const sUndirectedGraph                         &environment,
							 const sMultirobotInstance::MDD_vector          &MDD,
							 int                                             computed_cost,
							 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							 sMultirobotSolution                            &computed_solution);	

	sResult intract_ComputedMddPlusPlusFuelSolution(sSATSolver_Type                                *solver,
							const sRobotArrangement                        &start_arrangement,
							const sUndirectedGraph                         &environment,
							const sMultirobotInstance::MDD_vector          &MDD,
							int                                             computed_cost,
							int                                             fuel_makespan,
							const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
							sMultirobotSolution                            &computed_solution);	

	sResult extract_ComputedLMddPlusPlusSolution(const sRobotArrangement                        &start_arrangement,
						     const sUndirectedGraph                         &environment,
						     const sMultirobotInstance::MDD_vector          &MDD,
						     int                                             computed_cost,
						     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						     sMultirobotSolution                            &computed_solution,
						     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedLMddPlusPlusSolution(sSATSolver_Type                                *solver,
						     const sRobotArrangement                        &start_arrangement,
						     const sUndirectedGraph                         &environment,
						     const sMultirobotInstance::MDD_vector          &MDD,
						     int                                             computed_cost,
						     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						     sMultirobotSolution                            &computed_solution);	

	sResult extract_ComputedMddStarSolution(const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution,
						int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMddStarSolution(sSATSolver_Type                                *solver,
						const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMmddPlusSolution(const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_makespan,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution,
						 int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMmddPlusSolution(sSATSolver_Type                                *solver,
						 const sRobotArrangement                        &start_arrangement,
						 const sUndirectedGraph                         &environment,
						 const sMultirobotInstance::MDD_vector          &MDD,
						 int                                             computed_makespan,
						 const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						 sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedMmddPlusPlusSolution(const sRobotArrangement                        &start_arrangement,
						     const sUndirectedGraph                         &environment,
						     const sMultirobotInstance::MDD_vector          &MDD,
						     int                                             computed_makespan,
						     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						     sMultirobotSolution                            &computed_solution,
						     int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedMmddPlusPlusSolution(sSATSolver_Type                                *solver,
						     const sRobotArrangement                        &start_arrangement,
						     const sUndirectedGraph                         &environment,
						     const sMultirobotInstance::MDD_vector          &MDD,
						     int                                             computed_makespan,
						     const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						     sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedRXMddSolution(const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedRXMddSolution(sSATSolver_Type                                *solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution);

	sResult extract_ComputedNoMddSolution(const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution,
					      int                                             thread_id = THREAD_ID_UNDEFINED);	

	sResult intract_ComputedNoMddSolution(sSATSolver_Type                                *solver,
					      const sRobotArrangement                        &start_arrangement,
					      const sUndirectedGraph                         &environment,
					      const sMultirobotInstance::MDD_vector          &MDD,
					      int                                             computed_cost,
					      const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
					      sMultirobotSolution                            &computed_solution);
	
	sResult extract_ComputedRXNoMddSolution(const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution,
						int                                             thread_id = THREAD_ID_UNDEFINED);

	sResult intract_ComputedRXNoMddSolution(sSATSolver_Type                                *solver,
						const sRobotArrangement                        &start_arrangement,
						const sUndirectedGraph                         &environment,
						const sMultirobotInstance::MDD_vector          &MDD,
						int                                             computed_cost,
						const sMultirobotEncodingContext_CNFsat        &final_encoding_context,
						sMultirobotSolution                            &computed_solution);

	sResult shorten_Solution(const sRobotArrangement   &initial_arrangement,
				 const sMultirobotSolution &original_solution,
				 sUndirectedGraph          &environment,
				 const sUndirectedGraph    &sparse_environment,
				 sMultirobotSolution       &shortened_solution,
				 int                        thread_id = THREAD_ID_UNDEFINED);

	sResult shorten_Solution_mt(const sRobotArrangement   &initial_arrangement,
				    const sMultirobotSolution &original_solution,
				    sUndirectedGraph          &environment,
				    const sUndirectedGraph    &sparse_environment,
				    sMultirobotSolution       &shortened_solution);

	sResult primeShorten_Solution(const sRobotArrangement   &initial_arrangement,
				      const sMultirobotSolution &original_solution,
				      sUndirectedGraph          &environment,
				      const sUndirectedGraph    &sparse_environment,
				      sMultirobotSolution       &shortened_solution,
				      int                        thread_id = THREAD_ID_UNDEFINED);

	sResult primeShorten_Solution_mt(const sRobotArrangement   &initial_arrangement,
					 const sMultirobotSolution &original_solution,
					 sUndirectedGraph          &environment,
					 const sUndirectedGraph    &sparse_environment,
					 sMultirobotSolution       &shortened_solution);

	sResult primeShorten_Solution_mt2(const sRobotArrangement   &initial_arrangement,
					  const sMultirobotSolution &original_solution,
					  sUndirectedGraph          &environment,
					  const sUndirectedGraph    &sparse_environment,
					  sMultirobotSolution       &shortened_solution);

	sResult optimize_Solution(const sRobotArrangement   &initial_arrangement,
				  const sMultirobotSolution &original_solution,
				  sUndirectedGraph          &environment,
				  const sUndirectedGraph    &sparse_environment,
				  sMultirobotSolution       &optimized_solution);

	sResult optimize_Solution_mt(const sRobotArrangement   &initial_arrangement,
				     const sMultirobotSolution &original_solution,
				     sUndirectedGraph          &environment,
				     const sUndirectedGraph    &sparse_environment,
				     sMultirobotSolution       &optimized_solution);

	sResult primeOptimize_Solution(const sRobotArrangement   &initial_arrangement,
				       const sMultirobotSolution &original_solution,
				       sUndirectedGraph          &environment,
				       const sUndirectedGraph    &sparse_environment,
				       sMultirobotSolution       &optimized_solution);

	sResult primeOptimize_Solution_mt(const sRobotArrangement   &initial_arrangement,
					  const sMultirobotSolution &original_solution,
					  sUndirectedGraph          &environment,
					  const sUndirectedGraph    &sparse_environment,
					  sMultirobotSolution       &optimized_solution);

	sResult compress_Solution(const sRobotArrangement   &initial_arrangement,
				  const sMultirobotSolution &original_solution,
				  sUndirectedGraph          &environment,
				  const sUndirectedGraph    &sparse_environment,
				  sMultirobotSolution       &compressed_solution);

	sResult deflate_Solution(const sRobotArrangement   &initial_arrangement,
				 const sMultirobotSolution &original_solution,
				 sUndirectedGraph          &environment,
				 const sUndirectedGraph    &sparse_environment,
				 sMultirobotSolution       &deflated_solution);

	sResult deflate_Solution_mt(const sRobotArrangement   &initial_arrangement,
				    const sMultirobotSolution &original_solution,
				    sUndirectedGraph          &environment,
				    const sUndirectedGraph    &sparse_environment,
				    sMultirobotSolution       &deflated_solution);

	static void unfold_Solution(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, Arrangements_vector &unfolded_Arrangements);
	static void force_Solution(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, Arrangements_vector &unfolded_Arrangements);	
	static bool verify_Unfolding(const sRobotArrangement &initial_arrangement, const sMultirobotSolution &solution, const sUndirectedGraph &graph);

	void to_Screen_arrangements(const Arrangements_vector &unfolded_Arrangements, const sString &indent = "") const;
	void to_Stream_arrangements(FILE *fw, const Arrangements_vector &unfolded_Arrangements, const sString &indent = "") const;

    private:
	int m_minisat_timeout;
	int m_total_timeout;
	int m_makespan_upper_bound;
	sString m_minisat_path;

	int m_N_parallel_Threads;
	Encoding m_encoding;

	AttemptDatabaseRecords_set m_attempt_Database;
	double m_ratio;
	int m_robustness;
	int m_range;
    };


/*----------------------------------------------------------------------------*/
// Global functions

    void* s_shorten_Solution_mt(void *arg);
    void* s_prime_shorten_Solution_mt(void *arg);


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __COMPRESS_H__ */
