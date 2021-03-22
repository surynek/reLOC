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
/* encodings_binary.cpp / 0.22-robik_075                                      */
/*----------------------------------------------------------------------------*/
//
// Multi-robot path-finding encodings based on
// binary state variable representation.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <map>

#include "config.h"
#include "compile.h"
#include "version.h"
#include "defs.h"
#include "types.h"
#include "result.h"
#include "cnf.h"
#include "multirobot.h"
#include "compress.h"
#include "statistics.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/
// sMultirobotInstance encodings

    void sMultirobotInstance::to_Screen_InverseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_InverseCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_AdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_AdvancedCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_DifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_DifferentialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_BijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_BijectionCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicDifferentialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicDifferentialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicBijectionCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicBijectionCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicAdvancedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicAdvancedCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_PuzzleCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_PuzzleCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_BitwiseCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_BitwiseCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_FlowCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_FlowCNFsat(stdout, encoding_context, indent, verbose);
    }

    sResult sMultirobotInstance::to_File_InverseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_InverseCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_AdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_AdvancedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_DifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_DifferentialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_BijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_BijectionCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicDifferentialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicDifferentialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicBijectionCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicBijectionCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicAdvancedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicAdvancedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_PuzzleCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PuzzleCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_BitwiseCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_BitwiseCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_FlowCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_FlowCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    
    void sMultirobotInstance::to_Stream_InverseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;
 
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 N_neighbor_Neighbors + in_neighbor_order,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 N_vertex_Neighbors + out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											     2 * N_vertex_Neighbors,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot inverse SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)), verbose);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      N_neighbor_Neighbors + in_neighbor_order,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      verbose);
		    
		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      N_vertex_Neighbors + out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      in_neighbor_order,
										      verbose);
		    
		    ++out_neighbor_order;
		}
		encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										  sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										  2 * N_vertex_Neighbors,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  verbose);
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX, verbose);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Memory_InverseCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->cast_Alignment(solver,
										   sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 N_neighbor_Neighbors + in_neighbor_order,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 N_vertex_Neighbors + out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
											     sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											     2 * N_vertex_Neighbors,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									      
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->cast_Equality(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->cast_DisjunctiveEquality(solver,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    encoding_context.m_clause_generator->cast_Equality(solver,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


    void sMultirobotInstance::to_Stream_AdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_InverseCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_AdvancedCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Memory_InverseCNFsat(solver, encoding_context, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_DifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }		
	    Clause_cnt += encoding_context.m_clause_generator->count_AllDifferenceConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     robot_diff_Identifiers);
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
	 	    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_clause_generator->count_DifferenceConstraint(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											      prev_robot_diff_Identifiers);

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
		    sStateClauseGenerator::States_vector robot_case_split_States;

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
		    robot_case_split_States.push_back(vertex_id);

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
		    robot_case_split_States.push_back(vertex_id);

		    const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(neighbor_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_CaseSplitting(aux_Variable_cnt,
											   total_Literal_cnt,
											   robot_case_split_Identifiers,
											   robot_case_split_States);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									      vertex_id);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
	
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot differential SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		fprintf(fw, "c label alignment layer=%d robot=%d\n", layer, robot_id);
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), verbose);
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	    fprintf(fw, "c label alldifferent layer=%d\n", layer);
	    encoding_context.m_clause_generator->generate_AllDifferenceConstraint(fw, robot_diff_Identifiers, verbose);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		fprintf(fw, "c label different layer=%d robot=%d\n", layer, robot_id);
		encoding_context.m_clause_generator->generate_DifferenceConstraint(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)), prev_robot_diff_Identifiers, verbose);

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
		    sStateClauseGenerator::States_vector robot_case_split_States;

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
		    robot_case_split_States.push_back(vertex_id);

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
		    robot_case_split_States.push_back(vertex_id);

		    const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
 
		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(neighbor_id);
		    }
		    fprintf(fw, "c label case_split layer=%d robot=%d vertex=%d\n", layer, robot_id, vertex_id);
		    encoding_context.m_clause_generator->generate_CaseSplitting(fw, robot_case_split_Identifiers, robot_case_split_States, verbose);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);

	    fprintf(fw, "c label initial robot=%d\n", robot_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)), vertex_id, verbose);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		fprintf(fw, "c label goal robot=%d\n", robot_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), vertex_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    fprintf(fw, "c label goal robot=%d\n", robot_id);
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Memory_DifferentialCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }		
	    encoding_context.m_clause_generator->cast_AllDifferenceConstraint(solver,
											    robot_diff_Identifiers);
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
	 	    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		encoding_context.m_clause_generator->cast_DifferenceConstraint(solver,
											     sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											     prev_robot_diff_Identifiers);

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
		    sStateClauseGenerator::States_vector robot_case_split_States;

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
		    robot_case_split_States.push_back(vertex_id);

		    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
		    robot_case_split_States.push_back(vertex_id);

		    const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
		    {
			int neighbor_id = (*neighbor)->m_target->m_id;

			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(neighbor_id);
		    }
		    encoding_context.m_clause_generator->cast_CaseSplitting(solver,
											  robot_case_split_Identifiers,
											  robot_case_split_States);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									     vertex_id);
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										 vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    encoding_context.m_clause_generator->cast_DisjunctiveEquality(solver,
												goal_case_disj_Identifiers,
												goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


    void sMultirobotInstance::to_Stream_BijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_DifferentialCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_BijectionCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Memory_DifferentialCNFsat(solver, encoding_context, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_HeuristicDifferentialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose)
    {
	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
        {
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	    Clause_cnt += encoding_context.m_clause_generator->count_AllDifferenceConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     robot_diff_Identifiers);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_clause_generator->count_DifferenceConstraint(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											      prev_robot_diff_Identifiers);

		sStateClauseGenerator::States_vector vertex_IDs;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			vertex_IDs.push_back(vertex_id);
		    }
		    else
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
			sStateClauseGenerator::States_vector robot_case_split_States;
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
			robot_case_split_States.push_back(vertex_id);
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(vertex_id);
			
			const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    int neighbor_id = (*neighbor)->m_target->m_id;
			    
			    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			    robot_case_split_States.push_back(neighbor_id);
			}
			Clause_cnt += encoding_context.m_clause_generator->count_CaseSplitting(aux_Variable_cnt,
											       total_Literal_cnt,
											       robot_case_split_Identifiers,
											       robot_case_split_States);

#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveDisequality(aux_Variable_cnt,
															total_Literal_cnt, 
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_id),
																		  sIntegerIndex(layer)),
															vertex_id,
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_2_id),
																		  sIntegerIndex(layer)),
															vertex_2_id);
				    }
				}
			    }
			}
#endif
		    }
		    //			Clause_cnt += encoding_context.m_clause_generator->count_Disequality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), vertex_id);
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)),
										       vertex_IDs);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									      vertex_id);
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}


	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot differential SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), verbose);
	    }
	}
 
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }		
	    encoding_context.m_clause_generator->generate_AllDifferenceConstraint(fw, robot_diff_Identifiers, verbose);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		encoding_context.m_clause_generator->generate_DifferenceConstraint(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)), prev_robot_diff_Identifiers, verbose);

		sStateClauseGenerator::States_vector vertex_IDs;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			vertex_IDs.push_back(vertex_id);
		    }
		    else
		    {
		      sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
			sStateClauseGenerator::States_vector robot_case_split_States;
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
			robot_case_split_States.push_back(vertex_id);
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(vertex_id);
			
			const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    int neighbor_id = (*neighbor)->m_target->m_id;
			    
			    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			    robot_case_split_States.push_back(neighbor_id);
			}
			encoding_context.m_clause_generator->generate_CaseSplitting(fw, robot_case_split_Identifiers, robot_case_split_States, verbose);

#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->generate_DisjunctiveDisequality(fw,
													     sSpecifiedStateIdentifier(&robot_location,
																       sIntegerIndex(robot_id),
																       sIntegerIndex(layer)),
													     vertex_id,
													     sSpecifiedStateIdentifier(&robot_location,
																       sIntegerIndex(robot_2_id),
																       sIntegerIndex(layer)),
													     vertex_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), vertex_IDs, verbose);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)), vertex_id, verbose);
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), vertex_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Memory_HeuristicDifferentialCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier robot_location(&encoding_context.m_variable_store, "robot_location", N_Vertices, sIntegerScope(1, N_Robots), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_robot_location = robot_location;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
        {
	    sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_diff_Identifiers;
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
	    }
	    encoding_context.m_clause_generator->cast_AllDifferenceConstraint(solver,
											    robot_diff_Identifiers);
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector prev_robot_diff_Identifiers;
		int prev_robot_id;
		for (prev_robot_id = 1; prev_robot_id < robot_id; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		for (++prev_robot_id; prev_robot_id <= N_Robots; ++prev_robot_id)
		{
		    prev_robot_diff_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(prev_robot_id), sIntegerIndex(layer)));
		}
		encoding_context.m_clause_generator->cast_DifferenceConstraint(solver,
											     sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)),
											     prev_robot_diff_Identifiers);

		sStateClauseGenerator::States_vector vertex_IDs;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			vertex_IDs.push_back(vertex_id);
		    }
		    else
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector robot_case_split_Identifiers;
			sStateClauseGenerator::States_vector robot_case_split_States;
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)));
			robot_case_split_States.push_back(vertex_id);
			
			robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			robot_case_split_States.push_back(vertex_id);
			
			const sVertex::Neighbors_list &Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			{
			    int neighbor_id = (*neighbor)->m_target->m_id;
			    
			    robot_case_split_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer + 1)));
			    robot_case_split_States.push_back(neighbor_id);
			}
			encoding_context.m_clause_generator->cast_CaseSplitting(solver,
											      robot_case_split_Identifiers,
											      robot_case_split_States);

#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->cast_DisjunctiveDisequality(solver,
															total_Literal_cnt, 
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_id),
																		  sIntegerIndex(layer)),
															vertex_id,
															sSpecifiedStateIdentifier(&robot_location,
																		  sIntegerIndex(robot_2_id),
																		  sIntegerIndex(layer)),
															vertex_2_id);
				    }
				}
			    }
			}
#endif
		    }
		    //			encoding_context.m_clause_generator->cast_Disequality(solver, total_Literal_cnt, sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)), vertex_id);
		}
		encoding_context.m_clause_generator->cast_Disequalities(solver,
										      sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(layer)),
										      vertex_IDs);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    int vertex_id = m_initial_arrangement.get_RobotLocation(robot_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(0)),
									     vertex_id);
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		int vertex_id = m_goal_arrangement.get_RobotLocation(robot_id);
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										 vertex_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		const sRobotGoal::Vertices_set &vertex_IDs = m_goal_specification.get_RobotGoal(robot_id);
		
		if (!vertex_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;

		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Vertices_set::const_iterator vertex_id = vertex_IDs.begin(); vertex_id != vertex_IDs.end(); ++vertex_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&robot_location, sIntegerIndex(robot_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*vertex_id);
		    }
		    encoding_context.m_clause_generator->cast_DisjunctiveEquality(solver,
												goal_case_disj_Identifiers,
												goal_case_disj_States);
		}
		else
		{
		    sASSERT(false);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


    void sMultirobotInstance::to_Stream_HeuristicBijectionCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Stream_HeuristicDifferentialCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_HeuristicBijectionCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_AdvancedGeneratingMode();
	to_Memory_HeuristicDifferentialCNFsat(solver, encoding_context, indent, verbose);
    }
    

    void sMultirobotInstance::to_Stream_HeuristicAdvancedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose)
    {
	if (encoding_context.get_GeneratingMode() != sMultirobotEncodingContext_CNFsat::GENERATING_BITWISE)
	{
	    encoding_context.switchTo_AdvancedGeneratingMode();
	}
	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;
 
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		    else
		    {
#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveDisequality(aux_Variable_cnt,
															total_Literal_cnt,
															sSpecifiedStateIdentifier(&vertex_occupancy,
																		  sIntegerIndex(vertex_id),
																		  sIntegerIndex(layer)),
															robot_id,
															sSpecifiedStateIdentifier(&vertex_occupancy,
																		  sIntegerIndex(vertex_2_id),
																		  sIntegerIndex(layer)),
															robot_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       robot_IDs);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 N_neighbor_Neighbors + in_neighbor_order,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												 0,
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												 N_vertex_Neighbors + out_neighbor_order,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												 in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		Clause_cnt += encoding_context.m_clause_generator->count_ConditionalEquality(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											     2 * N_vertex_Neighbors,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt, total_Literal_cnt, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;
		    
		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    Clause_cnt += encoding_context.m_clause_generator->count_DisjunctiveEquality(aux_Variable_cnt,
												 total_Literal_cnt,
												 goal_case_disj_Identifiers,
												 goal_case_disj_States);
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot advanced SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)), verbose);
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			sASSERT(!goal_IDs.empty());

			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		    else
		    {
#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->generate_DisjunctiveDisequality(fw,
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     robot_id,
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_2_id),
																       sIntegerIndex(layer)),
													     robot_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), robot_IDs);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      N_neighbor_Neighbors + in_neighbor_order,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										      0,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      verbose);
		    
		    encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										      N_vertex_Neighbors + out_neighbor_order,
										      sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
										      in_neighbor_order,
										      verbose);
		    
		    ++out_neighbor_order;
		}
		encoding_context.m_clause_generator->generate_ConditionalEquality(fw,
										  sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
										  2 * N_vertex_Neighbors,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  verbose);
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	} 
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;
		    
		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States, verbose);
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX, verbose);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Memory_HeuristicAdvancedCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	if (encoding_context.get_GeneratingMode() != sMultirobotEncodingContext_CNFsat::GENERATING_BITWISE)
	{
	    encoding_context.switchTo_AdvancedGeneratingMode();
	}
	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

#ifdef MUTEX
	m_environment.calc_AllPairsShortestCoopPaths();
	const sUndirectedGraph::Distances_4d_vector &all_pairs_coop_Distances = m_environment.get_AllPairsShortestCoopPaths();

	s_GlobalPhaseStatistics.enter_Phase("SAT");
#endif

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_transition_Actions.resize(N_Vertices);
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action-" + sInt_32_to_String(vertex_id), 2 * m_environment.get_Vertex(vertex_id)->calc_NeighborCount() + 1, sIntegerScope(0, encoding_context.m_N_Layers - 1)); 
	    encoding_context.m_transition_Actions[vertex_id] = transition_action;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_transition_Actions[vertex_id]);

	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
		sStateClauseGenerator::States_vector robot_IDs;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    bool forbid_vertex = true;

		    switch (m_goal_type)
		    {
		    case GOAL_TYPE_ARRANGEMENT:
		    {
			if (all_pairs_Distances[vertex_id][m_goal_arrangement.get_RobotLocation(robot_id)] <= encoding_context.m_N_Layers - layer)
			{
			    forbid_vertex = false;
			}
			break;
		    }
		    case GOAL_TYPE_SPECIFICATION:
		    {
			const sRobotGoal::Vertices_set &goal_IDs = m_goal_specification.get_RobotGoal(robot_id);
			for (sRobotGoal::Vertices_set::const_iterator goal_id = goal_IDs.begin(); goal_id != goal_IDs.end(); ++goal_id)
			{
			    if (all_pairs_Distances[vertex_id][*goal_id] <= encoding_context.m_N_Layers - layer)
			    {
				forbid_vertex = false;
				break;
			    }
			}
			break;
		    }
		    default:
		    {
			sASSERT(false);
			break;
		    }
		    }
		    if (forbid_vertex || all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] > layer)
		    {
			robot_IDs.push_back(robot_id);
		    }
		    else
		    {
#ifdef MUTEX
			for (int vertex_2_id = vertex_id + 1; vertex_2_id < N_Vertices; ++vertex_2_id)
			{
			    for (int robot_2_id = 1; robot_2_id <= N_Robots; ++robot_2_id)
			    {
				if (all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_2_id] <= layer && all_pairs_Distances[vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_2_id)] <= encoding_context.m_N_Layers - layer)
				{

				    if (   all_pairs_coop_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][m_initial_arrangement.get_RobotLocation(robot_2_id)][vertex_id][vertex_2_id] > layer 
					|| all_pairs_coop_Distances[vertex_id][vertex_2_id][m_goal_arrangement.get_RobotLocation(robot_id)][m_goal_arrangement.get_RobotLocation(robot_2_id)] > encoding_context.m_N_Layers - layer)
				    {
					encoding_context.m_clause_generator->cast_DisjunctiveDisequality(solver,
														       sSpecifiedStateIdentifier(&vertex_occupancy,
																		 sIntegerIndex(vertex_id),
																		 sIntegerIndex(layer)),
														       robot_id,
														       sSpecifiedStateIdentifier(&vertex_occupancy,
																		 sIntegerIndex(vertex_2_id),
																		 sIntegerIndex(layer)),
														       robot_2_id);
				    }
				}
			    }
			}
#endif
		    }
		}
		encoding_context.m_clause_generator->cast_Disequalities(solver,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      robot_IDs);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int N_vertex_Neighbors = m_environment.get_Vertex(vertex_id)->calc_NeighborCount();
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		int out_neighbor_order = 0;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int N_neighbor_Neighbors = m_environment.get_Vertex(neighbor_id)->calc_NeighborCount();
		    int in_neighbor_order = m_environment.get_Vertex(neighbor_id)->calc_NeighborOrder(vertex_id);

		    encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
												sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												out_neighbor_order,
												sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												N_neighbor_Neighbors + in_neighbor_order,
												sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												0,
												sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												0,
												sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)),
												sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		    
		    encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
												sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
												N_vertex_Neighbors + out_neighbor_order,
												sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[neighbor_id], sIntegerIndex(layer)),
												in_neighbor_order);

		    ++out_neighbor_order;
		}
		
		encoding_context.m_clause_generator->cast_ConditionalEquality(solver,
											    sSpecifiedStateIdentifier(&encoding_context.m_transition_Actions[vertex_id], sIntegerIndex(layer)),
											    2 * N_vertex_Neighbors,
											    sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											    sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									     robot_id);
	}
/*
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver, total_Literal_cnt, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id);
	}
*/
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										 robot_id);
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
		    sStateClauseGenerator::States_vector goal_case_disj_States;
		    
		    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    goal_case_disj_States.push_back(0);
		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(*robot_id);
		    }
		    encoding_context.m_clause_generator->cast_DisjunctiveEquality(solver,
												goal_case_disj_Identifiers,
												goal_case_disj_States);
		}
		else
		{
		    encoding_context.m_clause_generator->cast_Equality(solver,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sRobotArrangement::VACANT_VERTEX);
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


    void sMultirobotInstance::to_Stream_PuzzleCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool verbose) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Edges = m_environment.get_EdgeCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action", 1 + 2 * N_Edges, sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_transition_action = transition_action;
	encoding_context.register_TranslateIdentifier(encoding_context.m_transition_action);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	    Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	}
         
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int action_id;

		    if ((*out_neighbor)->m_edge->m_arc_uv.m_target->m_id == vertex_id)
		    {
			action_id = (*out_neighbor)->m_edge->m_id;
		    }
		    else
		    {
			action_id = (*out_neighbor)->m_edge->m_id + N_Edges;
		    }

		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		    {
			if (remain_vertex_id != vertex_id && remain_vertex_id != neighbor_id)
			{
			    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
			    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
			}
		    }
		    
		    Clause_cnt += encoding_context.m_clause_generator->count_LargeConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												      action_id,
												      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												      0,
												      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												      0,
												      occupancy_Identifiers_A,
												      occupancy_Identifiers_B);
		}
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		{
		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
		}

		Clause_cnt += encoding_context.m_clause_generator->count_LargeConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												  2 * N_Edges,
												  occupancy_Identifiers_A,
												  occupancy_Identifiers_B);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      robot_id);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									      robot_id);
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot inverse SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {

		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), verbose);
	    }
	    encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)), verbose);
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;

		    int action_id;
		    if ((*out_neighbor)->m_edge->m_arc_uv.m_target->m_id == vertex_id)
		    {
			action_id = (*out_neighbor)->m_edge->m_id;
		    }
		    else
		    {
			action_id = (*out_neighbor)->m_edge->m_id + N_Edges;
		    }

		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		    {
			if (remain_vertex_id != vertex_id && remain_vertex_id != neighbor_id)
			{
			    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
			    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
			}
		    }
		    
		    encoding_context.m_clause_generator->generate_LargeConditionalEquality(fw,
											   sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
											   action_id,
											   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
											   0,
											   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
											   0,
											   occupancy_Identifiers_A,
											   occupancy_Identifiers_B,
											   verbose);
		}
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		{
		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
		}

		encoding_context.m_clause_generator->generate_LargeConditionalEquality(fw,
										       sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
										       2 * N_Edges,
										       occupancy_Identifiers_A,
										       occupancy_Identifiers_B,
										       verbose);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), robot_id, verbose);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), robot_id, verbose);
	} 	
    }


    void sMultirobotInstance::to_Memory_PuzzleCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Edges = m_environment.get_EdgeCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableStateIdentifier transition_action(&encoding_context.m_variable_store, "transition_action", 1 + 2 * N_Edges, sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_transition_action = transition_action;
	encoding_context.register_TranslateIdentifier(encoding_context.m_transition_action);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	    encoding_context.m_clause_generator->cast_Alignment(solver,
									      sSpecifiedStateIdentifier(&transition_action, sIntegerIndex(layer)));
	}
         
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{	
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sVertex::Neighbors_list &out_Neighbors =  m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    int action_id;

		    if ((*out_neighbor)->m_edge->m_arc_uv.m_target->m_id == vertex_id)
		    {
			action_id = (*out_neighbor)->m_edge->m_id;
		    }
		    else
		    {
			action_id = (*out_neighbor)->m_edge->m_id + N_Edges;
		    }

		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		    sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		    {
			if (remain_vertex_id != vertex_id && remain_vertex_id != neighbor_id)
			{
			    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
			    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
			}
		    }
		    
		    encoding_context.m_clause_generator->cast_LargeConditionalEquality(solver,
												     sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												     action_id,
												     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
												     0,
												     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(neighbor_id), sIntegerIndex(layer)),
												     0,
												     occupancy_Identifiers_A,
												     occupancy_Identifiers_B);
		}
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_A;
		sStateClauseGenerator::SpecifiedStateIdentifiers_vector occupancy_Identifiers_B;

		for (int remain_vertex_id = 0; remain_vertex_id < N_Vertices; ++remain_vertex_id)
		{
		    occupancy_Identifiers_A.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer + 1)));
		    occupancy_Identifiers_B.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(remain_vertex_id), sIntegerIndex(layer)));
		}

		encoding_context.m_clause_generator->cast_LargeConditionalEquality(solver,
												 sSpecifiedStateIdentifier(&encoding_context.m_transition_action, sIntegerIndex(layer)),
												 2 * N_Edges,
												 occupancy_Identifiers_A,
												 occupancy_Identifiers_B);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									     robot_id);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									     robot_id);
	}
    }    


    void sMultirobotInstance::to_Stream_BitwiseCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_BitwiseGeneratingMode();
	to_Stream_HeuristicAdvancedCNFsat(fw, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_BitwiseCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	encoding_context.switchTo_BitwiseGeneratingMode();
	to_Memory_HeuristicAdvancedCNFsat(solver, encoding_context, indent, verbose);
    }
    

    void sMultirobotInstance::to_Stream_FlowCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_robot(&encoding_context.m_variable_store, "vertex_occupancy_by_robot", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	
	encoding_context.m_vertex_occupancy_by_robot = vertex_occupancy_by_robot;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_robot);

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()),
							    sIntegerScope(0, encoding_context.m_N_Layers - 2));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       mutex_target_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt, 
											 mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										       mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Effect(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Effect(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
			
			++out_neighbor_index;
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    for (int robot_id = 1; robot_id <= N_Robots;  ++robot_id)
	    {
		if (robot_id == init_robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
	    }
	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_id == goal_robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_bit_disj_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(*robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_bit_disj_Identifiers);

		    if (robot_IDs.size() == 1)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_IDs.find(robot_id) == robot_IDs.end())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}

	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot flow SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", encoding_context.m_N_Layers);
	fprintf(fw, "c number of visible variables = %d\n", N_cnf_Variables - aux_Variable_cnt);
	fprintf(fw, "c number of hidden variables = %d\n", aux_Variable_cnt);
	fprintf(fw, "c number of literals = %d\n", total_Literal_cnt);
	fprintf(fw, "c number of clauses = %d\n", Clause_cnt);
	fprintf(fw, "c number of propositional variables = %d\n", N_cnf_Variables);
	fprintf(fw, "c number of clauses / number of variables = %.3f\n", (double)Clause_cnt / N_cnf_Variables);
	fprintf(fw, "c number of literals / number of clauses = %.3f\n", (double)total_Literal_cnt /  Clause_cnt);
	to_Stream(fw, "c ");
#endif
	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    encoding_context.m_bit_generator->generate_BiangleMutex(fw,
									    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
	    }	    
	}
       
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		encoding_context.m_bit_generator->generate_Implication(fw,
								       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    mutex_target_Identifiers);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
									    mutex_source_Identifiers);

		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_Effect(fw,
								      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			encoding_context.m_bit_generator->generate_Effect(fw,
									  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									  sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									  sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
			
			++out_neighbor_index;
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    for (int robot_id = 1; robot_id <= N_Robots;  ++robot_id)
	    {
		if (robot_id == init_robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
	    }
	    if (init_robot_id > 0)
	    {
		encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_id == goal_robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    else
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_bit_disj_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(*robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_bit_disj_Identifiers);

		    if (robot_IDs.size() == 1)
		    {
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_IDs.find(robot_id) == robot_IDs.end())
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }


    void sMultirobotInstance::to_Memory_FlowCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_robot(&encoding_context.m_variable_store, "vertex_occupancy_by_robot", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	
	encoding_context.m_vertex_occupancy_by_robot = vertex_occupancy_by_robot;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_robot);

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()),
							    sIntegerScope(0, encoding_context.m_N_Layers - 2));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));

		    encoding_context.m_bit_generator->cast_Implication(solver,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_Identifiers;
		target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    int neighbor_id = (*out_neighbor)->m_target->m_id;
		    target_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));

		    encoding_context.m_bit_generator->cast_BiangleMutex(solver,
										      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      target_Identifiers);
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		encoding_context.m_bit_generator->cast_Implication(solver,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->cast_Implication(solver,
										     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      mutex_target_Identifiers);
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_target_Identifiers);

		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		{
		    int in_neighbor_id = (*in_neighbor)->m_target->m_id;

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(in_neighbor_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int out_neighbor_id = (*out_neighbor)->m_target->m_id;
			if (out_neighbor_id == vertex_id)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[in_neighbor_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));
			}
			++out_neighbor_index;
		    }
		}
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
										      mutex_source_Identifiers);
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_Effect(solver,
										sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
										sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    int out_neighbor_index = 1;
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			encoding_context.m_bit_generator->cast_Effect(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
			
			++out_neighbor_index;
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    for (int robot_id = 1; robot_id <= N_Robots;  ++robot_id)
	    {
		if (robot_id == init_robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
		else
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(0)));
		}
	    }
	    if (init_robot_id > 0)
	    {
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
		encoding_context.m_bit_generator->cast_BitUnset(solver,
									      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);
		
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_id == goal_robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

		    
		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_bit_disj_Identifiers.push_back(sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(*robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_Disjunction(solver,
										     goal_bit_disj_Identifiers);

		    if (robot_IDs.size() == 1)
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    if (robot_IDs.find(robot_id) == robot_IDs.end())
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_robot, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
	    }
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


/*----------------------------------------------------------------------------*/

} // namespace sReloc
