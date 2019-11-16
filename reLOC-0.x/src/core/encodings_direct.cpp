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
/* encodings_direct.cpp / 0.21-robik_013                                      */
/*----------------------------------------------------------------------------*/
//
// Multi-robot path-finding encodings based on
// direct state variable representation.
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

    void sMultirobotInstance::to_Screen_MatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_MatchingCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicMatchingCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicMatchingCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_DirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_DirectCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicDirectCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicDirectCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_SimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_SimplicialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeuristicSimplicialCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeuristicSimplicialCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_SingularCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_SingularCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_PluralCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	to_Stream_PluralCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_Plural2CNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_Plural2CNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_HeightedCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_HeightedCNFsat(stdout, encoding_context, indent, verbose);
    }

    
    sResult sMultirobotInstance::to_File_MatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MatchingCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicMatchingCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicMatchingCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_DirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_DirectCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicDirectCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicDirectCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_SimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_SimplicialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeuristicSimplicialCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeuristicSimplicialCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_SingularCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_SingularCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_PluralCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose) const
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PluralCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_Plural2CNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_Plural2CNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_HeightedCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_HeightedCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    void sMultirobotInstance::to_Stream_MatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
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
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
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
		/*
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
		*/
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
		Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									      init_robot_id);

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
		Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									       goal_robot_id);

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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
		    
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
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);

		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot matching SAT encoding\n", sPRODUCT);
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
//		fprintf(fw, "c label alignment layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
//		fprintf(fw, "c label nonzero layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
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

//		    fprintf(fw, "c label biangle_mutex layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_BiangleMutex(fw,
									    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(neighbor_id), sIntegerIndex(layer)));
		    ++out_neighbor_index;
		}
		/*
		fprintf(fw, "c label target_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
		*/
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

//		fprintf(fw, "c label water_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_Implication(fw,
								       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
								       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
		mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
//		    fprintf(fw, "c label water_target_implication layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_Implication(fw,
									   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)));

		    ++out_neighbor_index;
		}

//		fprintf(fw, "c label water_target_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    mutex_target_Identifiers);

//		fprintf(fw, "c label mutex_target layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_target_Identifiers);

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
		
//		fprintf(fw, "c label water_source_implication layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)),
									    mutex_source_Identifiers);
//		fprintf(fw, "c label mutex_source layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,  mutex_source_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
//		fprintf(fw, "c label occupancy_water_implication_1 layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
//		    fprintf(fw, "c label occupancy_water_implication_2 layer=%d vertex=%d\n", layer, vertex_id);
		    encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
										   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    
		    ++out_neighbor_index;
		}
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
//	    fprintf(fw, "c label init_occupancy vertex=%d\n", vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

	    if (init_robot_id > 0)
	    {
//		fprintf(fw, "c label init_water_set vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(0)));
	    }
	    else
	    {
//		fprintf(fw, "c label init_water_unset vertex=%d\n", vertex_id);
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
//		fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), goal_robot_id);

		if (goal_robot_id > 0)
		{
//		    fprintf(fw, "c label goal_water_set vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
//		    fprintf(fw, "c label goal_water_unset vertex=%d\n", vertex_id);
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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;

			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
//			fprintf(fw, "c label goal_occupancy_disjunction vertex=%d\n", vertex_id);
			encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States);
		    }
		    else
		    {
//			fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
			encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), *robot_IDs.begin());
//			fprintf(fw, "c label goal_water_set vertex=%d\n", vertex_id);
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
//		    fprintf(fw, "c label goal_occupancy vertex=%d\n", vertex_id);
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);

//		    fprintf(fw, "c label goal_water_unset vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Memory_MatchingCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
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
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
											sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
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
		encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)),
									     init_robot_id);

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
		encoding_context.m_bit_generator->cast_Equality(solver,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									      goal_robot_id);

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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
		    
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
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_clause_generator->cast_Equality(solver,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sRobotArrangement::VACANT_VERTEX);

		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_HeuristicMatchingCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		*/
	    }
	}

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
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
/*
		Clause_cnt += encoding_context.m_bit_generator->count_ZeroImplication(aux_Variable_cnt, total_Literal_cnt,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
*/
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
		/*
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
		*/
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
		Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
									      total_Literal_cnt,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

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
		Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									       goal_robot_id);

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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
			
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
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      sRobotArrangement::VACANT_VERTEX);

		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
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
		}
		Clause_cnt += encoding_context.m_clause_generator->count_Disequalities(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       robot_IDs);
	    }
	}


	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot heuristic matching SAT encoding\n", sPRODUCT);
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
		/*
		encoding_context.m_clause_generator->generate_Alignment(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		*/
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));

/*
		encoding_context.m_bit_generator->generate_ZeroImplication(fw,
									   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
*/
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
		/*
		encoding_context.m_bit_generator->generate_MultiImplication(fw,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									    target_Identifiers);
		*/
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

		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, mutex_target_Identifiers);

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
		encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
										   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    
		    ++out_neighbor_index;
		}
	    }
	}
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

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
		encoding_context.m_bit_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), goal_robot_id);

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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
			
			goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			goal_case_disj_States.push_back(0);
			
			for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
			{
			    goal_case_disj_Identifiers.push_back(sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
			    goal_case_disj_States.push_back(*robot_id);
			}
			encoding_context.m_clause_generator->generate_DisjunctiveEquality(fw, goal_case_disj_Identifiers, goal_case_disj_States);
		    }
		    else
		    {
			encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), *robot_IDs.begin());
			encoding_context.m_bit_generator->generate_BitSet(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_clause_generator->generate_Equality(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)), sRobotArrangement::VACANT_VERTEX);
		    encoding_context.m_bit_generator->generate_BitUnset(fw, sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
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
		}
		encoding_context.m_clause_generator->generate_Disequalities(fw, sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)), robot_IDs);
	    }
	}
    }


    void sMultirobotInstance::to_Memory_HeuristicMatchingCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	    {
		/*
		encoding_context.m_clause_generator->cast_Alignment(solver,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		*/
	    }
	}

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
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
											sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
/*
		encoding_context.m_bit_generator->cast_ZeroImplication(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)));
*/
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
		/*
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										       target_Identifiers);
		*/
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
		encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(0), sIntegerIndex(layer)),
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		int out_neighbor_index = 1;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index), sIntegerIndex(layer)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex((*out_neighbor)->m_target->m_id), sIntegerIndex(layer + 1)));
		    ++out_neighbor_index;
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);
	    encoding_context.m_clause_generator->cast_Equality(solver,
									     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(0)), init_robot_id);

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
		encoding_context.m_bit_generator->cast_Equality(solver,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
									      goal_robot_id);

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
		    if (robot_IDs.size() > 1)
		    {
			sStateClauseGenerator::SpecifiedStateIdentifiers_vector goal_case_disj_Identifiers;
			sStateClauseGenerator::States_vector goal_case_disj_States;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_bit_disj_Identifiers;
			
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
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    encoding_context.m_clause_generator->cast_Equality(solver,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sRobotArrangement::VACANT_VERTEX);

		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id), sIntegerIndex(encoding_context.m_N_Layers - 1)));
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

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	    {		    
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
		}
		encoding_context.m_clause_generator->cast_Disequalities(solver,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id), sIntegerIndex(layer)),
										      robot_IDs);
	    }
	}
    }
    

    void sMultirobotInstance::to_Stream_DirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										  total_Literal_cnt,
										  robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   source_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			{
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(vertex_id),
										       sIntegerIndex(layer + 1)));
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(neighbor_id),
										       sIntegerIndex(layer)));
			}

			Clause_cnt += encoding_context.m_bit_generator->count_SwapConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     sSpecifiedBitIdentifier(&robot_location_in_vertex,
														     sIntegerIndex(robot_id),
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     sSpecifiedBitIdentifier(&robot_location_in_vertex,
														     sIntegerIndex(robot_id),
														     sIntegerIndex(neighbor_id),
														     sIntegerIndex(layer + 1)),
											     unset_robots_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot direct SAT encoding\n", sPRODUCT);
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

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_Identifiers);
		encoding_context.m_bit_generator->generate_Disjunction(fw, robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    encoding_context.m_bit_generator->generate_MultiImplication(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(layer)),
										source_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			{
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(vertex_id),
										       sIntegerIndex(layer + 1)));
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(neighbor_id),
										       sIntegerIndex(layer)));
			}

			encoding_context.m_bit_generator->generate_SwapConstraint(fw,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(layer)),
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(neighbor_id),
													  sIntegerIndex(layer + 1)),
										  unset_robots_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Memory_DirectCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_Identifiers);
		encoding_context.m_bit_generator->cast_Disjunction(solver,
										 robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  source_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			{
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(vertex_id),
										       sIntegerIndex(layer + 1)));
			    unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										       sIntegerIndex(unset_robot_id),
										       sIntegerIndex(neighbor_id),
										       sIntegerIndex(layer)));
			}

			encoding_context.m_bit_generator->cast_SwapConstraint(solver,
											    sSpecifiedBitIdentifier(&robot_location_in_vertex,
														    sIntegerIndex(robot_id),
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&robot_location_in_vertex,
														    sIntegerIndex(robot_id),
														    sIntegerIndex(neighbor_id),
														    sIntegerIndex(layer + 1)),
											    unset_robots_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
								    sSpecifiedBitIdentifier(&robot_location_in_vertex,
											    sIntegerIndex(robot_id),
											    sIntegerIndex(vertex_id),
											    sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->cast_BitSet(solver,
							      sSpecifiedBitIdentifier(&robot_location_in_vertex,
										      sIntegerIndex(init_robot_id),
										      sIntegerIndex(vertex_id),
										      sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
								    sSpecifiedBitIdentifier(&robot_location_in_vertex,
											    sIntegerIndex(robot_id),
											    sIntegerIndex(vertex_id),
											    sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_BitSet(solver,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(goal_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_Disjunction(solver,
								       goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->cast_BitUnset(solver,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_HeuristicDirectCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										  total_Literal_cnt,
										  robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       target_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			    for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			    {
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(vertex_id),
											   sIntegerIndex(layer + 1)));
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(neighbor_id),
											   sIntegerIndex(layer)));
			    }

			    Clause_cnt += encoding_context.m_bit_generator->count_SwapConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 sSpecifiedBitIdentifier(&robot_location_in_vertex,
															 sIntegerIndex(robot_id),
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 sSpecifiedBitIdentifier(&robot_location_in_vertex,
															 sIntegerIndex(robot_id),
															 sIntegerIndex(neighbor_id),
															 sIntegerIndex(layer + 1)),
												 unset_robots_Identifiers);
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot direct SAT encoding\n", sPRODUCT);
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
	
	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_Identifiers);
		encoding_context.m_bit_generator->generate_Disjunction(fw, robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(layer)));
		    }
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
			
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&robot_location_in_vertex,
															  sIntegerIndex(robot_id),
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  target_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->generate_MultiImplication(fw,
										    sSpecifiedBitIdentifier(&robot_location_in_vertex,
													    sIntegerIndex(robot_id),
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(layer)),
										    source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			    for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			    {
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(vertex_id),
											   sIntegerIndex(layer + 1)));
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(neighbor_id),
											   sIntegerIndex(layer)));
			    }
			    
			    encoding_context.m_bit_generator->generate_SwapConstraint(fw,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(layer)),
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(neighbor_id),
													      sIntegerIndex(layer + 1)),
										      unset_robots_Identifiers);
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Memory_HeuristicDirectCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// robot is placed in exactly one vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_Identifiers;

		for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
		{
		    robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_Identifiers);
		encoding_context.m_bit_generator->cast_Disjunction(solver,
										 robot_Identifiers);
	    }
	}

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      target_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector unset_robots_Identifiers;
			    for (int unset_robot_id = 1; unset_robot_id <= N_Robots; ++unset_robot_id)
			    {
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(vertex_id),
											   sIntegerIndex(layer + 1)));
				unset_robots_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
											   sIntegerIndex(unset_robot_id),
											   sIntegerIndex(neighbor_id),
											   sIntegerIndex(layer)));
			    }

			    encoding_context.m_bit_generator->cast_SwapConstraint(solver,
												sSpecifiedBitIdentifier(&robot_location_in_vertex,
															sIntegerIndex(robot_id),
															sIntegerIndex(vertex_id),
															sIntegerIndex(layer)),
												sSpecifiedBitIdentifier(&robot_location_in_vertex,
															sIntegerIndex(robot_id),
															sIntegerIndex(neighbor_id),
															sIntegerIndex(layer + 1)),
												unset_robots_Identifiers);
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(init_robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(goal_robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_Disjunction(solver,
										     goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->cast_BitUnset(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
    

    void sMultirobotInstance::to_Stream_SimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&empty_vertex,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       no_robot_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   target_robot_Identifiers);
		}
	    }	    
	}
	/*
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   source_robot_Identifiers);
		}
	    }	    
	}
	*/

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			Clause_cnt += encoding_context.m_bit_generator->count_NegativeSwapConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     sSpecifiedBitIdentifier(&robot_location_in_vertex,
															     sIntegerIndex(robot_id),
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     sSpecifiedBitIdentifier(&robot_location_in_vertex,
															     sIntegerIndex(robot_id),
															     sIntegerIndex(neighbor_id),
															     sIntegerIndex(layer + 1)),
												     sSpecifiedBitIdentifier(&empty_vertex,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer + 1)),
												     sSpecifiedBitIdentifier(&empty_vertex,
															     sIntegerIndex(neighbor_id),
															     sIntegerIndex(layer)));
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot simplicial SAT encoding\n", sPRODUCT);
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

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		fprintf(fw, "c label allmutex layer=%d vertex=%d\n", layer, vertex_id);
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}

		fprintf(fw, "c label multi_negative layer=%d vertex=%d\n", layer, vertex_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
												  sSpecifiedBitIdentifier(&empty_vertex,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  no_robot_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }

		    fprintf(fw, "c label multi_target_implication layer=%d vertex=%d robot=%d\n", layer, vertex_id, robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      target_robot_Identifiers);
		}
	    }	    
	}

	/*
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
		    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
		    }
		    encoding_context.m_bit_generator->generate_MultiImplication(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(layer)),
										source_robot_Identifiers);
		}
	    }	    
	}
	*/

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			fprintf(fw, "c label negative_swap layer=%d vertex=%d robot=%d\n", layer, vertex_id, robot_id);
			Clause_cnt += encoding_context.m_bit_generator->generate_NegativeSwapConstraint(fw,
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(neighbor_id),
																sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(neighbor_id),
																sIntegerIndex(layer)));
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		fprintf(fw, "c label init_bit_set vertex=%d\n", vertex_id);
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    fprintf(fw, "c label init_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    fprintf(fw, "c label goal_bit_set vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    fprintf(fw, "c label goal_disjunction vertex=%d\n", vertex_id);
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			fprintf(fw, "c label goal_bit_unset vertex=%d robot=%d\n", vertex_id, robot_id);
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Memory_SimplicialCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
											      sSpecifiedBitIdentifier(&empty_vertex,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      no_robot_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
		    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  target_robot_Identifiers);
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			int neighbor_id = (*out_neighbor)->m_target->m_id;

			encoding_context.m_bit_generator->cast_NegativeSwapConstraint(solver,
												    sSpecifiedBitIdentifier(&robot_location_in_vertex,
															    sIntegerIndex(robot_id),
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												    sSpecifiedBitIdentifier(&robot_location_in_vertex,
															    sIntegerIndex(robot_id),
															    sIntegerIndex(neighbor_id),
															    sIntegerIndex(layer + 1)),
												    sSpecifiedBitIdentifier(&empty_vertex,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)),
												    sSpecifiedBitIdentifier(&empty_vertex,
															    sIntegerIndex(neighbor_id),
															    sIntegerIndex(layer)));
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(init_robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(goal_robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_Disjunction(solver,
										     goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->cast_BitUnset(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_HeuristicSimplicialCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&empty_vertex,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       no_robot_Identifiers);
	    }
	}
	/*
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       target_robot_Identifiers);
		    }
		}
	    }	    
	}
	*/
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&robot_location_in_vertex,
														       sIntegerIndex(robot_id),
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;

			    Clause_cnt += encoding_context.m_bit_generator->count_NegativeSwapConstraint(aux_Variable_cnt,
													 total_Literal_cnt,
													 sSpecifiedBitIdentifier(&robot_location_in_vertex,
																 sIntegerIndex(robot_id),
																 sIntegerIndex(vertex_id),
																 sIntegerIndex(layer)),
													 sSpecifiedBitIdentifier(&robot_location_in_vertex,
																 sIntegerIndex(robot_id),
																 sIntegerIndex(neighbor_id),
																 sIntegerIndex(layer + 1)),
													 sSpecifiedBitIdentifier(&empty_vertex,
																 sIntegerIndex(vertex_id),
																 sIntegerIndex(layer + 1)),
													 sSpecifiedBitIdentifier(&empty_vertex,
																 sIntegerIndex(neighbor_id),
																 sIntegerIndex(layer)));
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&robot_location_in_vertex,
												     sIntegerIndex(init_robot_id),
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedBitIdentifier(&robot_location_in_vertex,
													   sIntegerIndex(robot_id),
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&robot_location_in_vertex,
													 sIntegerIndex(goal_robot_id),
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_Disjunction(aux_Variable_cnt,
										      total_Literal_cnt,
										      goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&robot_location_in_vertex,
														   sIntegerIndex(robot_id),
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&robot_location_in_vertex,
													       sIntegerIndex(robot_id),
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot simplicial SAT encoding\n", sPRODUCT);
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
	
	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(layer)));
		    }
		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->generate_AllMutexConstraint(fw, robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
												  sSpecifiedBitIdentifier(&empty_vertex,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  no_robot_Identifiers);
	    }
	}
	/*
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector target_robot_Identifiers;
			target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer + 1)));
			
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
			
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    target_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer + 1)));
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&robot_location_in_vertex,
															  sIntegerIndex(robot_id),
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  target_robot_Identifiers);
		    }
		}
	    }	    
	}
	*/
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));

			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->generate_MultiImplication(fw,
										    sSpecifiedBitIdentifier(&robot_location_in_vertex,
													    sIntegerIndex(robot_id),
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(layer)),
										    source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_NegativeSwapConstraint(fw,
													    sSpecifiedBitIdentifier(&robot_location_in_vertex,
																    sIntegerIndex(robot_id),
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													    sSpecifiedBitIdentifier(&robot_location_in_vertex,
																    sIntegerIndex(robot_id),
																    sIntegerIndex(neighbor_id),
																    sIntegerIndex(layer + 1)),
													    sSpecifiedBitIdentifier(&empty_vertex,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer + 1)),
													    sSpecifiedBitIdentifier(&empty_vertex,
																    sIntegerIndex(neighbor_id),
																    sIntegerIndex(layer)));
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->generate_BitSet(fw,
								  sSpecifiedBitIdentifier(&robot_location_in_vertex,
											  sIntegerIndex(init_robot_id),
											  sIntegerIndex(vertex_id),
											  sIntegerIndex(0)));
		
		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->generate_BitUnset(fw,
									sSpecifiedBitIdentifier(&robot_location_in_vertex,
												sIntegerIndex(robot_id),
												sIntegerIndex(vertex_id),
												sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_BitSet(fw,
								      sSpecifiedBitIdentifier(&robot_location_in_vertex,
											      sIntegerIndex(goal_robot_id),
											      sIntegerIndex(vertex_id),
											      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->generate_Disjunction(fw, goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->generate_BitUnset(fw,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Memory_HeuristicSimplicialCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	m_environment.calc_AllPairsShortestPaths();
	const sUndirectedGraph::Distances_2d_vector &all_pairs_Distances = m_environment.get_AllPairsShortestPaths();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableBitIdentifier robot_location_in_vertex(&encoding_context.m_variable_store, "robot_location_in_vertex", sIntegerScope(1, N_Robots), sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));
	sIndexableBitIdentifier empty_vertex(&encoding_context.m_variable_store, "empty_vertex", sIntegerScope(0, N_Vertices - 1), sIntegerScope(0, encoding_context.m_N_Layers - 1));

	encoding_context.m_robot_location_in_vertex = robot_location_in_vertex;
	encoding_context.register_TranslateIdentifier(encoding_context.m_robot_location_in_vertex);

	// there is at most one robot in a vertex
	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector robot_mutex_Identifiers;

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
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(layer)));
		    }

		    robot_mutex_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											robot_mutex_Identifiers);
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector no_robot_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    no_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer)));
		}
		encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
											      sSpecifiedBitIdentifier(&empty_vertex,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      no_robot_Identifiers);
	    }
	}
	for (int layer = 1; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector source_robot_Identifiers;
			source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(vertex_id), sIntegerIndex(layer - 1)));
		    
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;
			    source_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex, sIntegerIndex(robot_id), sIntegerIndex(neighbor_id), sIntegerIndex(layer - 1)));
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&robot_location_in_vertex,
														      sIntegerIndex(robot_id),
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      source_robot_Identifiers);
		    }
		}
	    }	    
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
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
		    if (!forbid_vertex && all_pairs_Distances[m_initial_arrangement.get_RobotLocation(robot_id)][vertex_id] <= layer)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    int neighbor_id = (*out_neighbor)->m_target->m_id;

			    encoding_context.m_bit_generator->cast_NegativeSwapConstraint(solver,
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
													sSpecifiedBitIdentifier(&robot_location_in_vertex,
																sIntegerIndex(robot_id),
																sIntegerIndex(neighbor_id),
																 sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer + 1)),
													sSpecifiedBitIdentifier(&empty_vertex,
																sIntegerIndex(neighbor_id),
																sIntegerIndex(layer)));
			}
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id == 0)
	    {
		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	    else
	    {
		for (int robot_id = 1; robot_id < init_robot_id; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&robot_location_in_vertex,
												    sIntegerIndex(init_robot_id),
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));

		for (int robot_id = init_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		{
		    encoding_context.m_bit_generator->cast_BitUnset(solver,
										  sSpecifiedBitIdentifier(&robot_location_in_vertex,
													  sIntegerIndex(robot_id),
													  sIntegerIndex(vertex_id),
													  sIntegerIndex(0)));
		}
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id == 0)
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id < goal_robot_id; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&robot_location_in_vertex,
													sIntegerIndex(goal_robot_id),
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    
		    for (int robot_id = goal_robot_id + 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_robot_Identifiers;

		    for (sRobotGoal::Robots_set::const_iterator robot_id = robot_IDs.begin(); robot_id != robot_IDs.end(); ++robot_id)
		    {
			goal_robot_Identifiers.push_back(sSpecifiedBitIdentifier(&robot_location_in_vertex,
										 sIntegerIndex(*robot_id),
										 sIntegerIndex(vertex_id),
										 sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		    encoding_context.m_bit_generator->cast_Disjunction(solver,
										     goal_robot_Identifiers);

		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			if (robot_IDs.find(robot_id) == robot_IDs.end())
			{
			    encoding_context.m_bit_generator->cast_BitUnset(solver,
											  sSpecifiedBitIdentifier(&robot_location_in_vertex,
														  sIntegerIndex(robot_id),
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)));			    
			}
		    }
		}
		else
		{
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			encoding_context.m_bit_generator->cast_BitUnset(solver,
										      sSpecifiedBitIdentifier(&robot_location_in_vertex,
													      sIntegerIndex(robot_id),
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_SingularCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
									       total_Literal_cnt,
									       sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)));
	}

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water[vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
	    
	    int out_neighbor_index = 0;
	    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		if (m_goal_arrangement.get_VertexOccupancy(vertex_id) == 0)
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														  sIntegerIndex(out_neighbor_index)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											   mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											total_Literal_cnt,
											mutex_target_Identifiers);
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.get_GoalCompatibility(vertex_id).empty())
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														  sIntegerIndex(out_neighbor_index)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id)),
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											   mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											total_Literal_cnt,
											mutex_target_Identifiers);
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

	    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
	    {
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;

		int in_neighbor_index = 0;
		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    if ((*out_neighbor)->m_target->m_id == vertex_id)
		    {
			break;
		    }
		    ++in_neighbor_index;
		}
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[(*in_neighbor)->m_target->m_id], sIntegerIndex(in_neighbor_index)));
	    }
	    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
										       mutex_source_Identifiers);
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
										    total_Literal_cnt,
										    mutex_source_Identifiers);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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
	fprintf(fw, "c %s : multirobot matching SAT encoding\n", sPRODUCT);
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
	    Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	}
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
	    
	    int out_neighbor_index = 0;
	    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		if (m_goal_arrangement.get_VertexOccupancy(vertex_id) == 0)
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														     sIntegerIndex(out_neighbor_index)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));		    
			++out_neighbor_index;
		    }

		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											      mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw, mutex_target_Identifiers);   
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.get_GoalCompatibility(vertex_id).empty())
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														     sIntegerIndex(out_neighbor_index)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id)),
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											      mutex_target_Identifiers);
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw, mutex_target_Identifiers);   
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;
  	    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
	    
	    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
	    {
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;

		int in_neighbor_index = 0;
		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    if ((*out_neighbor)->m_target->m_id == vertex_id)
		    {
			break;
		    }
		    ++in_neighbor_index;
		}
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[(*in_neighbor)->m_target->m_id], sIntegerIndex(in_neighbor_index)));
	    }
	    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											  mutex_source_Identifiers);
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_source_Identifiers);
	    }
	    else
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
										       mutex_source_Identifiers);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{

	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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


    void sMultirobotInstance::to_Memory_SingularCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store, "vertex_occupancy", N_Robots + 1, sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store, "vertex_occupancy_by_water", sIntegerScope(0, N_Vertices - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    encoding_context.m_clause_generator->cast_Alignment(solver,
									      sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)));
	}

	encoding_context.m_edge_occupancy_by_water.resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water(&encoding_context.m_variable_store, "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water[vertex_id] = edge_occupancy_by_water;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water[vertex_id]);

	    encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
										    sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
	    
	    int out_neighbor_index = 0;
	    const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		if (m_goal_arrangement.get_VertexOccupancy(vertex_id) == 0)
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														 sIntegerIndex(out_neighbor_index)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex((*out_neighbor)->m_target->m_id)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											  mutex_target_Identifiers);
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    encoding_context.m_bit_generator->cast_MultiNegation(solver,
										       mutex_target_Identifiers);
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		if (m_goal_specification.get_GoalCompatibility(vertex_id).empty())
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
														 sIntegerIndex(out_neighbor_index)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex((*out_neighbor)->m_target->m_id)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex((*out_neighbor)->m_target->m_id)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
											  mutex_target_Identifiers);
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_target_Identifiers);   
		}
		else
		{
		    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[vertex_id], sIntegerIndex(out_neighbor_index)));
		    
			++out_neighbor_index;
		    }
		    encoding_context.m_bit_generator->cast_MultiNegation(solver,
										       mutex_target_Identifiers);
		}
		break;
	    }
	    default:
	    {
		sASSERT(false);
		break;
	    }
	    }
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

	    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

	    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
	    {
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;

		int in_neighbor_index = 0;
		for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
		{
		    if ((*out_neighbor)->m_target->m_id == vertex_id)
		    {
			break;
		    }
		    ++in_neighbor_index;
		}
		mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water[(*in_neighbor)->m_target->m_id], sIntegerIndex(in_neighbor_index)));
	    }
	    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
	    {
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_source_Identifiers);
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)),
										      mutex_source_Identifiers);
	    }
	    else
	    {
		encoding_context.m_bit_generator->cast_MultiNegation(solver,
										   mutex_source_Identifiers);
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										 init_robot_id);
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->cast_Equality(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
										  goal_robot_id);
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			encoding_context.m_clause_generator->cast_Equality(solver,
											 sSpecifiedStateIdentifier(&vertex_occupancy, sIntegerIndex(vertex_id)),
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water, sIntegerIndex(vertex_id)));
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


    void sMultirobotInstance::to_Stream_PluralCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot plural SAT encoding\n", sPRODUCT);
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
		/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }

			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));

			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));

			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),*/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	}

//	int robot_limit = 0;

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    printf("ARANGEMENT\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    printf("SPECIFICATION A1\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			// no goal setup
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(encoding_context.m_N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));


/*
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

			int out_neighbor_index = 0;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id],
										       sIntegerIndex(out_neighbor_index)));
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_target_Identifiers);
*/
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


    void sMultirobotInstance::to_Memory_PluralCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose)) const
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		encoding_context.m_clause_generator->cast_Alignment(solver,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							    sIntegerScope(0, m_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
										    sSpecifiedStateIdentifier(&vertex_occupancy,
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											     /*	
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_source_Identifiers);
			
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_MultiNegation(solver,
											   mutex_source_Identifiers);
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_source_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&vertex_occupancy,
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)),
										 init_robot_id);
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->cast_Equality(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  goal_robot_id);
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			encoding_context.m_clause_generator->cast_Equality(solver,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)),
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_Plural2CNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() + m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
	
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							     "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							     sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(encoding_context.m_N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														       sIntegerIndex(out_neighbor_index + 1)),
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex((*out_neighbor)->m_target->m_id),
														       sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														       sIntegerIndex(out_neighbor_index + 1)),
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex((*out_neighbor)->m_target->m_id),
														       sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;
		    const sVertex::Neighbors_list &full_in_Neighbors = m_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = full_in_Neighbors.begin(); in_neighbor != full_in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			int out_neighbor_count = out_Neighbors.size();
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][(*in_neighbor)->m_target->m_id],
										   sIntegerIndex(out_neighbor_count + in_neighbor_index + 1)));
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}

		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(encoding_context.m_N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot plural-2 SAT encoding\n", sPRODUCT);
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
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
	    }
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(encoding_context.m_N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer + 1)));			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index + 1)),
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer + 1)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     /**/
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     /**/
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 /**/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer + 1)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index + 1)),
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer + 1)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;
		    const sVertex::Neighbors_list &full_in_Neighbors = m_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
//			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = full_in_Neighbors.begin(); in_neighbor != full_in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
			const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
			int out_neighbor_count = out_Neighbors.size();
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!full_out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][(*in_neighbor)->m_source->m_id],
										   sIntegerIndex(out_neighbor_count + in_neighbor_index + 1)));
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_source->m_id)->m_out_Neighbors;
		    
			int in_neighbor_index = 0;
#ifdef sDEBUG
			bool breaked = false;
#endif
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
//			    if ((*out_neighbor)->m_edge->m_arc_vu.m_target->m_id == vertex_id)
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
#ifdef sDEBUG
				breaked = true;
#endif	
				break;
			    }
			    ++in_neighbor_index;
			}
			if (!out_Neighbors.empty())
			{
			    sASSERT(breaked);
			}

			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_source->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			if (!mutex_source_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_source_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_source_Identifiers);
			}
		    }
		    else
		    {
			if (!mutex_source_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_source_Identifiers);
			}
		    }
		}
		else
		{
		    if (!mutex_source_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);

		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	    else
	    {
/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);

		Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
*/
	    }
	}

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);

		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
		}
		else
		{
/*
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)),
										      goal_robot_id);

		    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
*/
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(encoding_context.m_N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(encoding_context.m_N_Layers - 1)));
		    }
		}
		else
		{
/*
		    Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)),
											 *robot_IDs.begin());
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
										      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)));
*/
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


    void sMultirobotInstance::to_Memory_Plural2CNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, encoding_context.m_N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		encoding_context.m_clause_generator->cast_Alignment(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(encoding_context.m_N_Layers);

	for (int layer = 0; layer < encoding_context.m_N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() + m_environment.get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
	
		encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							     "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(encoding_context.m_N_Layers - 1),
							     sIntegerScope(0, m_sparse_environment.get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[encoding_context.m_N_Layers - 1][vertex_id]);

	    encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
										    sSpecifiedStateIdentifier(&vertex_occupancy,
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(encoding_context.m_N_Layers - 1)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
	}

	for (int layer = 0; layer < encoding_context.m_N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer + 1)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer + 1)));
			    encoding_context.m_bit_generator->cast_BiangleMutex(solver,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == encoding_context.m_N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											     /*	
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer + 1)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer + 1)));
			    encoding_context.m_bit_generator->cast_BiangleMutex(solver,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer + 1)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_in_Neighbors;
		    const sVertex::Neighbors_list &full_in_Neighbors = m_environment.get_Vertex(vertex_id)->m_in_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = full_in_Neighbors.begin(); in_neighbor != full_in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			const sVertex::Neighbors_list &full_out_Neighbors = m_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
			int out_neighbor_count = out_Neighbors.size();
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = full_out_Neighbors.begin(); out_neighbor != full_out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][(*in_neighbor)->m_target->m_id],
										   sIntegerIndex(out_neighbor_count + in_neighbor_index + 1)));
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_sparse_environment.get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_sparse_environment.get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == encoding_context.m_N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}

		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_source_Identifiers);
			
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_MultiNegation(solver,
											   mutex_source_Identifiers);
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_source_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&vertex_occupancy,
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)),
										 init_robot_id);
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->cast_Equality(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)),
										  goal_robot_id);
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(encoding_context.m_N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			encoding_context.m_clause_generator->cast_Equality(solver,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(encoding_context.m_N_Layers - 1)),
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(encoding_context.m_N_Layers - 1)));
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


    void sMultirobotInstance::to_Stream_HeightedCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	
	build_HeightedEnvironments_(encoding_context.m_max_total_cost);

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	int N_Layers = m_heighted_Environments.size();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->count_Alignment(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(N_Layers);

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
											 total_Literal_cnt,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(N_Layers - 1),
							    sIntegerScope(0, m_heighted_Environments[N_Layers - 1].get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id]);

	    Clause_cnt += encoding_context.m_bit_generator->count_NonzeroImplication(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(N_Layers - 1)),
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)));
	}
/*
	int soft_Clause_cnt = 0;

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int neighbor_idx = 0; neighbor_idx < m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount(); ++neighbor_idx)
		{
		    int cnt = encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
													 sIntegerIndex(neighbor_idx)));
		    Clause_cnt += cnt;
		    soft_Clause_cnt += cnt;
		}
	    }
	}
*/
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
												total_Literal_cnt,
												mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex((*out_neighbor)->m_target->m_id),
															  sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																  sIntegerIndex(out_neighbor_index)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex(vertex_id),
																    sIntegerIndex(layer)),
													  sSpecifiedStateIdentifier(&vertex_occupancy,
																    sIntegerIndex((*out_neighbor)->m_target->m_id),
																    sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												   mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														  sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												  total_Literal_cnt,
												  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															  sIntegerIndex(0)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer)),
												  sSpecifiedStateIdentifier(&vertex_occupancy,
															    sIntegerIndex(vertex_id),
															    sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														      sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex((*out_neighbor)->m_target->m_id),
														      sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->count_ConditionalEquality(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															      sIntegerIndex(out_neighbor_index + 1)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex(vertex_id),
																sIntegerIndex(layer)),
												      sSpecifiedStateIdentifier(&vertex_occupancy,
																sIntegerIndex((*out_neighbor)->m_target->m_id),
																sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(layer)),
											       mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiNegation(aux_Variable_cnt,
											    total_Literal_cnt,
											    mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(layer)),
											   mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
										  total_Literal_cnt,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(0)),
										  init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
									     total_Literal_cnt,
									     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												     sIntegerIndex(vertex_id),
												     sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Equality(aux_Variable_cnt,
										   total_Literal_cnt,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)),
										   goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													 sIntegerIndex(vertex_id),
													 sIntegerIndex(N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->count_Equality(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedStateIdentifier(&vertex_occupancy,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(N_Layers - 1)),
											  *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(N_Layers - 1)));
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
	fprintf(fw, "c %s : multirobot plural SAT encoding\n", sPRODUCT);
	fprintf(fw, "c %s\n", sCOPYRIGHT);
	fprintf(fw, "c number of layers = %d\n", N_Layers);
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
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		Clause_cnt += encoding_context.m_clause_generator->generate_Alignment(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											    sSpecifiedStateIdentifier(&vertex_occupancy,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														    sIntegerIndex(vertex_id),
														    sIntegerIndex(layer)));
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_NonzeroImplication(fw,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(N_Layers - 1)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)));
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);			    
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));
			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),
												 */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
												   mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index)),
												     /*
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex(vertex_id),
															     sIntegerIndex(layer)),
												     */
												     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															     sIntegerIndex((*out_neighbor)->m_target->m_id),
															     sIntegerIndex(layer)));
				Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																     sIntegerIndex(out_neighbor_index)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex(vertex_id),
																       sIntegerIndex(layer)),
													     sSpecifiedStateIdentifier(&vertex_occupancy,
																       sIntegerIndex((*out_neighbor)->m_target->m_id),
																       sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }

			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															      sIntegerIndex(vertex_id),
															      sIntegerIndex(layer)),
												      mutex_target_Identifiers);
			    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
													mutex_target_Identifiers);
			}
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(0)),
											     /*
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer)),
											     */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex(vertex_id),
														     sIntegerIndex(layer + 1)));

			Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(0)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer + 1)));

			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index + 1)),
												 /*
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex(vertex_id),
															 sIntegerIndex(layer)),*/
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_ConditionalEquality(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index + 1)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));

			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_target_Identifiers);
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												    mutex_source_Identifiers);
			
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_source_Identifiers);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_source_Identifiers);
		    }
		}
		else
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_source_Identifiers);
		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		}
	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
										     sSpecifiedStateIdentifier(&vertex_occupancy,
													       sIntegerIndex(vertex_id),
													       sIntegerIndex(0)),
										     init_robot_id);
		Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(0)));
	    }
	}

//	int robot_limit = 0;

	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    printf("ARANGEMENT\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Equality(fw,
										      sSpecifiedStateIdentifier(&vertex_occupancy,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)),
										      goal_robot_id);
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(N_Layers - 1)));
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    printf("SPECIFICATION A1\n");
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		const sRobotGoal::Robots_set &robot_IDs = m_goal_specification.get_GoalCompatibility(vertex_id);
		
		if (!robot_IDs.empty())
		{
		    if (robot_IDs.size() > 1)
		    {
			// no goal setup
			sASSERT(false);
		    }
		    else
		    {
			Clause_cnt += encoding_context.m_clause_generator->generate_Equality(fw,
											     sSpecifiedStateIdentifier(&vertex_occupancy,
														       sIntegerIndex(vertex_id),
														       sIntegerIndex(N_Layers - 1)),
											     *robot_IDs.begin());
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(N_Layers - 1)));


/*
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

			int out_neighbor_index = 0;
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id],
										       sIntegerIndex(out_neighbor_index)));
			    ++out_neighbor_index;
			}
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegation(fw,
											       mutex_target_Identifiers);
*/
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
	printf("generation finished\n");
    }


    void sMultirobotInstance::to_Memory_HeightedCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	
	build_HeightedEnvironments_(encoding_context.m_max_total_cost);
	
	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();

	int N_Layers = m_heighted_Environments.size();

	sIndexableStateIdentifier vertex_occupancy(&encoding_context.m_variable_store,
						   "vertex_occupancy",
						   N_Robots + 1,
						   sIntegerScope(0, N_Vertices - 1),
						   sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy = vertex_occupancy;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy);

	sIndexableBitIdentifier vertex_occupancy_by_water(&encoding_context.m_variable_store,
							  "vertex_occupancy_by_water",
							  sIntegerScope(0, N_Vertices - 1),
							  sIntegerScope(0, N_Layers - 1));
	encoding_context.m_vertex_occupancy_by_water = vertex_occupancy_by_water;
	encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		/*
		encoding_context.m_clause_generator->cast_Alignment(solver,
										   sSpecifiedStateIdentifier(&vertex_occupancy,
													     sIntegerIndex(vertex_id),
													     sIntegerIndex(layer)));
		*/
	    }
	}
	encoding_context.m_edge_occupancy_by_water_.resize(N_Layers);

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    encoding_context.m_edge_occupancy_by_water_[layer].resize(N_Vertices);


	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
								 "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount()));
		encoding_context.m_edge_occupancy_by_water_[layer][vertex_id] = edge_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[layer][vertex_id]);
		
		encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
											sSpecifiedStateIdentifier(&vertex_occupancy,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														sIntegerIndex(vertex_id),
														sIntegerIndex(layer)));
	    }
	}
	encoding_context.m_edge_occupancy_by_water_[N_Layers - 1].resize(N_Vertices);

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    sIndexableBitIdentifier edge_occupancy_by_water_(&encoding_context.m_variable_store,
							    "edge_occupancy_by_water-" + sInt_32_to_String(vertex_id) + "_" + sInt_32_to_String(N_Layers - 1),
							    sIntegerScope(0, m_heighted_Environments[N_Layers - 1].get_Vertex(vertex_id)->calc_NeighborCount() - 1));
	    encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id] = edge_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water_[N_Layers - 1][vertex_id]);

	    encoding_context.m_bit_generator->cast_NonzeroImplication(solver,
										    sSpecifiedStateIdentifier(&vertex_occupancy,
													      sIntegerIndex(vertex_id),
													      sIntegerIndex(N_Layers - 1)),
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(N_Layers - 1)));
	}
/*
	int soft_Clause_cnt = 0;

	for (int layer = 0; layer < N_Layers - 1; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		for (int neighbor_idx = 0; neighbor_idx < m_heighted_Environments[layer].get_Vertex(vertex_id)->calc_NeighborCount(); ++neighbor_idx)
		{
		    int cnt = encoding_context.m_bit_generator->cast_BitSet(solver,
										 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
													 sIntegerIndex(neighbor_idx)));
		    Clause_cnt += cnt;
		    soft_Clause_cnt += cnt;
		}
	    }
	}
*/
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int out_neighbor_index = 0;
		const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		switch (m_goal_type)
		{
		case GOAL_TYPE_ARRANGEMENT:
		{
		    if (layer == N_Layers - 1)
		    {
			if (m_goal_arrangement.get_VertexOccupancy(vertex_id) != 0)
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														 sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		case GOAL_TYPE_SPECIFICATION:
		{
		    if (layer == N_Layers - 1)
		    {
			if (!m_goal_specification.get_GoalCompatibility(vertex_id).empty())
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
											   sIntegerIndex(out_neighbor_index)));
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiNegation(solver,
											       mutex_target_Identifiers);
			}
			else
			{
			    for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(out_neighbor_index)),
												  /*
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  */
												 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															 sIntegerIndex((*out_neighbor)->m_target->m_id),
															 sIntegerIndex(layer)));
				encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
																 sIntegerIndex(out_neighbor_index)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex(vertex_id),
																   sIntegerIndex(layer)),
													 sSpecifiedStateIdentifier(&vertex_occupancy,
																   sIntegerIndex((*out_neighbor)->m_target->m_id),
																   sIntegerIndex(layer)));
				mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index)));
			    
				++out_neighbor_index;
			    }
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
															  sIntegerIndex(vertex_id),
															  sIntegerIndex(layer)),
												  mutex_target_Identifiers);
			    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												    mutex_target_Identifiers);   
			}
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														 sIntegerIndex(0)),
											  /*
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  */
											 sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														 sIntegerIndex(vertex_id),
														  sIntegerIndex(layer + 1)));
			encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															 sIntegerIndex(0)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer)),
												 sSpecifiedStateIdentifier(&vertex_occupancy,
															   sIntegerIndex(vertex_id),
															   sIntegerIndex(layer + 1)));
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(0)));

			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
														     sIntegerIndex(out_neighbor_index + 1)),
											      /*
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      */
											     sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														     sIntegerIndex((*out_neighbor)->m_target->m_id),
														     sIntegerIndex(layer)));
			    encoding_context.m_bit_generator->cast_ConditionalEquality(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id],
															     sIntegerIndex(out_neighbor_index + 1)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex(vertex_id),
															       sIntegerIndex(layer)),
												     sSpecifiedStateIdentifier(&vertex_occupancy,
															       sIntegerIndex((*out_neighbor)->m_target->m_id),
															       sIntegerIndex(layer)));
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][vertex_id], sIntegerIndex(out_neighbor_index + 1)));
			    
			    ++out_neighbor_index;
			}
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_target_Identifiers);
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_target_Identifiers);   
		    }
		    break;
		}
		default:
		{
		    sASSERT(false);
		    break;
		}
		}
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_source_Identifiers;

		if (layer > 0)
		{
		    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer - 1][vertex_id],
									       sIntegerIndex(0)));
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;

		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		else
		{
		    const sVertex::Neighbors_list &in_Neighbors = m_heighted_Environments[layer].get_Vertex(vertex_id)->m_Neighbors;
		
		    for (sVertex::Neighbors_list::const_iterator in_neighbor = in_Neighbors.begin(); in_neighbor != in_Neighbors.end(); ++in_neighbor)
		    {
			const sVertex::Neighbors_list &out_Neighbors = m_heighted_Environments[layer].get_Vertex((*in_neighbor)->m_target->m_id)->m_Neighbors;
		    
			int in_neighbor_index = 0;
			for (sVertex::Neighbors_list::const_iterator out_neighbor = out_Neighbors.begin(); out_neighbor != out_Neighbors.end(); ++out_neighbor)
			{
			    if ((*out_neighbor)->m_target->m_id == vertex_id)
			    {
				break;
			    }
			    ++in_neighbor_index;
			}
			if (layer == N_Layers - 1)
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index)));
			}
			else
			{
			    mutex_source_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water_[layer][(*in_neighbor)->m_target->m_id],
										       sIntegerIndex(in_neighbor_index + 1)));
			}
		    }
		}
		if (layer == 0)
		{
		    if (m_initial_arrangement.get_VertexOccupancy(vertex_id) == 0)
		    {
			encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
												mutex_source_Identifiers);
			
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														      sIntegerIndex(vertex_id),
														      sIntegerIndex(layer)),
											      mutex_source_Identifiers);
		    }
		    else
		    {
			encoding_context.m_bit_generator->cast_MultiNegation(solver,
											   mutex_source_Identifiers);
		    }
		}
		else
		{
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_source_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
														  sIntegerIndex(vertex_id),
														  sIntegerIndex(layer)),
											  mutex_source_Identifiers);
		}

	    }
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    int init_robot_id = m_initial_arrangement.get_VertexOccupancy(vertex_id);

	    if (init_robot_id > 0)
	    {
		encoding_context.m_clause_generator->cast_Equality(solver,
										 sSpecifiedStateIdentifier(&vertex_occupancy,
													   sIntegerIndex(vertex_id),
													   sIntegerIndex(0)),
										 init_robot_id);
		encoding_context.m_bit_generator->cast_BitSet(solver,
									    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
												    sIntegerIndex(vertex_id),
												    sIntegerIndex(0)));
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		int goal_robot_id = m_goal_arrangement.get_VertexOccupancy(vertex_id);

		if (goal_robot_id > 0)
		{
		    encoding_context.m_bit_generator->cast_Equality(solver,
										  sSpecifiedStateIdentifier(&vertex_occupancy,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(N_Layers - 1)),
										  goal_robot_id);
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													sIntegerIndex(vertex_id),
													sIntegerIndex(N_Layers - 1)));
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
		    if (robot_IDs.size() > 1)
		    {
			sASSERT(false);
		    }
		    else
		    {
			encoding_context.m_clause_generator->cast_Equality(solver,
											 sSpecifiedStateIdentifier(&vertex_occupancy,
														   sIntegerIndex(vertex_id),
														   sIntegerIndex(N_Layers - 1)),
											 *robot_IDs.begin());
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&vertex_occupancy_by_water,
													    sIntegerIndex(vertex_id),
													    sIntegerIndex(N_Layers - 1)));
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
