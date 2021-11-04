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
/* encodings_derived.cpp / 0.22-robik_094                                     */
/*----------------------------------------------------------------------------*/
//
// Multi-robot path-finding encodings derived from the standard MDD encoding.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <limits.h>

#include <map>
#include <set>
#include <unordered_set>

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

    void sMultirobotInstance::to_Screen_WaterMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_WaterMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_AnoCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_AnoCNFsat(stdout, encoding_context, indent, verbose);
    }        

    
    void sMultirobotInstance::to_Screen_GAnoCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_GAnoCNFsat(stdout, encoding_context, indent, verbose);
    }            
    
    
    void sMultirobotInstance::to_Screen_MddStarCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MddStarCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }        


    void sMultirobotInstance::to_Screen_WaterMddCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_WaterMddCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }    

    
    void sMultirobotInstance::to_Screen_MddStarCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MddStarCNFsat(stdout, encoding_context, indent, verbose);
    }    


/*----------------------------------------------------------------------------*/

    sResult sMultirobotInstance::to_File_WaterMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_WaterMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_AnoCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_AnoCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_GAnoCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_GAnoCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }            

    
    sResult sMultirobotInstance::to_File_WaterMddCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_WaterMddCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);
	
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	{
	    if (unlink(filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }	    
	    return result;
	}
	return sRESULT_SUCCESS;
    }    
    
    
    sResult sMultirobotInstance::to_File_MddStarCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MddStarCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);
	
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO)
	{
	    if (unlink(filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }	    
	    return result;
	}
	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotInstance::to_File_MddStarCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MddStarCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/

    void sMultirobotInstance::to_Stream_WaterMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_WaterMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_WaterMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_WaterMddCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_WaterMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_WaterMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_AnoCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;	
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);

	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);	

	to_Stream_AnoCNFsat(fw, encoding_context, m_the_MDD, unified_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_AnoCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;	
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);

	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);	

	to_Memory_AnoCNFsat(solver, encoding_context, m_the_MDD, unified_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_GAnoCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;	
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);

	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);	

	to_Stream_GAnoCNFsat(fw, encoding_context, m_the_MDD, unified_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_GAnoCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;	
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);	

	to_Memory_GAnoCNFsat(solver, encoding_context, m_the_MDD, unified_MDD, indent, verbose);
    }                

    
    sResult sMultirobotInstance::to_Stream_WaterMddCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);
	reduce_MDD(unfolded_solution, m_the_extra_MDD, m_the_reduced_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO;
	}
	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_WaterMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_WaterMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_WaterMddCNFsat_avoid(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);
	reduce_MDD(unfolded_solution, m_the_extra_MDD, m_the_reduced_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO;
	}
	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_WaterMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_WaterMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }        

    
    sResult sMultirobotInstance::to_Stream_MddStarCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);
	reduce_MDD(unfolded_solution, m_the_extra_MDD, m_the_reduced_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO;
	}
	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddStarCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddStarCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MddStarCNFsat_avoid(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);
	reduce_MDD(unfolded_solution, m_the_extra_MDD, m_the_reduced_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO;
	}
	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddStarCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddStarCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }    

    
    void sMultirobotInstance::to_Stream_WaterMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	RobotMDD_vector unified_MDD;
	unify_MDD(MDD, unified_MDD);

	MDDIndices_vector MDD_Indices;	
	RobotMDDIndices_vector unified_MDD_Indices;

	construct_MDDIndices(MDD, MDD_Indices);
	construct_MDDIndices(unified_MDD, unified_MDD_Indices);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

	encoding_context.m_wet_vertex_occupancy_by_water_.resize(N_Layers + 1);
	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier wet_vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "wet_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
								   sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_wet_vertex_occupancy_by_water_[layer] = wet_vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_wet_vertex_occupancy_by_water_[layer]);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
										      
										      sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
													      sIntegerIndex(unified_MDD_Indices[layer][MDD[robot_id][layer][u]])));
		}
		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
								      "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								      sIntegerScope(0, 0));
		    encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;
		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											       total_Literal_cnt,
											       sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
											       prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		}
	    }
	}       
	if (!cardinality_Identifiers.empty())
	{
	    Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	}
	encoding_context.m_wet_edge_occupancy_by_water__.resize(N_Layers);
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_wet_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());

	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(unified_MDD[layer][u])->m_Neighbors;

		if (unified_MDD_Indices[layer + 1].find(unified_MDD[layer][u]) != unified_MDD_Indices[layer + 1].end())
		{
		    ++N_neighbors;
		}
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		{
		    if (unified_MDD_Indices[layer + 1].find((*neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier wet_edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "wet_edge_occupancy_by_water-" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_wet_edge_occupancy_by_water__[layer][u] = wet_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_wet_edge_occupancy_by_water__[layer][u]);
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;
		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;

		    if (MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]) != MDD_Indices[robot_id][layer + 1].end())
		    {
			++N_neighbors;
		    }
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			if (MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id) != MDD_Indices[robot_id][layer + 1].end())
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_vertex_Identifiers);
	    }
	}
*/	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;

		    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]);
		    Indices_map::const_iterator unified_mdd_index = unified_MDD_Indices[layer + 1].find(MDD[robot_id][layer][u]);

		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
													      sIntegerIndex(mdd_index->second)),
										      sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
													      sIntegerIndex(unified_mdd_index->second)));
		    if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(0)));
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														  sIntegerIndex(0)),
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],			
														  sIntegerIndex(mdd_index->second)));
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														  sIntegerIndex(0)),
											  sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
														  sIntegerIndex(0)));
		    }
		    int neighbor_index = 1;

		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));
			    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											      total_Literal_cnt,
											      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														      sIntegerIndex(neighbor_index)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(mdd_index->second)));

			    int unified_neighbor_index = 1;
			    for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
			    {
				if (unified_MDD_Indices[layer + 1].find((*unified_neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
				{
				    if ((*unified_neighbor)->m_target->m_id == (*neighbor)->m_target->m_id)
				    {
					Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																  sIntegerIndex(neighbor_index)),
													  sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
																  sIntegerIndex(unified_neighbor_index)));
				    }
				    ++unified_neighbor_index;
				}
			    }
			    neighbor_index++;
			}
		    }
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);
		}
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
										     total_Literal_cnt,
										     mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_occupancy_Identifiers);
		}
	    }
	}
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {						
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
*/	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													 sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													     sIntegerIndex(u)));
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

	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Writing");

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot Water MDD SAT encoding\n", sPRODUCT);
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


	printf("alpha 1\n");
	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											 
											 sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
														 sIntegerIndex(unified_MDD_Indices[layer][MDD[robot_id][layer][u]])));
		}
		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;

		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												  sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
												  prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	printf("alpha 2\n");
	s_GlobalPhaseStatistics.leave_Phase();
	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}

/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_vertex_Identifiers);
	    }
	}
*/

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector exclusion_vertex_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer].find(unified_MDD[layer][u]);

		    if (mdd_index != MDD_Indices[robot_id][layer].end())
		    {
			exclusion_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(mdd_index->second)));
		    }
		}
		if (!exclusion_vertex_Identifiers.empty())
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiExclusiveImplication(fw,
												       sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
															       sIntegerIndex(u)),
												       exclusion_vertex_Identifiers);
		}
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(unified_MDD[layer][u])->m_Neighbors;

		int unified_neighbor_index = 0;

		if (unified_MDD_Indices[layer + 1].find(unified_MDD[layer][u]) != unified_MDD_Indices[layer + 1].end())
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector exclusion_edge_Identifiers;
		    
		    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer].find(unified_MDD[layer][u]);
			
			if (mdd_index != MDD_Indices[robot_id][layer].end())
			{
			    if (MDD_Indices[robot_id][layer + 1].find(unified_MDD[layer][u]) != MDD_Indices[robot_id][layer + 1].end())
			    {
				exclusion_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][mdd_index->second], sIntegerIndex(0)));
			    }
			}
		    }
		    if (!exclusion_edge_Identifiers.empty())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiExclusiveImplication(fw,
													   sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
																   sIntegerIndex(unified_neighbor_index)),
													   exclusion_edge_Identifiers);
			
		    }
		    ++unified_neighbor_index;
		}
		for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
		{
		    if (unified_MDD_Indices[layer + 1].find((*unified_neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
		    {
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector exclusion_edge_Identifiers;

			for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
			{
			    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer].find(unified_MDD[layer][u]);

			    if (mdd_index != MDD_Indices[robot_id][layer].end())
			    {
				int neighbor_index = 0;

				if (MDD_Indices[robot_id][layer + 1].find(unified_MDD[layer][u]) != MDD_Indices[robot_id][layer + 1].end())
				{
				    ++neighbor_index;
				}
				for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
				{
				    if (MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id) != MDD_Indices[robot_id][layer + 1].end())
				    {
					if ((*unified_neighbor)->m_target->m_id == (*neighbor)->m_target->m_id)
					{
					    exclusion_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][mdd_index->second], sIntegerIndex(neighbor_index)));
					}
					++neighbor_index;
				    }
				}
			    }
			}
			if (!exclusion_edge_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiExclusiveImplication(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
																       sIntegerIndex(unified_neighbor_index)),
													       exclusion_edge_Identifiers);

			}
			++unified_neighbor_index;
		    }
		}
	    }
	}

	printf("alpha 3\n");
	s_GlobalPhaseStatistics.leave_Phase();
	s_GlobalPhaseStatistics.enter_Phase("Pregen 3");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;

		    Indices_map::const_iterator next_mdd_index = MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]);
		    Indices_map::const_iterator wet_mdd_index = unified_MDD_Indices[layer].find(MDD[robot_id][layer][u]);

		    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														 sIntegerIndex(u)),
											 sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
														 sIntegerIndex(wet_mdd_index->second)));
		    int neighbor_index = 0;
		    if (next_mdd_index != MDD_Indices[robot_id][layer + 1].end())
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(0)));
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(0)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],			
														     sIntegerIndex(next_mdd_index->second)));
			Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(0)),
											     sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
														     sIntegerIndex(0)));
			neighbor_index++;
		    }

		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));
			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															 sIntegerIndex(mdd_index->second)));
			    int unified_neighbor_index = 0;
			    if (unified_MDD_Indices[layer + 1].find(MDD[robot_id][layer][u]) != unified_MDD_Indices[layer + 1].end())
			    {
				if (MDD[robot_id][layer][u] == (*neighbor)->m_target->m_id)
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																 sIntegerIndex(neighbor_index)),
													 sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][wet_mdd_index->second],
																 sIntegerIndex(unified_neighbor_index)));
				}
				++unified_neighbor_index;
			    }
			    for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
			    {
				if (unified_MDD_Indices[layer + 1].find((*unified_neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
				{
				    if ((*unified_neighbor)->m_target->m_id == (*neighbor)->m_target->m_id)
				    {
					Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																     sIntegerIndex(neighbor_index)),
													     sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][wet_mdd_index->second],
																     sIntegerIndex(unified_neighbor_index)));
				    }
				    ++unified_neighbor_index;
				}
			    }
			    neighbor_index++;
			}
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);
		}
	    }
	}

	printf("alpha 4\n");
	s_GlobalPhaseStatistics.leave_Phase();
	s_GlobalPhaseStatistics.enter_Phase("Pregen 4");

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_edge_Identifiers;
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int unified_neighbor_index = 0;
		if (unified_MDD_Indices[layer + 1].find(unified_MDD[layer][u]) != unified_MDD_Indices[layer + 1].end())
		{
		    mutex_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u], sIntegerIndex(unified_neighbor_index)));
		    ++unified_neighbor_index;
		}
		const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(unified_MDD[layer][u])->m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
		{
		    if (unified_MDD_Indices[layer + 1].find((*unified_neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
		    {
			mutex_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u], sIntegerIndex(unified_neighbor_index)));
			++unified_neighbor_index;
		    }
		}
	    }
	}

	typedef std::map<int, sBitClauseGenerator::SpecifiedBitIdentifiers_vector, std::less<int> > SpecifiedBitIdentifiers_map;

	for (int layer = 1; layer <= N_Layers; ++layer)
	{
	    SpecifiedBitIdentifiers_map mutex_edge_Identifiers;
  
	    for (int u = 0; u < unified_MDD[layer - 1].size(); ++u)
	    {
		int unified_neighbor_index = 0;

		Indices_map::const_iterator mdd_index = unified_MDD_Indices[layer].find(unified_MDD[layer - 1][u]);
		if (mdd_index != unified_MDD_Indices[layer].end())
		{
		    mutex_edge_Identifiers[unified_MDD[layer - 1][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer - 1][u], sIntegerIndex(unified_neighbor_index)));
		    ++unified_neighbor_index;
		}
		const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(unified_MDD[layer - 1][u])->m_Neighbors;
		for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
		{
		    Indices_map::const_iterator target_mdd_index = unified_MDD_Indices[layer].find((*unified_neighbor)->m_target->m_id);

		    if (target_mdd_index != unified_MDD_Indices[layer].end())
		    {
			mutex_edge_Identifiers[(*unified_neighbor)->m_target->m_id].push_back(sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer - 1][u], sIntegerIndex(unified_neighbor_index)));
			++unified_neighbor_index;
		    }
		}
	    }
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_edge_Identifiers[unified_MDD[layer][u]]);
	    }
	}

	printf("alpha 5\n");
	s_GlobalPhaseStatistics.leave_Phase();
	s_GlobalPhaseStatistics.enter_Phase("Pregen 5");

/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_occupancy_Identifiers);
		}
	    }
	}
*/

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
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
	//------ alternative
	s_GlobalPhaseStatistics.leave_Phase();

	//	s_GlobalPhaseStatistics.leave_Phase();
    }


    void sMultirobotInstance::to_Memory_WaterMddCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	RobotMDD_vector unified_MDD;
	unify_MDD(MDD, unified_MDD);

	MDDIndices_vector MDD_Indices;	
	RobotMDDIndices_vector unified_MDD_Indices;

	construct_MDDIndices(MDD, MDD_Indices);
	construct_MDDIndices(unified_MDD, unified_MDD_Indices);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

	encoding_context.m_wet_vertex_occupancy_by_water_.resize(N_Layers + 1);
	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier wet_vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "wet_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
								   sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_wet_vertex_occupancy_by_water_[layer] = wet_vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_wet_vertex_occupancy_by_water_[layer]);
	}	
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    encoding_context.m_bit_generator->cast_Implication(solver,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
										     sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
													     sIntegerIndex(unified_MDD_Indices[layer][MDD[robot_id][layer][u]])));
		}
		if (!extra_MDD[robot_id][layer].empty())
		{
		    sASSERT(extra_MDD[robot_id][layer].size() == 1);

		    sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
								      "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								      sIntegerScope(0, 0));
		    encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);

		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			{
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			}
		    }
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;
		    for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
		    {
			if (!extra_MDD[robot_id][prev_layer].empty())
			{
			    prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			}
		    }
		    if (!prev_cardinality_Identifiers.empty())
		    {
			encoding_context.m_bit_generator->cast_MultiImplication(solver,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
											      prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		if (!extra_MDD[robot_id][layer].empty())
		{
		    cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		}
	    }
	}       
	if (!cardinality_Identifiers.empty())
	{
	    encoding_context.m_bit_generator->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	}
	encoding_context.m_wet_edge_occupancy_by_water__.resize(N_Layers);
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_wet_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());

	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(unified_MDD[layer][u])->m_Neighbors;

		if (unified_MDD_Indices[layer + 1].find(unified_MDD[layer][u]) != unified_MDD_Indices[layer + 1].end())
		{
		    ++N_neighbors;
		}
		for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		{
		    if (unified_MDD_Indices[layer + 1].find((*neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier wet_edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "wet_edge_occupancy_by_water-" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_wet_edge_occupancy_by_water__[layer][u] = wet_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_wet_edge_occupancy_by_water__[layer][u]);
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;
		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;

		    if (MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]) != MDD_Indices[robot_id][layer + 1].end())
		    {
			++N_neighbors;
		    }
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			if (MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id) != MDD_Indices[robot_id][layer + 1].end())
			{
			    ++N_neighbors;
			}
		    }
		    sIndexableBitIdentifier edge_occupancy_by_water__(&encoding_context.m_variable_store,
								      "edge_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(MDD[robot_id][layer][u]),
								      sIntegerScope(0, N_neighbors - 1));
		    encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u] = edge_occupancy_by_water__;
		    encoding_context.register_TranslateIdentifier(encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u]);
		}
	    }
	}
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;

		    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]);
		    Indices_map::const_iterator unified_mdd_index = unified_MDD_Indices[layer + 1].find(MDD[robot_id][layer][u]);

		    encoding_context.m_bit_generator->cast_Implication(solver,
										     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
													     sIntegerIndex(mdd_index->second)),
										     sSpecifiedBitIdentifier(&encoding_context.m_wet_vertex_occupancy_by_water_[layer],
													     sIntegerIndex(unified_mdd_index->second)));
		    if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(0)));
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														 sIntegerIndex(0)),
											 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],			
														 sIntegerIndex(mdd_index->second)));
			encoding_context.m_bit_generator->cast_Implication(solver,
											 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														 sIntegerIndex(0)),
											 sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
														 sIntegerIndex(0)));
		    }
		    int neighbor_index = 1;

		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));
			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(mdd_index->second)));

			    int unified_neighbor_index = 1;
			    for (sVertex::Neighbors_list::const_iterator unified_neighbor = vertex_Neighbors.begin(); unified_neighbor != vertex_Neighbors.end(); ++unified_neighbor)
			    {
				if (unified_MDD_Indices[layer + 1].find((*unified_neighbor)->m_target->m_id) != unified_MDD_Indices[layer + 1].end())
				{
				    if ((*unified_neighbor)->m_target->m_id == (*neighbor)->m_target->m_id)
				    {
					encoding_context.m_bit_generator->cast_Implication(solver,
													 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																 sIntegerIndex(neighbor_index)),
													 sSpecifiedBitIdentifier(&encoding_context.m_wet_edge_occupancy_by_water__[layer][u],
																 sIntegerIndex(unified_neighbor_index)));
				    }
				    ++unified_neighbor_index;
				}
			    }
			    neighbor_index++;
			}
		    }
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  mutex_target_Identifiers);
		}
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
										    mutex_vertex_Identifiers);
	}

	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		    {
			if (MDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_occupancy_Identifiers);
		}
	    }
	}
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {						
						encoding_context.m_bit_generator->cast_BiangleMutex(solver,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
					    }
					}
				    }
				}
			    }
			    ++neighbor_index;
			}
		    }
		}
	    }
	}
*/	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													    sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													    sIntegerIndex(u)));
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

    
    void sMultirobotInstance::to_Stream_AnoCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &sUNUSED(MDD), const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	//int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_unified_vertex_occupancy_by_water_.resize(N_Layers + 1);
	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers + 1);

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
							       "unified_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
							       sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_unified_vertex_occupancy_by_water_[layer] = vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_unified_vertex_occupancy_by_water_[layer]);
	}

	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_unified_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier unified_edge_occupancy_by_water__(&encoding_context.m_variable_store,
									  "unified_edge_occupancy_by_water-"  + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
									  sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_unified_edge_occupancy_by_water__[layer][u] = unified_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_unified_edge_occupancy_by_water__[layer][u]);
	    }
	}

	typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> SpecifiedBitIdentifiers_2d_vector;
		    
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    SpecifiedBitIdentifiers_2d_vector mutex_source_Identifiers;
	    mutex_source_Identifiers.resize(unified_MDD[layer + 1].size());
		
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
														  sIntegerIndex(neighbor_index)),
											  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
														  sIntegerIndex(v)));
			mutex_source_Identifiers[v].push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			neighbor_index++;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
													       sIntegerIndex(u)),
										       mutex_target_Identifiers);
		
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_target_Identifiers);   
	    }
	    for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_source_Identifiers[v]);
	    }
	}


	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
												       total_Literal_cnt,
												       sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
															       sIntegerIndex(neighbor_index)),
												       sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
															       sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
													 sIntegerIndex(u)));
		}
	    }

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
													     sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
													     sIntegerIndex(u)));
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
	    
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot ANO SAT encoding\n", sPRODUCT);
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
	    SpecifiedBitIdentifiers_2d_vector mutex_source_Identifiers;
	    mutex_source_Identifiers.resize(unified_MDD[layer + 1].size());
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		    int neighbor_index = 0;
		    for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
															 sIntegerIndex(v)));
			    mutex_source_Identifiers[v].push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			    neighbor_index++;
			}
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
	    }
	    for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
											    mutex_source_Identifiers[v]);
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
													  sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
																  sIntegerIndex(neighbor_index)),
													  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
																  sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
													    sIntegerIndex(u)));
		}
	    }
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
														sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
														sIntegerIndex(u)));
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
    }    
   

    void sMultirobotInstance::to_Memory_AnoCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &sUNUSED(MDD), const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	//int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_unified_vertex_occupancy_by_water_.resize(N_Layers + 1);
	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers + 1);

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
							       "unified_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
							       sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_unified_vertex_occupancy_by_water_[layer] = vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_unified_vertex_occupancy_by_water_[layer]);
	}

	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_unified_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier unified_edge_occupancy_by_water__(&encoding_context.m_variable_store,
									  "unified_edge_occupancy_by_water-"  + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
									  sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_unified_edge_occupancy_by_water__[layer][u] = unified_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_unified_edge_occupancy_by_water__[layer][u]);
	    }
	}

	typedef std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> SpecifiedBitIdentifiers_2d_vector;	

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    SpecifiedBitIdentifiers_2d_vector mutex_source_Identifiers;
	    mutex_source_Identifiers.resize(unified_MDD[layer + 1].size());

	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			
			encoding_context.m_bit_generator->cast_Implication(solver,
									   sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
												   sIntegerIndex(neighbor_index)),
									   sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
												   sIntegerIndex(v)));
			mutex_source_Identifiers[v].push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));			
			neighbor_index++;
		    }
		}
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
									sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
												sIntegerIndex(u)),
									mutex_target_Identifiers);	
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									  mutex_target_Identifiers);   
	    }
	    for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
	    {
		bool goaling = true;
		for (int some_robot_id = 1; some_robot_id <= N_Robots; ++some_robot_id)
		{
		    if (*m_goal_specification.get_RobotGoal(some_robot_id).begin() == unified_MDD[layer + 1][v])
		    {
			goaling = true;
			break;
		    }
		}
		if (goaling)
		{
		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									      mutex_source_Identifiers[v]);
		}
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    encoding_context.m_bit_generator->cast_BiangleMutex(solver,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
														sIntegerIndex(neighbor_index)),
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
								  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
											  sIntegerIndex(u)));
		}
	    }

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
								      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
											      sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
								      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
											      sIntegerIndex(u)));
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
	for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
	{
	    bool unset = true;
	    
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
	    
		if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		{
		    unset = false;
		    break;
		}
	    }
	    if (unset)
	    {
		encoding_context.m_bit_generator->cast_BitUnset(solver,
							      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
										      sIntegerIndex(u)));		
	    }
	}	
    }


    void sMultirobotInstance::to_Stream_GAnoCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &sUNUSED(MDD), const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	//int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_unified_vertex_occupancy_by_water_.resize(N_Layers + 1);
	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers + 1);

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
							       "unified_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
							       sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_unified_vertex_occupancy_by_water_[layer] = vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_unified_vertex_occupancy_by_water_[layer]);
	}

	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_unified_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier unified_edge_occupancy_by_water__(&encoding_context.m_variable_store,
									  "unified_edge_occupancy_by_water-"  + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
									  sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_unified_edge_occupancy_by_water__[layer][u] = unified_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_unified_edge_occupancy_by_water__[layer][u]);
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			
			Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
											  total_Literal_cnt,
											  sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
														  sIntegerIndex(neighbor_index)),
											  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
														  sIntegerIndex(v)));
			neighbor_index++;
		    }
		}
		Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
										       total_Literal_cnt,
										       sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
													       sIntegerIndex(u)),
										       mutex_target_Identifiers);
		
		Clause_cnt += encoding_context.m_bit_generator->count_AllMutexConstraint(aux_Variable_cnt,
											 total_Literal_cnt,
											 mutex_target_Identifiers);   
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
												       total_Literal_cnt,
												       sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
															       sIntegerIndex(neighbor_index)),
												       sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
															       sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										 total_Literal_cnt,
										 sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
													 sIntegerIndex(u)));
		}
	    }

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
													     sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->count_BitSet(aux_Variable_cnt,
										     total_Literal_cnt,
										     sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
													     sIntegerIndex(u)));
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
	    
	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot GANO SAT encoding\n", sPRODUCT);
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
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		    int neighbor_index = 0;
		    for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));

			    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
															 sIntegerIndex(neighbor_index)),
												 sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
															 sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AllMutexConstraint(fw,
												mutex_target_Identifiers);   
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
													  sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
																  sIntegerIndex(neighbor_index)),
													  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
																  sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
													    sIntegerIndex(u)));
		}
	    }
	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
														sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
														sIntegerIndex(u)));
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
    }    
   

    void sMultirobotInstance::to_Memory_GAnoCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &sUNUSED(MDD), const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	//int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_unified_vertex_occupancy_by_water_.resize(N_Layers + 1);
	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers + 1);

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
							       "unified_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
							       sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_unified_vertex_occupancy_by_water_[layer] = vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_unified_vertex_occupancy_by_water_[layer]);
	}

	encoding_context.m_unified_edge_occupancy_by_water__.resize(N_Layers);

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    encoding_context.m_unified_edge_occupancy_by_water__[layer].resize(unified_MDD[layer].size());
	    
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int N_neighbors = 0;
		
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			++N_neighbors;
		    }
		}
		sIndexableBitIdentifier unified_edge_occupancy_by_water__(&encoding_context.m_variable_store,
									  "unified_edge_occupancy_by_water-"  + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
									  sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_unified_edge_occupancy_by_water__[layer][u] = unified_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_unified_edge_occupancy_by_water__[layer][u]);
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;
		
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u], sIntegerIndex(neighbor_index)));
			
			encoding_context.m_bit_generator->cast_Implication(solver,
									   sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
												   sIntegerIndex(neighbor_index)),
									   sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer + 1],
												   sIntegerIndex(v)));
			neighbor_index++;
		    }
		}
		encoding_context.m_bit_generator->cast_MultiImplication(solver,
									sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
												sIntegerIndex(u)),
									mutex_target_Identifiers);
		
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									  mutex_target_Identifiers);   
	    }
	}

	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		int neighbor_index = 0;
		for (int v = 0; v < unified_MDD[layer + 1].size(); ++v)
		{
		    if (m_environment.is_Adjacent(unified_MDD[layer][u], unified_MDD[layer + 1][v]) || unified_MDD[layer][u] == unified_MDD[layer + 1][v])
		    {
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int vv = 0; vv < unified_MDD[layer].size(); ++vv)
			    {
				if (unified_MDD[layer + 1][v] == unified_MDD[layer][vv])
				{
				    encoding_context.m_bit_generator->cast_BiangleMutex(solver,
											sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
														sIntegerIndex(neighbor_index)),
											sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														sIntegerIndex(vv)));
				}
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < unified_MDD[0].size(); ++u)
	    {
		if (unified_MDD[0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
								  sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[0],
											  sIntegerIndex(u)));
		}
	    }

	    switch (m_goal_type)
	    {
	    case GOAL_TYPE_ARRANGEMENT:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    if (unified_MDD[N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
								      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
											      sIntegerIndex(u)));
		    }
		}
		break;
	    }
	    case GOAL_TYPE_SPECIFICATION:
	    {
		for (int u = 0; u < unified_MDD[N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);
		    
		    if (unified_MDD[N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
								      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[N_Layers],
											      sIntegerIndex(u)));
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
    }            
    

    
    void sMultirobotInstance::to_Stream_MddStarCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
//	s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddStarCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddStarCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_MddStarCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
//	s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddStarCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddStarCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }        


    void sMultirobotInstance::to_Stream_MddStarCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	bool bound = false;

	MDDIndices_vector MDD_Indices;       
	construct_MDDIndices(MDD, MDD_Indices);
	     	
	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	if (m_ratio > 0.0)
	{
	    int add_up = floor(encoding_context.m_max_total_cost * (m_ratio - 1.0));
	    extra_cost += add_up;

	    if (encoding_context.m_max_total_cost + extra_cost < mdd_depth * N_Robots)
	    {
		bound = true;
	    }
	}
	if (m_ratio < 0.0)
	{
	    bound = false;	    
	}			
	
	/*
	printf("Bound:%d (%d x %d)\n", bound, encoding_context.m_max_total_cost + extra_cost, mdd_depth * N_Robots);
	getchar();
	*/
	
	if (bound)
	{
	    encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    
	    if (bound)
	    {
		encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    }	    	    
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		if (bound)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			sASSERT(extra_MDD[robot_id][layer].size() == 1);
			
			sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
									  "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
									  sIntegerScope(0, 0));
			encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
			encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);
		    }
		}
	    }
	}

	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	    
        if (bound)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int layer = 0; layer <= N_Layers; ++layer)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		    }
		}
	    }       
	    if (!cardinality_Identifiers.empty())
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	    }
	}
//	int N_cnf_Variables = encoding_context.m_variable_store.get_Last_CNFVariable() + aux_Variable_cnt - 1;

#ifdef sDEBUG
	/*
	fprintf(fw, "c %s : multirobot MDD* SAT encoding\n", sPRODUCT);
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
	*/
#endif
//	fprintf(fw, "p cnf %d %d\n", N_cnf_Variables, Clause_cnt);
	fprintf(fw, "p cnf 0 0\n");

	if (bound)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int layer = 0; layer <= N_Layers; ++layer)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			sASSERT(extra_MDD[robot_id][layer].size() == 1);
			
			for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
			{
			    if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
												     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
												     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			    }
			}
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;
			
			for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
			{
			    if (!extra_MDD[robot_id][prev_layer].empty())
			    {
				prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			    }
			}
			if (!prev_cardinality_Identifiers.empty())
			{
			    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
												      sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
												      prev_cardinality_Identifiers);
			}
		    }
		}
	    }
	    if (!cardinality_Identifiers.empty())
	    {
		Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{		    
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;		    
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(mdd_index->second)));
			}
		    }
		    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]);
		    if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
		    {
			outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(mdd_index->second)));
		    }
		    
		    /*
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));			    
			}
		    }
		    */
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      outgo_target_Identifiers);		    
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_occupancy_Identifiers;
	    mutex_occupancy_Identifiers.resize(N_Vertices);

	    std::set<int> nonempty_Vertices;

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		    nonempty_Vertices.insert(MDD[robot_id][layer][u]);
		}
	    }
	    for (std::set<int>::const_iterator nonempty_vertex = nonempty_Vertices.begin(); nonempty_vertex != nonempty_Vertices.end(); ++nonempty_vertex)
	    {
		if (mutex_occupancy_Identifiers[*nonempty_vertex].size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers[*nonempty_vertex]);
		}
	    }
	    /*
	    for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	    {
		if (mutex_occupancy_Identifiers[vertex_id].size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers[vertex_id]);
		}
	    }
	    */
	}	    
/*	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{

		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;		    
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    sASSERT(MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][mdd_index->second]);

			    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
			    {
				if (other_robot_id != robot_id)
				{
				    Indices_map::const_iterator other_mdd_index = MDD_Indices[other_robot_id][layer].find((*neighbor)->m_target->m_id);

				    if (other_mdd_index != MDD_Indices[other_robot_id][layer].end())
				    {					
					Clause_cnt += encoding_context.m_bit_generator->generate_TriangleMutex(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																       sIntegerIndex(u)),
													       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																       sIntegerIndex(mdd_index->second)),
													       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																       sIntegerIndex(other_mdd_index->second)));
				    }
				}
			    }
			}
		    }			    
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{			    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (other_robot_id != robot_id)
				    {
					for (int vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {

						Clause_cnt += encoding_context.m_bit_generator->generate_TriangleMutex(fw,
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																	       sIntegerIndex(u)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	       sIntegerIndex(v)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	       sIntegerIndex(vv)));		
					    }
					}
				    }
				}
			    }
			}
		    }
		}
	    }
	}
*/
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													    sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			Clause_cnt += encoding_context.m_bit_generator->generate_BitSet(fw,
											sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
														sIntegerIndex(u)));
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
	
	//------ alternative
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.leave_Phase();
    }


    void sMultirobotInstance::to_Memory_MddStarCNFsat(sSATSolver_Type *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	bool bound = false;

	MDDIndices_vector MDD_Indices;       
	construct_MDDIndices(MDD, MDD_Indices);
	     	
	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	if (m_ratio > 0.0)
	{
	    int add_up = floor(encoding_context.m_max_total_cost * (m_ratio - 1.0));
	    extra_cost += add_up;

	    if (encoding_context.m_max_total_cost + extra_cost < mdd_depth * N_Robots)
	    {
		bound = true;
	    }
	}
	if (m_ratio < 0.0)
	{
	    bound = false;	    
	}			
	
	if (bound)
	{
	    encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    
	    if (bound)
	    {
		encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    }	    	    
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		if (bound)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			sASSERT(extra_MDD[robot_id][layer].size() == 1);
			
			sIndexableBitIdentifier vertex_water_cardinality_(&encoding_context.m_variable_store,
									  "vertex_water_cardinality-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
									  sIntegerScope(0, 0));
			encoding_context.m_vertex_water_cardinality_[robot_id][layer] = vertex_water_cardinality_;
			encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_water_cardinality_[robot_id][layer]);
		    }
		}
	    }
	}

	sBitClauseGenerator::SpecifiedBitIdentifiers_vector cardinality_Identifiers;
	    
        if (bound)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int layer = 0; layer <= N_Layers; ++layer)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
		    }
		}
	    }       
	    if (!cardinality_Identifiers.empty())
	    {
		Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	    }
	}

	if (bound)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int layer = 0; layer <= N_Layers; ++layer)
		{
		    if (!extra_MDD[robot_id][layer].empty())
		    {
			sASSERT(extra_MDD[robot_id][layer].size() == 1);
			
			for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
			{
			    if (extra_MDD[robot_id][layer][0] != MDD[robot_id][layer][u])
			    {
				encoding_context.m_bit_generator->cast_Implication(solver,
										   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
										   sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)));
			    }
			}
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector prev_cardinality_Identifiers;
			
			for (int prev_layer = 0; prev_layer < layer; ++prev_layer)
			{
			    if (!extra_MDD[robot_id][prev_layer].empty())
			    {
				prev_cardinality_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][prev_layer], sIntegerIndex(0)));
			    }
			}
			if (!prev_cardinality_Identifiers.empty())
			{
			    encoding_context.m_bit_generator->cast_MultiImplication(solver,
												  sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
												  prev_cardinality_Identifiers);
			}
		    }
		}
	    }
	    if (!cardinality_Identifiers.empty())
	    {
		encoding_context.m_bit_generator->cast_Cardinality(solver, cardinality_Identifiers, extra_cost);
	    }
	}
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{		    
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    const sVertex::Neighbors_list &vertex_Neighbors = m_environment.get_Vertex(MDD[robot_id][layer][u])->m_Neighbors;		    
		    for (sVertex::Neighbors_list::const_iterator neighbor = vertex_Neighbors.begin(); neighbor != vertex_Neighbors.end(); ++neighbor)
		    {
			Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find((*neighbor)->m_target->m_id);
			if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(mdd_index->second)));
			}
		    }
		    Indices_map::const_iterator mdd_index = MDD_Indices[robot_id][layer + 1].find(MDD[robot_id][layer][u]);
		    if (mdd_index != MDD_Indices[robot_id][layer + 1].end())
		    {
			outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(mdd_index->second)));
		    }
		    
		    /*
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));			    
			}
		    }
		    */
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  outgo_target_Identifiers);		    
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												mutex_vertex_Identifiers);
	    }
	}

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_occupancy_Identifiers;
	    mutex_occupancy_Identifiers.resize(N_Vertices);

	    std::set<int> nonempty_Vertices;

	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    mutex_occupancy_Identifiers[MDD[robot_id][layer][u]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
		    nonempty_Vertices.insert(MDD[robot_id][layer][u]);
		}
	    }
	    for (std::set<int>::const_iterator nonempty_vertex = nonempty_Vertices.begin(); nonempty_vertex != nonempty_Vertices.end(); ++nonempty_vertex)
	    {
		if (mutex_occupancy_Identifiers[*nonempty_vertex].size() > 1)
		{
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_occupancy_Identifiers[*nonempty_vertex]);
		}
	    }
	}	    	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int u = 0; u < MDD[robot_id][0].size(); ++u)
	    {
		if (MDD[robot_id][0][u] == m_initial_arrangement.get_RobotLocation(robot_id))
		{
		    encoding_context.m_bit_generator->cast_BitSet(solver,
										sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][0],
													sIntegerIndex(u)));
		}
	    }
	}
	switch (m_goal_type)
	{
	case GOAL_TYPE_ARRANGEMENT:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    if (MDD[robot_id][N_Layers][u] == m_goal_arrangement.get_RobotLocation(robot_id))
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
													    sIntegerIndex(u)));
		    }
		}
	    }
	    break;
	}
	case GOAL_TYPE_SPECIFICATION:
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	    {
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(robot_id).begin())
		    {
			encoding_context.m_bit_generator->cast_BitSet(solver,
								      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
											      sIntegerIndex(u)));
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
	
	//------ alternative
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.leave_Phase();
    }        

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc
