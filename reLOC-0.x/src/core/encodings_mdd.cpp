/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.20-kruh                               */
/*                                                                            */
/*                      (C) Copyright 2018 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* encodings_mdd.cpp / 0.20-kruh_045                                          */
/*----------------------------------------------------------------------------*/
//
// Multi-robot path-finding encodings based on
// MDDs and calculating cost-optimal solutions.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
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

    void sMultirobotInstance::to_Screen_MddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_GMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_GMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_GEMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_GEMddCNFsat(stdout, encoding_context, indent, verbose);
    }        


    void sMultirobotInstance::to_Screen_MddCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MddCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_RelaxedMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RelaxedMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_TokenMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_TokenMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_TokenEmptyMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_TokenEmptyMddCNFsat(stdout, encoding_context, indent, verbose);
    }    


    void sMultirobotInstance::to_Screen_PermutationMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_PermutationMddCNFsat(stdout, encoding_context, indent, verbose);
    }        

    
    void sMultirobotInstance::to_Screen_MddPlusCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MddPlusCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }    


    void sMultirobotInstance::to_Screen_MddPlusPlusCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MddPlusPlusCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }

    
    void sMultirobotInstance::to_Screen_MmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MmddCNFsat(stdout, encoding_context, indent, verbose);
    }

    
    void sMultirobotInstance::to_Screen_MmddCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MmddCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_RelaxedMmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RelaxedMmddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_TokenMmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_TokenMmddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_TokenEmptyMmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_TokenEmptyMmddCNFsat(stdout, encoding_context, indent, verbose);
    }        


    void sMultirobotInstance::to_Screen_PermutationMmddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_PermutationMmddCNFsat(stdout, encoding_context, indent, verbose);
    }    

    
    void sMultirobotInstance::to_Screen_MmddPlusCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MmddPlusCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }    


    void sMultirobotInstance::to_Screen_MmddPlusPlusCNFsat_avoid(sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	to_Stream_MmddPlusPlusCNFsat_avoid(stdout, encoding_context, unfolded_solution, indent, verbose);
    }        


    void sMultirobotInstance::to_Screen_RXMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RXMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_NoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_NoMddCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_RXNoMddCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_RXNoMddCNFsat(stdout, encoding_context, indent, verbose);
    }

    
    void sMultirobotInstance::to_Screen_MddPlusCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MddPlusCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_MmddPlusCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MmddPlusCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_MddPlusPlusCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MddPlusPlusCNFsat(stdout, encoding_context, indent, verbose);
    }


    void sMultirobotInstance::to_Screen_LMddPlusPlusCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_LMddPlusPlusCNFsat(stdout, encoding_context, indent, verbose);
    }
    

    void sMultirobotInstance::to_Screen_MmddPlusPlusCNFsat(sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	to_Stream_MmddPlusPlusCNFsat(stdout, encoding_context, indent, verbose);
    }    
    

/*----------------------------------------------------------------------------*/
    
    sResult sMultirobotInstance::to_File_MddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_GMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_GMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }

    
    sResult sMultirobotInstance::to_File_GEMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_GEMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotInstance::to_File_RelaxedMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RelaxedMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_TokenMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_TokenMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_TokenEmptyMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_TokenEmptyMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotInstance::to_File_PermutationMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PermutationMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }            

    
    sResult sMultirobotInstance::to_File_MddCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MddCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
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

    
    sResult sMultirobotInstance::to_File_MddPlusCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MddPlusCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
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


    sResult sMultirobotInstance::to_File_MddPlusPlusCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MddPlusPlusCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
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


    sResult sMultirobotInstance::to_File_MmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_RelaxedMmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RelaxedMmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_TokenMmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_TokenMmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_TokenEmptyMmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_TokenEmptyMmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }            


    sResult sMultirobotInstance::to_File_PermutationMmddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_PermutationMmddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }        
    

    sResult sMultirobotInstance::to_File_MmddCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MmddCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);
	
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	{
	    if (unlink(filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }
	    return result;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MmddPlusCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MmddPlusCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);
	
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	{
	    if (unlink(filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }	    
	    return result;
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MmddPlusPlusCNFsat_avoid(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	sResult result;
	
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	if (sFAILED(result = to_Stream_MmddPlusPlusCNFsat_avoid(fw, encoding_context, unfolded_solution, indent, verbose)))
	{
	    fclose(fw);
	    return result;
	}
	fclose(fw);
	
	if (result == sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO)
	{
	    if (unlink(filename.c_str()) < 0)
	    {
		return sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR;
	    }	    
	    return result;
	}
	return sRESULT_SUCCESS;
    }        


    sResult sMultirobotInstance::to_File_RXMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RXMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_NoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_NoMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_RXNoMddCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_RXNoMddCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MddPlusCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MddPlusCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_MmddPlusCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MmddPlusCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    

    
    sResult sMultirobotInstance::to_File_MddPlusPlusCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MddPlusPlusCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_File_LMddPlusPlusCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_LMddPlusPlusCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    
    

    sResult sMultirobotInstance::to_File_MmddPlusPlusCNFsat(const sString &filename, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	FILE *fw;
	if ((fw = fopen(filename.c_str(), "w")) == NULL)
	{
	    return sMULTIROBOT_CNF_OPEN_ERROR;
	}
	to_Stream_MmddPlusPlusCNFsat(fw, encoding_context, indent, verbose);
	fclose(fw);

	return sRESULT_SUCCESS;
    }    


/*----------------------------------------------------------------------------*/
    
    void sMultirobotInstance::to_Stream_MddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_GMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_GMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_GMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_GEMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);

	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_GEMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, unified_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_GEMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, unified_MDD, indent, verbose);
	}
    }        


    void sMultirobotInstance::to_Memory_MddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_GMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_GMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_GMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }

    
    void sMultirobotInstance::to_Memory_GEMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	RobotMDD_vector unified_MDD;
	unify_MDD(m_the_MDD, unified_MDD);		

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_GEMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, unified_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_GEMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, unified_MDD, indent, verbose);
	}
    }            


    void sMultirobotInstance::to_Stream_RelaxedMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_RelaxedMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_RelaxedMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_TokenMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_TokenMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_TokenMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_TokenEmptyMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_TokenEmptyMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_TokenEmptyMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }        


    void sMultirobotInstance::to_Stream_PermutationMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_PermutationMddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_PermutationMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }
    

    void sMultirobotInstance::to_Memory_RelaxedMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_RelaxedMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_RelaxedMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_TokenMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_TokenMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_TokenMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_TokenEmptyMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_TokenEmptyMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_TokenEmptyMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }    


    void sMultirobotInstance::to_Memory_PermutationMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_PermutationMddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_PermutationMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }                


    sResult sMultirobotInstance::to_Stream_MddCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Stream_MddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MddCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Memory_MddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }    

  
    sResult sMultirobotInstance::to_Stream_MddPlusCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Stream_MddPlusCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddPlusCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MddPlusCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Memory_MddPlusCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddPlusCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotInstance::to_Stream_MddPlusPlusCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Stream_MddPlusPlusCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddPlusPlusCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MddPlusPlusCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
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
	    to_Memory_MddPlusPlusCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddPlusPlusCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_reduced_MDD, m_the_reduced_extra_MDD, indent, verbose);
	}
	return sRESULT_SUCCESS;
    }    
    

    void sMultirobotInstance::to_Stream_MmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_MmddCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_MmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_MmddCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_RelaxedMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_RelaxedMmddCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Stream_TokenMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_TokenMmddCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }

    
    void sMultirobotInstance::to_Stream_TokenEmptyMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_TokenEmptyMmddCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }        


    void sMultirobotInstance::to_Stream_PermutationMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_PermutationMmddCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }        

    
    void sMultirobotInstance::to_Memory_RelaxedMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_RelaxedMmddCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }        

    
    void sMultirobotInstance::to_Memory_TokenMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_TokenMmddCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_TokenEmptyMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_TokenEmptyMmddCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }            
    

    void sMultirobotInstance::to_Memory_PermutationMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_PermutationMmddCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }        
    
    
    sResult sMultirobotInstance::to_Stream_MmddCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Stream_MmddCNFsat(fw, encoding_context, m_the_reduced_MDD, indent, verbose);
		
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MmddCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Memory_MmddCNFsat(solver, encoding_context, m_the_reduced_MDD, indent, verbose);
		
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotInstance::to_Stream_MmddPlusCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Stream_MmddPlusCNFsat(fw, encoding_context, m_the_reduced_MDD, indent, verbose);
	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MmddPlusCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Memory_MmddPlusCNFsat(solver, encoding_context, m_the_reduced_MDD, indent, verbose);
	
	return sRESULT_SUCCESS;
    }    


    sResult sMultirobotInstance::to_Stream_MmddPlusPlusCNFsat_avoid(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Stream_MmddCNFsat(fw, encoding_context, m_the_reduced_MDD, indent, verbose);
	
	return sRESULT_SUCCESS;
    }


    sResult sMultirobotInstance::to_Memory_MmddPlusPlusCNFsat_avoid(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const Arrangements_vector &unfolded_solution, const sString &indent, bool verbose)
    {
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	reduce_MDD(unfolded_solution, m_the_MDD, m_the_reduced_MDD);

	if (!check_Connectivity(m_the_reduced_MDD))
	{
	    return sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO;
	}
	to_Memory_MmddCNFsat(solver, encoding_context, m_the_reduced_MDD, indent, verbose);
	
	return sRESULT_SUCCESS;
    }        


    void sMultirobotInstance::to_Stream_RXMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_RXMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_RXMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_RXMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_NoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_NoMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }    


    void sMultirobotInstance::to_Stream_RXNoMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_RXMddCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_RXNoMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;

	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_NoMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	// s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_RXMddCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_MddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }
				}
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																    sIntegerIndex(neighbor_index)),
													    biangular_Identifiers);
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}				
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 biangular_Identifiers);
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
    }


    void sMultirobotInstance::to_Stream_GMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {				
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{					    
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
														       total_Literal_cnt,
														       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	       sIntegerIndex(neighbor_index)),
														       conflict_edge_Identifiers);
				    }
				}
			    }
				
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}

				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																    sIntegerIndex(neighbor_index)),
													    biangular_Identifiers);
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot GMDD SAT encoding\n", sPRODUCT);
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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
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
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
															  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																		  sIntegerIndex(neighbor_index)),
															  conflict_edge_Identifiers);
				    }
				}
			    }
			    
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }			    
				}

				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				
				if (!biangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
												 sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															 sIntegerIndex(neighbor_index)),
												 biangular_Identifiers);
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
    }


    void sMultirobotInstance::to_Stream_GEMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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
	
	encoding_context.m_unified_vertex_occupancy_by_water_.resize(N_Layers + 1);

	MDDIndices_vector MDD_Indices;
	construct_MDDIndices(MDD, MDD_Indices);

	RobotMDDIndices_vector unified_MDD_Indices;
	construct_MDDIndices(unified_MDD, unified_MDD_Indices);

	for (int layer = 0; layer <= N_Layers; ++layer)
	{
	    sIndexableBitIdentifier unified_vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								       "unified_vertex_occupancy_by_water-" + sInt_32_to_String(layer),
								       sIntegerScope(0, unified_MDD[layer].size() - 1));
	    encoding_context.m_unified_vertex_occupancy_by_water_[layer] = unified_vertex_occupancy_by_water_;
	    encoding_context.register_TranslateIdentifier(encoding_context.m_unified_vertex_occupancy_by_water_[layer]);
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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
													  total_Literal_cnt,
													  sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													  prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_Implication(aux_Variable_cnt,
										      total_Literal_cnt,
										      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
										      sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
													      sIntegerIndex(unified_MDD_Indices[layer][MDD[robot_id][layer][u]])));
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
									  "unified_edge_occupancy_by_water-" + sInt_32_to_String(layer) + "_" + sInt_32_to_String(unified_MDD[layer][u]),
									  sIntegerScope(0, N_neighbors - 1));
		encoding_context.m_unified_edge_occupancy_by_water__[layer][u] = unified_edge_occupancy_by_water__;
		encoding_context.register_TranslateIdentifier(encoding_context.m_unified_edge_occupancy_by_water__[layer][u]);		
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {				
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{					    
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
														       total_Literal_cnt,
														       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	       sIntegerIndex(neighbor_index)),
														       conflict_edge_Identifiers);
				    }
				}
			    }
				
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

	for (int layer = 0; layer < N_Layers; ++layer)
	{		
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
		
		for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[unified_MDD[layer][u]].m_Conflicts.begin();
		     conflict != m_environment.m_Vertices[unified_MDD[layer][u]].m_Conflicts.end(); ++conflict)
		{
		    for (int k = 0; k < m_robustness; ++k)
		    {
			if (layer - k >= 0)
			{
			    for (int vv = 0; vv < unified_MDD[layer - k].size(); ++vv)
			    {
				if (*conflict == unified_MDD[layer - k][vv] && *conflict != unified_MDD[layer][u])
				{
				    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer - k],
											    sIntegerIndex(vv)));
				}
			    }
			}
		    }
		}
		if (!biangular_Identifiers.empty())
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
											    total_Literal_cnt,
											    sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														    sIntegerIndex(u)),
											    biangular_Identifiers);
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
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int k = 0; k < m_robustness; ++k)
			    {
				if (layer - k >= 0)
				{
				    for (int vv = 0; vv < unified_MDD[layer - k].size(); ++vv)
				    {
					if (unified_MDD[layer + 1][v] == unified_MDD[layer - k][vv])
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer - k],
												    sIntegerIndex(vv)));
					}
				    }
				}
			    }
			    if (!biangular_Identifiers.empty())
			    {
				Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													total_Literal_cnt,
													sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
																sIntegerIndex(neighbor_index)),
													biangular_Identifiers);
			    }
			}
			++neighbor_index;
		    }
		}
	    }
	}
	
/*
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
*/

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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot GEMDD SAT encoding\n", sPRODUCT);
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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_Implication(fw,
											 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
											 sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														 sIntegerIndex(unified_MDD_Indices[layer][MDD[robot_id][layer][u]])));
		}
	    }
	}	
	
	if (!cardinality_Identifiers.empty())
	{
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


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
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
															  sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																		  sIntegerIndex(neighbor_index)),
															  conflict_edge_Identifiers);
				    }
				}
			    }
			    
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
/*
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
	    
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
*/
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

	for (int layer = 0; layer < N_Layers; ++layer)
	{		
	    for (int u = 0; u < unified_MDD[layer].size(); ++u)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
		
		for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[unified_MDD[layer][u]].m_Conflicts.begin();
		     conflict != m_environment.m_Vertices[unified_MDD[layer][u]].m_Conflicts.end(); ++conflict)
		{
		    for (int k = 0; k < m_robustness; ++k)
		    {
			if (layer - k >= 0)
			{
			    for (int vv = 0; vv < unified_MDD[layer - k].size(); ++vv)
			    {
				if (*conflict == unified_MDD[layer - k][vv] && *conflict != unified_MDD[layer][u])
				{
				    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer - k],
											    sIntegerIndex(vv)));
				}
			    }
			}
		    }
		}
		if (!biangular_Identifiers.empty())
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
											       sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer],
														       sIntegerIndex(u)),
											       biangular_Identifiers);
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
			sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
			if (unified_MDD[layer][u] != unified_MDD[layer + 1][v])
			{
			    for (int k = 0; k < m_robustness; ++k)
			    {
				if (layer - k >= 0)
				{
				    for (int vv = 0; vv < unified_MDD[layer - k].size(); ++vv)
				    {
					if (unified_MDD[layer + 1][v] == unified_MDD[layer - k][vv])
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_unified_vertex_occupancy_by_water_[layer - k],
												    sIntegerIndex(vv)));
					}
				    }
				}
			    }
			    if (!biangular_Identifiers.empty())
			    {
				Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
													   sSpecifiedBitIdentifier(&encoding_context.m_unified_edge_occupancy_by_water__[layer][u],
																   sIntegerIndex(neighbor_index)),
													   biangular_Identifiers);
			    }
			}
			++neighbor_index;
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
    }        


    void sMultirobotInstance::to_Memory_MddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									  mutex_vertex_Identifiers);
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
		    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}

				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     biangular_Identifiers);
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


    void sMultirobotInstance::to_Memory_GMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {				
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
													sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																sIntegerIndex(neighbor_index)),
													conflict_edge_Identifiers);
				    }
				}
			    }
			    
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									  mutex_vertex_Identifiers);
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
		    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					}
					}
				    }				    
				}				
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     biangular_Identifiers);
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


    void sMultirobotInstance::to_Memory_GEMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const RobotMDD_vector &unified_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	MDDIndices_vector MDD_Indices;
	construct_MDDIndices(MDD, MDD_Indices);

	RobotMDDIndices_vector unified_MDD_Indices;
	construct_MDDIndices(unified_MDD, unified_MDD_Indices);		

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sVertex::Neighbors_list &Neighbors = m_environment.m_Vertices[MDD[robot_id][layer][u]].m_Neighbors;
			    for (sVertex::Neighbors_list::const_iterator neighbor = Neighbors.begin(); neighbor != Neighbors.end(); ++neighbor)
			    {				
				if ((*neighbor)->m_target->m_id == MDD[robot_id][layer + 1][v])
				{
				    if (!(*neighbor)->m_edge->m_Conflicts.empty())
				    {
					sBitClauseGenerator::SpecifiedBitIdentifiers_vector conflict_edge_Identifiers;
				 
					sEdge::Conflicts_vector &Conflicts = (*neighbor)->m_edge->m_Conflicts;
					for (sEdge::Conflicts_vector::const_iterator conflict = Conflicts.begin(); conflict != Conflicts.end(); ++conflict)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (robot_id != other_robot_id)
						{
						    for (int uu = 0; uu < MDD[other_robot_id][layer].size(); ++uu)
						    {
							int neighbor_index = 0;
							for (int vv = 0; vv < MDD[other_robot_id][layer + 1].size(); ++vv)
							{
							    if (   m_environment.is_Adjacent(MDD[other_robot_id][layer][uu], MDD[other_robot_id][layer + 1][vv])
								|| MDD[other_robot_id][layer][uu] == MDD[other_robot_id][layer + 1][vv])
							    {
								if (   (conflict->m_x_ID == MDD[other_robot_id][layer][uu] && conflict->m_y_ID == MDD[other_robot_id][layer + 1][vv])
								    || (conflict->m_y_ID == MDD[other_robot_id][layer][uu] && conflict->m_x_ID == MDD[other_robot_id][layer + 1][vv]))
								{
								    conflict_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][uu],
																sIntegerIndex(neighbor_index)));
								}
								neighbor_index++;
							    }
							}
						    }
						}
					    }
					}
					encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
													sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																sIntegerIndex(neighbor_index)),
													conflict_edge_Identifiers);
				    }
				}
			    }
			    
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
									  mutex_vertex_Identifiers);
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
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector biangular_Identifiers;
		    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														    sIntegerIndex(vv)));
							}
						    }
						}
					}
					}
				    }				    
				}				
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														sIntegerIndex(vv)));
						    }
						}
					    }
					}
				    }
				}
				if (!biangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     biangular_Identifiers);
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
    

    void sMultirobotInstance::to_Stream_RelaxedMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	#ifdef sVERBOSE
	{
	    printf("Goal specification\n");
	    m_goal_specification.to_Screen_brief();
	}
	#endif

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
	fprintf(fw, "c %s : multirobot relaxed MDD SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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


    void sMultirobotInstance::to_Stream_TokenMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	#ifdef sVERBOSE
	{
	    printf("Goal specification\n");
	    m_goal_specification.to_Screen_brief();
	}
	#endif

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);

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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}		
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
													   total_Literal_cnt,
													   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																   sIntegerIndex(neighbor_index)),
													   complementary_edge_Identifiers);
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															   sIntegerIndex(neighbor_index)));
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
	fprintf(fw, "c %s : multirobot token MDD SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    		
		    
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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}	       
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}


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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
													      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																      sIntegerIndex(neighbor_index)),
													      complementary_edge_Identifiers);
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															      sIntegerIndex(neighbor_index)));
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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


    void sMultirobotInstance::to_Stream_TokenEmptyMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	#ifdef sVERBOSE
	{
	    printf("Goal specification\n");
	    m_goal_specification.to_Screen_brief();
	}
	#endif

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);

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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}		
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;				
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
										    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImpliedImplication(aux_Variable_cnt,
														     total_Literal_cnt,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
				                                                                              total_Literal_cnt,	
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */				    
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																    sIntegerIndex(neighbor_index)),
													    biangular_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
	fprintf(fw, "c %s : multirobot token empty MDD SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    		
		    
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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}	       
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}


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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);
					
					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImpliedImplication(fw,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																       sIntegerIndex(neighbor_index)),
													       biangular_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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


    void sMultirobotInstance::to_Stream_PermutationMddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);

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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}		
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}
	
	for (int layer = 0; layer < N_Layers; ++layer)
	{
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   complementary_edge_Identifiers);
				}
				/*
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															   sIntegerIndex(neighbor_index)));
				}
				*/
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
	fprintf(fw, "c %s : multirobot permutation MDD SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    		
		    
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
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      mutex_target_Identifiers);

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{		    
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);		    
		}	       
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}
	
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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      complementary_edge_Identifiers);
				}
				/*
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															      sIntegerIndex(neighbor_index)));
				}
				*/
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
    

    void sMultirobotInstance::to_Memory_RelaxedMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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


    void sMultirobotInstance::to_Memory_TokenMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    				

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}	       		
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)),
											    complementary_edge_Identifiers);
				}
				else
				{
				    encoding_context.m_bit_generator->cast_BitUnset(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
													    sIntegerIndex(neighbor_index)));
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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


    void sMultirobotInstance::to_Memory_TokenEmptyMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    				

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}	       		
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);					

					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    encoding_context.m_bit_generator->cast_MultiImpliedImplication(solver,
												   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															   sIntegerIndex(neighbor_index)),
												   complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
				                                                                              total_Literal_cnt,	
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */				    
				}
				else
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     biangular_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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


    void sMultirobotInstance::to_Memory_PermutationMddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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
		std::vector<sBitClauseGenerator::SpecifiedBitIdentifiers_vector> mutex_source_Identifiers;		
		mutex_source_Identifiers.resize(N_Vertices);		    				

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_target_Identifiers;

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    mutex_source_Identifiers[MDD[robot_id][layer + 1][v]].push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)));			    
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		{
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_source_Identifiers[MDD[robot_id][layer + 1][v]]);
		}	       		
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
												    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															    sIntegerIndex(neighbor_index)),
												    complementary_edge_Identifiers);
				}
				/*
				else
				{
				    encoding_context.m_bit_generator->cast_BitUnset(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
													    sIntegerIndex(neighbor_index)));
				}
				*/
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
    
    
    void sMultirobotInstance::to_Stream_MmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
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
    }


    void sMultirobotInstance::to_Memory_MmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_vertex_Identifiers);
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
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id) // non-anonymous
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
            /*   
	    for (int robot_id = 1; robot_id <= N_Robots; ++robot_id) // anonymous
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector goal_Identifiers;
		
		for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
		{
		    sASSERT(m_goal_specification.get_RobotGoal(robot_id).size() == 1);

		    for (int some_robot_id = 1; some_robot_id <= N_Robots; ++some_robot_id)
		    {
		
			if (MDD[robot_id][N_Layers][u] == *m_goal_specification.get_RobotGoal(some_robot_id).begin())
			{
			    goal_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers],
									       sIntegerIndex(u)));
			}
		    }
		}
		encoding_context.m_bit_generator->cast_Disjunction(solver, goal_Identifiers);
	    }
            */
	    break;
	}
	default:
	{
	    sASSERT(false);
	    break;
	}
	}
    }    


    void sMultirobotInstance::to_Stream_RelaxedMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
						*/
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot relaxed MMDD SAT encoding\n", sPRODUCT);
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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
						*/
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
    }


    void sMultirobotInstance::to_Stream_TokenMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);					

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
													   total_Literal_cnt,
													   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																   sIntegerIndex(neighbor_index)),
													   complementary_edge_Identifiers);
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_BitUnset(aux_Variable_cnt,
												   total_Literal_cnt,
												   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															   sIntegerIndex(neighbor_index)));
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
						*/
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot token MMDD SAT encoding\n", sPRODUCT);
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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
													      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																      sIntegerIndex(neighbor_index)),
													      complementary_edge_Identifiers);
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_BitUnset(fw,
												      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															      sIntegerIndex(neighbor_index)));
				}

			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
						*/
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
    }        


    void sMultirobotInstance::to_Stream_TokenEmptyMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;				
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);					

					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImpliedImplication(aux_Variable_cnt,
														     total_Literal_cnt,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
				                                                                              total_Literal_cnt,	
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */				    
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																    sIntegerIndex(neighbor_index)),
													    biangular_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
						*/
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot token empty MMDD SAT encoding\n", sPRODUCT);
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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;				
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImpliedImplication(fw,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiImplication(fw,
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */
				}
				else
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																       sIntegerIndex(neighbor_index)),
													       biangular_Identifiers);				    
				}

			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
						*/
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
    }        
    
    
    void sMultirobotInstance::to_Stream_PermutationMmddCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiNegativeImplication(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   complementary_edge_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->count_BiangleMutex(aux_Variable_cnt,
														   total_Literal_cnt,
														   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	   sIntegerIndex(neighbor_index)),
														   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	   sIntegerIndex(vv)));
						*/
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot permutation MMDD SAT encoding\n", sPRODUCT);
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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiNegativeImplication(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      complementary_edge_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						Clause_cnt += encoding_context.m_bit_generator->generate_BiangleMutex(fw,
														      sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	      sIntegerIndex(neighbor_index)),
														      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	      sIntegerIndex(vv)));
						*/
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
    }        

    
    void sMultirobotInstance::to_Memory_RelaxedMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						encoding_context.m_bit_generator->cast_BiangleMutex(solver,
						sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
						sIntegerIndex(neighbor_index)),
						sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
						sIntegerIndex(vv)));
						*/
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


    void sMultirobotInstance::to_Memory_TokenMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														    sIntegerIndex(neighbor_index)),
											    complementary_edge_Identifiers);
				}
				else
				{
				    encoding_context.m_bit_generator->cast_BitUnset(solver,
										    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
													    sIntegerIndex(neighbor_index)));
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						encoding_context.m_bit_generator->cast_BiangleMutex(solver,
						sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
						sIntegerIndex(neighbor_index)),
						sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
						sIntegerIndex(vv)));
						*/
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


    void sMultirobotInstance::to_Memory_TokenEmptyMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers, complementary_node_Identifiers, biangular_Identifiers;				
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv = -1;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}
					sASSERT(vv >= 0);

					if (vv < MDD[other_robot_id][layer].size())
					{
					    biangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
					    
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer], sIntegerIndex(vv)));
//							    complementary_node_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer + 1], sIntegerIndex(w)));
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    sASSERT(!complementary_node_Identifiers.empty());
				    encoding_context.m_bit_generator->cast_MultiImpliedImplication(solver,
												   sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															   sIntegerIndex(neighbor_index)),
												   complementary_node_Identifiers, complementary_edge_Identifiers);
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
				                                                                              total_Literal_cnt,	
														     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
																	     sIntegerIndex(neighbor_index)),
														     complementary_edge_Identifiers);
				    */				    
				}
				else
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     biangular_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						encoding_context.m_bit_generator->cast_BiangleMutex(solver,
						sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
						sIntegerIndex(neighbor_index)),
						sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
						sIntegerIndex(vv)));
						*/
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


    void sMultirobotInstance::to_Memory_PermutationMmddCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
									       sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
												       sIntegerIndex(neighbor_index)),
									       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
												       sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
									    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
												    sIntegerIndex(u)),
									    mutex_target_Identifiers);
		    
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										  mutex_vertex_Identifiers);
	    }
	}

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
				sBitClauseGenerator::SpecifiedBitIdentifiers_vector complementary_edge_Identifiers;
				
				for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
				{
				    if (robot_id != other_robot_id)
				    {
					int vv;
					
					for (vv = 0; vv < MDD[other_robot_id][layer].size(); ++vv)
					{
					    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer][vv])
					    {
						break;
					    }
					}

					if (vv < MDD[other_robot_id][layer].size())
					{
					    int other_neighbor_index = 0;
					    for (int w = 0; w < MDD[other_robot_id][layer + 1].size(); ++w)
					    {					    
						if (m_environment.is_Adjacent(MDD[other_robot_id][layer][vv], MDD[other_robot_id][layer + 1][w]) || MDD[other_robot_id][layer][vv] == MDD[other_robot_id][layer + 1][w])
						{
						    if (MDD[other_robot_id][layer][vv] != MDD[other_robot_id][layer + 1][w])
						    {
							if (MDD[other_robot_id][layer + 1][w] == MDD[robot_id][layer][u])
							{
							    complementary_edge_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[other_robot_id][layer][vv], sIntegerIndex(other_neighbor_index)));
							}
						    }
						    ++other_neighbor_index;
						}
					    }
					}
				    }
				}
				if (!complementary_edge_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiNegativeImplication(solver,
												    sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
															    sIntegerIndex(neighbor_index)),
												    complementary_edge_Identifiers);
				}
			    }
			    neighbor_index++;
			}
		    }
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
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_occupancy_Identifiers);
		}
	    }
	}

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
						/*
						encoding_context.m_bit_generator->cast_BiangleMutex(solver,
						sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
						sIntegerIndex(neighbor_index)),
						sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
						sIntegerIndex(vv)));
						*/
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
    

    void sMultirobotInstance::to_Stream_RXMddCNFsat(FILE                              *fw,
						    sMultirobotEncodingContext_CNFsat &encoding_context,
						    int                                sUNUSED(extra_cost),
						    int                                mdd_depth,
						    const MDD_vector                  &MDD,
						    const MDD_vector                  &extra_MDD,
						    const sString                     &sUNUSED(indent),
						    bool                               sUNUSED(verbose))
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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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
/*
	if (!cardinality_Identifiers.empty())
	{
	    Clause_cnt += encoding_context.m_bit_generator->count_Cardinality(aux_Variable_cnt,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	}
*/
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MDD SAT encoding\n", sPRODUCT);
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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
/*
	if (!cardinality_Identifiers.empty())
	{
	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
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
    }


    void sMultirobotInstance::to_Memory_RXMddCNFsat(Glucose::Solver                   *solver,
						    sMultirobotEncodingContext_CNFsat &encoding_context,
						    int                                sUNUSED(extra_cost),
						    int                                mdd_depth,
						    const MDD_vector                  &MDD,
						    const MDD_vector                  &extra_MDD,
						    const sString                     &sUNUSED(indent),
						    bool                               sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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
/*
	if (!cardinality_Identifiers.empty())
	{
	    encoding_context.m_bit_generator->cast_Cardinality(solver,total_Literal_cnt, cardinality_Identifiers, extra_cost);
	}
*/
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											    mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AllMutexConstraint(solver,
											mutex_vertex_Identifiers);
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


/*----------------------------------------------------------------------------*/
    
    void sMultirobotInstance::to_Stream_MddPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddPlusCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddPlusCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_MddPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddPlusCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddPlusCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }    


    void sMultirobotInstance::to_Stream_MmddPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_MmddPlusCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_MmddPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_MmddPlusCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }    


    void sMultirobotInstance::to_Stream_MddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_MddPlusPlusCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_MddPlusPlusCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Stream_LMddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_LimitedMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);	
	construct_DisplacementMDD(encoding_context.m_max_total_cost, m_the_LMDD, extra_cost, m_the_extra_LMDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Stream_LMddPlusPlusCNFsat(fw, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_LMDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Stream_LMddPlusPlusCNFsat(fw, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_LMDD, m_the_extra_MDD, indent, verbose);
	}
    }    


    void sMultirobotInstance::to_Memory_MddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_MDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_MddPlusPlusCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_MddPlusPlusCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_extra_MDD, indent, verbose);
	}
    }


    void sMultirobotInstance::to_Memory_LMddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	int extra_cost;
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	int mdd_depth = construct_LimitedMDD(encoding_context.m_max_total_cost, m_the_MDD, extra_cost, m_the_extra_MDD);	
	construct_DisplacementMDD(encoding_context.m_max_total_cost, m_the_LMDD, extra_cost, m_the_extra_LMDD);
	//s_GlobalPhaseStatistics.leave_Phase();

	if (encoding_context.m_extra_cost >= 0)
	{
	    to_Memory_LMddPlusPlusCNFsat(solver, encoding_context, encoding_context.m_extra_cost, mdd_depth, m_the_MDD, m_the_LMDD, m_the_extra_MDD, indent, verbose);
	}
	else
	{
	    encoding_context.m_extra_cost = extra_cost;
	    to_Memory_LMddPlusPlusCNFsat(solver, encoding_context, extra_cost, mdd_depth, m_the_MDD, m_the_LMDD, m_the_extra_MDD, indent, verbose);
	}
    }        


    void sMultirobotInstance::to_Stream_MmddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Stream_MmddPlusPlusCNFsat(fw, encoding_context, m_the_MDD, indent, verbose);
    }


    void sMultirobotInstance::to_Memory_MmddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const sString &indent, bool verbose)
    {
	//	s_GlobalPhaseStatistics.enter_Phase("MDD build");
	--encoding_context.m_N_Layers;
	construct_MakespanMDD(encoding_context.m_N_Layers, m_the_MDD);
	
	//s_GlobalPhaseStatistics.leave_Phase();

	to_Memory_MmddPlusPlusCNFsat(solver, encoding_context, m_the_MDD, indent, verbose);
    }        


    void sMultirobotInstance::to_Stream_MddPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											       total_Literal_cnt,
											       mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_occupancy_Identifiers);
		}
	    }
	}

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
	fprintf(fw, "c %s : multirobot MDD+ SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												    mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												      mutex_occupancy_Identifiers);
		}
	    }
	}

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
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.leave_Phase();
    }


    void sMultirobotInstance::to_Memory_MddPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_edge_occupancy_by_water__[robot_id].resize(N_Layers);

	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		encoding_context.m_edge_occupancy_by_water__[robot_id][layer].resize(MDD[robot_id][layer].size());

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    int N_neighbors = 0;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_occupancy_Identifiers);
		}
	    }
	}

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


    void sMultirobotInstance::to_Stream_MmddPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_target_Identifiers);   
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											       total_Literal_cnt,
											       mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_occupancy_Identifiers);
		}
	    }
	}

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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MMDD+ SAT encoding\n", sPRODUCT);
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

		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												      mutex_target_Identifiers);   
		}
		
		Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												  mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												      mutex_occupancy_Identifiers);
		}
	    }
	}

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
    }


    void sMultirobotInstance::to_Memory_MmddPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
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

		    int neighbor_index = 0;
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    mutex_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u], sIntegerIndex(neighbor_index)));

			    encoding_context.m_bit_generator->cast_Implication(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_edge_occupancy_by_water__[robot_id][layer][u],
														     sIntegerIndex(neighbor_index)),
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)));
			    neighbor_index++;
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    encoding_context.m_bit_generator->cast_MultiImplication(solver,
											  sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														  sIntegerIndex(u)),
											  mutex_target_Identifiers);

		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_target_Identifiers);   
		}
		encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												mutex_vertex_Identifiers);
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_occupancy_Identifiers);
		}
	    }
	}

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
    

    void sMultirobotInstance::to_Stream_MddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
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
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   outgo_target_Identifiers);
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_vertex_Identifiers);
	}
*/
	
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{					
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));							    
							}
						    }
						}
					    }
					}
				    }
				}
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {					
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));

							/*
							Clause_cnt += encoding_context.m_bit_generator->count_TriangleMutex(aux_Variable_cnt,
															    total_Literal_cnt,
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																		    sIntegerIndex(u)),
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																		    sIntegerIndex(v)),
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																		    sIntegerIndex(vv)));
							*/
						    }
						}
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																    sIntegerIndex(v)),
													    triangular_Identifiers);				    
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiTriangleMutex(aux_Variable_cnt,
													     total_Literal_cnt,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																     sIntegerIndex(u)),
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																     sIntegerIndex(v)),
													     triangular_Identifiers);
				    */
				}
			    }
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
	fprintf(fw, "c %s : multirobot MDD++ SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));			    
			}
		    }
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
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												mutex_vertex_Identifiers);
	    
	}
*/
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												      mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{					
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}				
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {					
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));
/*
						Clause_cnt += encoding_context.m_bit_generator->generate_TriangleMutex(fw,
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																	       sIntegerIndex(u)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	       sIntegerIndex(v)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	       sIntegerIndex(vv)));		
*/
						    }
						}
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiTriangleMutex(fw,
														sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																	sIntegerIndex(u)),
														sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	sIntegerIndex(v)),
														triangular_Identifiers);		
				    */
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																       sIntegerIndex(v)),
													       triangular_Identifiers);
				}
			    }
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
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.leave_Phase();
    }


    void sMultirobotInstance::to_Stream_LMddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &LMDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	MDDIndices_vector LMDD_Indices;
	construct_MDDIndices(LMDD, LMDD_Indices);

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_occupancy_by_gas_.resize(N_Robots + 1);	
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_occupancy_by_gas_[robot_id].resize(N_Layers + 1);	    
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
		
		sIndexableBitIdentifier vertex_occupancy_by_gas_(&encoding_context.m_variable_store,
								 "vertex_occupancy_by_gas-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, LMDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer] = vertex_occupancy_by_gas_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer]);		

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
			Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
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
	
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector gas_displacement_Identifiers;			
		    int robot_size = m_initial_arrangement.m_robot_Sizes[robot_id];

		    for (int dr = 0; dr < robot_size; ++dr)
		    {
			for (int dc = 0; dc < robot_size; ++dc)
			{
			    int next_vertex_id = m_environment.get_GridNeighborVertexID(MDD[robot_id][layer][u], dr, dc);
			    if (next_vertex_id >= 0)
			    {
				int mdd_index = LMDD_Indices[robot_id][layer][next_vertex_id];
				gas_displacement_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(mdd_index)));
			    }
			}
		    }		    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiConjunctiveImplication(aux_Variable_cnt,
												      total_Literal_cnt,
												      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
												      gas_displacement_Identifiers);
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   outgo_target_Identifiers);
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												 total_Literal_cnt,
												 mutex_vertex_Identifiers);
	    }
	}

/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											     total_Literal_cnt,
											     mutex_vertex_Identifiers);
	}
*/
	/*
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												     total_Literal_cnt,
												     mutex_occupancy_Identifiers);
		}
	    }
	}
	*/
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < LMDD[robot_id][layer].size(); ++u)
		    {
			if (LMDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
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
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
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
						triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
						sIntegerIndex(vv)));
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																    sIntegerIndex(v)),
													    triangular_Identifiers);				    
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
	fprintf(fw, "c %s : multirobot LMDD++ SAT encoding\n", sPRODUCT);
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

	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 1");

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
			Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_water_cardinality_[robot_id][layer], sIntegerIndex(0)),
													     prev_cardinality_Identifiers);
		    }
		}
	    }
	}
	//	s_GlobalPhaseStatistics.leave_Phase();
	//	s_GlobalPhaseStatistics.enter_Phase("Pregen 2");

	if (!cardinality_Identifiers.empty())
	{
//	    printf("----> Cardinality: %d, %d, %d <----\n", cardinality_Identifiers.size(), extra_cost, encoding_context.m_max_total_cost);
	    Clause_cnt += encoding_context.m_bit_generator->generate_Cardinality(fw, cardinality_Identifiers, extra_cost);
	}


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector gas_displacement_Identifiers;			
		    int robot_size = m_initial_arrangement.m_robot_Sizes[robot_id];

		    for (int dr = 0; dr < robot_size; ++dr)
		    {
			for (int dc = 0; dc < robot_size; ++dc)
			{
			    int next_vertex_id = m_environment.get_GridNeighborVertexID(MDD[robot_id][layer][u], dr, dc);
			    if (next_vertex_id >= 0)
			    {
				int mdd_index = LMDD_Indices[robot_id][layer][next_vertex_id];
				gas_displacement_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(mdd_index)));
			    }
			}
		    }
		    Clause_cnt += encoding_context.m_bit_generator->generate_MultiConjunctiveImplication(fw,
													 sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
													 gas_displacement_Identifiers);
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));			    
			}
		    }
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

/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												mutex_vertex_Identifiers);
	    
	}
*/
	/*
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
												      mutex_occupancy_Identifiers);
		}
	    }
	}
	*/
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < LMDD[robot_id][layer].size(); ++u)
		    {
			if (LMDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
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
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
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
						triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
													 sIntegerIndex(vv)));
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
													       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																       sIntegerIndex(v)),
													       triangular_Identifiers);
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
    

    void sMultirobotInstance::to_Memory_MddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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


	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;

		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
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
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
											     mutex_vertex_Identifiers);
	}
*/
	
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
												    mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));
							/*	
						        encoding_context.m_bit_generator->cast_TriangleMutex(solver,
												     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
															     sIntegerIndex(u)),
												     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
															     sIntegerIndex(v)),
												     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
															     sIntegerIndex(vv)));
							*/
						    }
						}
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)),
											     triangular_Identifiers);
				    /*
				    encoding_context.m_bit_generator->cast_MultiTriangleMutex(solver,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)),
											      triangular_Identifiers);
				    */
				}
			    }
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


    void sMultirobotInstance::to_Memory_LMddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, int extra_cost, int mdd_depth, const MDD_vector &MDD, const MDD_vector &LMDD, const MDD_vector &extra_MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = mdd_depth;

	MDDIndices_vector LMDD_Indices;
	construct_MDDIndices(LMDD, LMDD_Indices);	

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
	encoding_context.m_vertex_occupancy_by_gas_.resize(N_Robots + 1);	
	encoding_context.m_vertex_water_cardinality_.resize(N_Robots + 1);
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	//	s_GlobalPhaseStatistics.enter_Phase("Counting");

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    encoding_context.m_vertex_occupancy_by_gas_[robot_id].resize(N_Layers + 1);	    
	    encoding_context.m_vertex_water_cardinality_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);

		sIndexableBitIdentifier vertex_occupancy_by_gas_(&encoding_context.m_variable_store,
								 "vertex_occupancy_by_gas-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								 sIntegerScope(0, LMDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer] = vertex_occupancy_by_gas_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer]);				

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
			encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
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

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector gas_displacement_Identifiers;			
		    int robot_size = m_initial_arrangement.m_robot_Sizes[robot_id];

		    for (int dr = 0; dr < robot_size; ++dr)
		    {
			for (int dc = 0; dc < robot_size; ++dc)
			{
			    int next_vertex_id = m_environment.get_GridNeighborVertexID(MDD[robot_id][layer][u], dr, dc);
			    if (next_vertex_id >= 0)
			    {
				int mdd_index = LMDD_Indices[robot_id][layer][next_vertex_id];
				gas_displacement_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(mdd_index)));
			    }
			}
		    }
		    encoding_context.m_bit_generator->cast_MultiConjunctiveImplication(solver,
										       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)),
										       gas_displacement_Identifiers);
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
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
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
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
											     mutex_vertex_Identifiers);
	}
*/
	
	for (int vertex_id = 0; vertex_id < N_Vertices; ++vertex_id)
	{
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_occupancy_Identifiers;

		for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
		{
		    for (int u = 0; u < LMDD[robot_id][layer].size(); ++u)
		    {
			if (LMDD[robot_id][layer][u] == vertex_id)
			{
			    mutex_occupancy_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_gas_[robot_id][layer], sIntegerIndex(u)));
			}
		    }
		}
		if (mutex_occupancy_Identifiers.size() > 1)
		{
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
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
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
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
						triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
													 sIntegerIndex(vv)));
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)),
											     triangular_Identifiers);
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
    

    void sMultirobotInstance::to_Stream_MmddPlusPlusCNFsat(FILE *fw, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int Clause_cnt = 0;
	int aux_Variable_cnt = 0;
	int total_Literal_cnt = 0;

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
		    mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer], sIntegerIndex(u)));	
	    
		    Clause_cnt += encoding_context.m_bit_generator->count_MultiImplication(aux_Variable_cnt,
											   total_Literal_cnt,
											   sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														   sIntegerIndex(u)),
											   outgo_target_Identifiers);
		}
		Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											       total_Literal_cnt,
											       mutex_vertex_Identifiers);
	    }
	}
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
											   total_Literal_cnt,
											   mutex_vertex_Identifiers);
	}
*/
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
		    Clause_cnt += encoding_context.m_bit_generator->count_AdaptiveAllMutexConstraint(aux_Variable_cnt,
												   total_Literal_cnt,
												   mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));
							}
						    }
					    }
					    }
					}
				    }				    
				}				
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));
							/*
							Clause_cnt += encoding_context.m_bit_generator->count_TriangleMutex(aux_Variable_cnt,
															    total_Literal_cnt,
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																		    sIntegerIndex(u)),
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																		    sIntegerIndex(v)),
															    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																		    sIntegerIndex(vv)));
							*/							
						    }
						}
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiBiangleMutex(aux_Variable_cnt,
													    total_Literal_cnt,
													    sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																    sIntegerIndex(v)),
													    triangular_Identifiers);				    
/*				    
				    Clause_cnt += encoding_context.m_bit_generator->count_MultiTriangleMutex(aux_Variable_cnt,
													     total_Literal_cnt,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																     sIntegerIndex(u)),
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																     sIntegerIndex(v)),
													     triangular_Identifiers);
*/
				}
			    }
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

#ifdef sDEBUG
	fprintf(fw, "c %s : multirobot MMDD++ SAT encoding\n", sPRODUCT);
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

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    sBitClauseGenerator::SpecifiedBitIdentifiers_vector outgo_target_Identifiers;

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));

			}
		    }
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
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
											      mutex_vertex_Identifiers);
	}
*/
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
		    Clause_cnt += encoding_context.m_bit_generator->generate_AdaptiveAllMutexConstraint(fw,
													mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;			    
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));
							}
						    }
						}					    
					    }
					}
				    }				    
				}
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));
						/*
						Clause_cnt += encoding_context.m_bit_generator->generate_TriangleMutex(fw,
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																	       sIntegerIndex(u)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	       sIntegerIndex(v)),
														       sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																	       sIntegerIndex(vv)));
						*/
						    }
						}
					    }					    
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiBiangleMutex(fw,
														sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	sIntegerIndex(v)),
														triangular_Identifiers);				    
				    /*
				    Clause_cnt += encoding_context.m_bit_generator->generate_MultiTriangleMutex(fw,
														sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																	sIntegerIndex(u)),
														sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																	sIntegerIndex(v)),
														triangular_Identifiers);
				    */
				}				    
			    }
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
    }


    void sMultirobotInstance::to_Memory_MmddPlusPlusCNFsat(Glucose::Solver *solver, sMultirobotEncodingContext_CNFsat &encoding_context, const MDD_vector &MDD, const sString &sUNUSED(indent), bool sUNUSED(verbose))
    {
	encoding_context.switchTo_AdvancedGeneratingMode();	

	int N_Vertices = m_environment.get_VertexCount();
	int N_Robots = m_initial_arrangement.get_RobotCount();
	int N_Layers = encoding_context.m_N_Layers;

	encoding_context.m_vertex_occupancy_by_water_.resize(N_Robots + 1);
//	encoding_context.m_edge_occupancy_by_water__.resize(N_Robots + 1);

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    encoding_context.m_vertex_occupancy_by_water_[robot_id].resize(N_Layers + 1);
	    for (int layer = 0; layer <= N_Layers; ++layer)
	    {
		sIndexableBitIdentifier vertex_occupancy_by_water_(&encoding_context.m_variable_store,
								   "vertex_occupancy_by_water-" + sInt_32_to_String(robot_id) + "_" + sInt_32_to_String(layer),
								   sIntegerScope(0, MDD[robot_id][layer].size() - 1));
		encoding_context.m_vertex_occupancy_by_water_[robot_id][layer] = vertex_occupancy_by_water_;
		encoding_context.register_TranslateIdentifier(encoding_context.m_vertex_occupancy_by_water_[robot_id][layer]);
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

		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    outgo_target_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1], sIntegerIndex(v)));
			}
		    }
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
/*
	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    sBitClauseGenerator::SpecifiedBitIdentifiers_vector mutex_vertex_Identifiers;
		    
	    for (int u = 0; u < MDD[robot_id][N_Layers].size(); ++u)
	    {
		mutex_vertex_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][N_Layers], sIntegerIndex(u)));
	    }
	    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
											   mutex_vertex_Identifiers);
	}
*/
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
		    encoding_context.m_bit_generator->cast_AdaptiveAllMutexConstraint(solver,
										      mutex_occupancy_Identifiers);
		}
	    }
	}

	for (int robot_id = 1; robot_id <= N_Robots; ++robot_id)
	{
	    for (int layer = 0; layer < N_Layers; ++layer)
	    {
		for (int u = 0; u < MDD[robot_id][layer].size(); ++u)
		{
		    for (int v = 0; v < MDD[robot_id][layer + 1].size(); ++v)
		    {
			if (m_environment.is_Adjacent(MDD[robot_id][layer][u], MDD[robot_id][layer + 1][v]) || MDD[robot_id][layer][u] == MDD[robot_id][layer + 1][v])
			{
			    sBitClauseGenerator::SpecifiedBitIdentifiers_vector triangular_Identifiers;
			    if (MDD[robot_id][layer][u] != MDD[robot_id][layer + 1][v])
			    {
				for (sVertex::VertexIDs_vector::const_iterator conflict = m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.begin();
				     conflict != m_environment.m_Vertices[MDD[robot_id][layer + 1][v]].m_Conflicts.end(); ++conflict)
				{
				    for (int k = 0; k < m_robustness; ++k)
				    {
					if (layer - k >= 0)
					{
					    for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					    {
						if (other_robot_id != robot_id)
						{
						    for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						    {
							if (*conflict == MDD[other_robot_id][layer - k][vv])
							{
							    triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														     sIntegerIndex(vv)));
							}
						    }
						}
					    }
					}
				    }				    
				}
				for (int k = 0; k < m_robustness; ++k)
				{
				    if (layer - k >= 0)
				    {
					for (int other_robot_id = 1; other_robot_id <= N_Robots; ++other_robot_id)
					{
					    if (other_robot_id != robot_id)
					    {
						for (int vv = 0; vv < MDD[other_robot_id][layer - k].size(); ++vv)
						{
						    if (MDD[robot_id][layer + 1][v] == MDD[other_robot_id][layer - k][vv])
						    {
							triangular_Identifiers.push_back(sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer - k],
														 sIntegerIndex(vv)));
							/*
							encoding_context.m_bit_generator->cast_TriangleMutex(solver,
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
																     sIntegerIndex(u)),
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
																     sIntegerIndex(v)),								   
													     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[other_robot_id][layer],
																     sIntegerIndex(vv)));
							*/
						    }
						}
					    }
					}
				    }
				}
				if (!triangular_Identifiers.empty())
				{
				    encoding_context.m_bit_generator->cast_MultiBiangleMutex(solver,
											     sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														     sIntegerIndex(v)),
											     triangular_Identifiers);				    
				    /*
				    encoding_context.m_bit_generator->cast_MultiTriangleMutex(solver,
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer],
														      sIntegerIndex(u)),
											      sSpecifiedBitIdentifier(&encoding_context.m_vertex_occupancy_by_water_[robot_id][layer + 1],
														      sIntegerIndex(v)),
											      triangular_Identifiers);
				    */
				}
			    }
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
    
    
/*----------------------------------------------------------------------------*/

} // namespace sReloc
