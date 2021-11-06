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
/* result.h / 0.22-robik_095                                                  */
/*----------------------------------------------------------------------------*/

#ifndef __RESULT_H__
#define __RESULT_H__

#include "types.h"


/*----------------------------------------------------------------------------*/

namespace sReloc
{

typedef sInt_32 sResult;

enum sStandard_Result
{
    sRESULT_SUCCESS =  0,
    sRESULT_INFO    =  1,
    sRESULT_ERROR   = -1
};

enum sUndirectedGraph_Result
{
    sUNDIRECTED_GRAPH_INFO       =  1000,
    sUNDIRECTED_GRAPH_ERROR      = -1000,
    sUNDIRECTED_GRAPH_OPEN_ERROR = (sUNDIRECTED_GRAPH_ERROR - 1)
};

enum sRobotArrangement_Result
{
    sROBOT_ARRANGEMENT_INFO       =  2000,
    sROBOT_ARRANGEMENT_ERROR      = -2000,
    sROBOT_ARRANGEMENT_OPEN_ERROR = (sROBOT_ARRANGEMENT_ERROR - 1),
    sROBOT_ARRANGEMENT_SEEK_ERROR = (sROBOT_ARRANGEMENT_ERROR - 2)
};

enum sRobotGoal_Result {
    sROBOT_GOAL_INFO       =  2000,
    sROBOT_GOAL_ERROR      = -2000,
    sROBOT_GOAL_OPEN_ERROR = (sROBOT_GOAL_ERROR - 1),
    sROBOT_GOAL_SEEK_ERROR = (sROBOT_GOAL_ERROR - 2)
};

enum sMultirobotSolution_Result
{
    sMULTIROBOT_SOLUTION_INFO       =  4000,
    sMULTIROBOT_SOLUTION_ERROR      = -4000,
    sMULTIROBOT_SOLUTION_OPEN_ERROR = (sMULTIROBOT_SOLUTION_ERROR - 1)
};

enum sMultirobotInstance_Result
{
    sMULTIROBOT_INSTANCE_INFO       =  5000,
    sMULTIROBOT_INSTANCE_ERROR      = -5000,
    sMULTIROBOT_OPEN_ERROR          = (sMULTIROBOT_INSTANCE_ERROR - 1),
    sMULTIROBOT_PDDL_OPEN_ERROR     = (sMULTIROBOT_INSTANCE_ERROR - 2),
    sMULTIROBOT_BGU_OPEN_ERROR      = (sMULTIROBOT_INSTANCE_ERROR - 3),
    sMULTIROBOT_USC_MAP_OPEN_ERROR  = (sMULTIROBOT_INSTANCE_ERROR - 4),
    sMULTIROBOT_USC_AGNT_OPEN_ERROR = (sMULTIROBOT_INSTANCE_ERROR - 5),    
    sMULTIROBOT_DIBOX_OPEN_ERROR    = (sMULTIROBOT_INSTANCE_ERROR - 6),    
    sMULTIROBOT_CNF_OPEN_ERROR      = (sMULTIROBOT_INSTANCE_ERROR - 7)
};

enum sHierarchicalRelocationInstance_Result
{
    sHIERARCHICAL_RELOCATION_INSTANCE_INFO   =  6000,
    sHIERARCHICAL_RELOCATION_INSTANCE_ERROR  = -6000,
    sHIERARCHICAL_RELOCATION_OPEN_ERROR      = (sHIERARCHICAL_RELOCATION_INSTANCE_ERROR - 1),
    sHIERARCHICAL_RELOCATION_PDDL_OPEN_ERROR = (sHIERARCHICAL_RELOCATION_INSTANCE_ERROR - 2),
    sHIERARCHICAL_RELOCATION_BGU_OPEN_ERROR  = (sHIERARCHICAL_RELOCATION_INSTANCE_ERROR - 3),
    sHIERARCHICAL_RELOCATION_CNF_OPEN_ERROR  = (sHIERARCHICAL_RELOCATION_INSTANCE_ERROR - 4)
};

enum sMultirobotSolutionCompressor_Result
{
    sMULTIROBOT_SOLUTION_COMPRESSOR_INFO                =  7000,
    sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR               = -7000,

    sMULTIROBOT_SOLUTION_COMPRESSOR_SAT_INFO            = (sMULTIROBOT_SOLUTION_COMPRESSOR_INFO + 1),
    sMULTIROBOT_SOLUTION_COMPRESSOR_UNSAT_INFO          = (sMULTIROBOT_SOLUTION_COMPRESSOR_INFO + 2),
    sMULTIROBOT_SOLUTION_COMPRESSOR_INDET_INFO          = (sMULTIROBOT_SOLUTION_COMPRESSOR_INFO + 3),
    sMULTIROBOT_SOLUTION_COMPRESSOR_MDD_DISCO_INFO      = (sMULTIROBOT_SOLUTION_COMPRESSOR_INFO + 4),
    sMULTIROBOT_SOLUTION_COMPRESSOR_MMDD_DISCO_INFO     = (sMULTIROBOT_SOLUTION_COMPRESSOR_INFO + 5),
    
    sMULTIROBOT_SOLUTION_COMPRESSOR_SYSTEM_CALL_ERROR   = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 1),
    sMULTIROBOT_SOLUTION_COMPRESSOR_OPEN_ERROR          = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 2),
    sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_ATTR_ERROR   = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 3),
    sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_CREATE_ERROR = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 4),
    sMULTIROBOT_SOLUTION_COMPRESSOR_THREAD_JOIN_ERROR   = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 5),
    sMULTIROBOT_SOLUTION_COMPRESSOR_UNLINK_ERROR        = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 6),
    sMULTIROBOT_SOLUTION_COMPRESSOR_CPULIM_ERROR        = (sMULTIROBOT_SOLUTION_COMPRESSOR_ERROR - 7)
};

enum sOptimizerProgram_Result
{
    sOPTIMIZER_PROGRAM_INFO                         =  100000,
    sOPTIMIZER_PROGRAM_ERROR                        = -100000,
    sOPTIMIZER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR = (sOPTIMIZER_PROGRAM_ERROR - 1)
};

enum sResolverProgram_Result
{
    sRESOLVER_PROGRAM_INFO                         =  200000,
    sRESOLVER_PROGRAM_ERROR                        = -200000,
    sRESOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR = (sRESOLVER_PROGRAM_ERROR - 1),
    sRESOLVER_PROGRAM_OUTPUT_OPEN_ERROR            = (sRESOLVER_PROGRAM_ERROR - 2)
};

enum sSolverProgram_Result
{
    sSOLVER_PROGRAM_INFO                         =  300000,
    sSOLVER_PROGRAM_ERROR                        = -300000,
    sSOLVER_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR = (sSOLVER_PROGRAM_ERROR - 1),
    sSOLVER_PROGRAM_OUTPUT_OPEN_ERROR            = (sSOLVER_PROGRAM_ERROR - 2)
};


enum sSimplifyProgram_Result
{
    sSIMPLIFY_PROGRAM_INFO                         =  400000,
    sSIMPLIFY_PROGRAM_ERROR                        = -400000,
    sSIMPLIFY_PROGRAM_UNRECOGNIZED_PARAMETER_ERROR = (sSIMPLIFY_PROGRAM_ERROR - 1),
    sSIMPLIFY_PROGRAM_ELIMINATE_OPEN_ERROR         = (sSIMPLIFY_PROGRAM_ERROR - 2),
    sSIMPLIFY_PROGRAM_SAT_CALL_ERROR               = (sSIMPLIFY_PROGRAM_ERROR - 3),
    sSIMPLIFY_PROGRAM_ANSWER_OPEN_ERROR            = (sSIMPLIFY_PROGRAM_ERROR - 4),
    sSIMPLIFY_PROGRAM_OUTPUT_OPEN_ERROR            = (sSIMPLIFY_PROGRAM_ERROR - 5),
    sSIMPLIFY_PROGRAM_INPUT_OPEN_ERROR             = (sSIMPLIFY_PROGRAM_ERROR - 6)
};


#define sFAILED(result) ((result) < sRESULT_SUCCESS)
#define sSUCCEEDED(result) ((result) >= sRESULT_SUCCESS)


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __RESULT_H__ */

