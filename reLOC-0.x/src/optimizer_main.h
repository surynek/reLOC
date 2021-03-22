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
/* optimizer_main.h / 0.22-robik_075                                          */
/*----------------------------------------------------------------------------*/
//
// Solution optimizer - main program.
//
// This optimizes a given solution of multirobot path planning instance.
// SAT-based techniques are used for the solution optimization.
//
/*----------------------------------------------------------------------------*/


#ifndef __OPTIMIZER_MAIN_H__
#define __OPTIMIZER_MAIN_H__

#include "reloc.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    struct sCommandParameters
    {
	enum BaseMethod
	{
	    METHOD_UNDEFINED,
	    METHOD_OPTIMIZATION,
	    METHOD_PRIME_OPTIMIZATION,
	    METHOD_DEFLATION
	};

	BaseMethod m_base_method;
	sMultirobotSolutionCompressor::Encoding m_cnf_encoding;

	sString m_solution_filename;
	int m_minisat_timeout;
	int m_makespan_bound;
	int m_N_Threads;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters);

    sResult optimize_MultirobotSolution(const sCommandParameters &command_parameters);
    sResult prime_optimize_MultirobotSolution(const sCommandParameters &command_parameters);
    sResult deflate_MultirobotSolution(const sCommandParameters &command_parameters);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __OPTIMIZER_MAIN_H__ */
