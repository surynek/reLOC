/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* autooptim_main.h / 0.21-robik_058                                          */
/*----------------------------------------------------------------------------*/
//
// Automatic solution optimizer - main program.
//
// The program automatically optimizes given solution to multi-robot
// path planning instance. Maximum makespan bound that fits into the
// given time limit is used.
//
/*----------------------------------------------------------------------------*/


#ifndef __AUTOOPTIM_MAIN_H__
#define __AUTOOPTIM_MAIN_H__

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
	int m_total_timeout;

	int m_N_Threads;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &command_parameters);

    sResult autoOptimize_MultirobotSolution(const sCommandParameters &command_parameters);
    sResult autoPrimeOptimize_MultirobotSolution(const sCommandParameters &command_parameters);
    sResult autoDeflate_MultirobotSolution(const sCommandParameters &command_parameters);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __AUTOOPTIM_MAIN_H__ */
