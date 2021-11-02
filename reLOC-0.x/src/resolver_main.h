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
/* resolver_main.h / 0.22-robik_091                                           */
/*----------------------------------------------------------------------------*/
//
// Solution resolver - main program.
//
// Resolves a given solution of multi-robot path planning problem by a
// specified method.
//
/*----------------------------------------------------------------------------*/


#ifndef __RESOLVER_MAIN_H__
#define __RESOLVER_MAIN_H__

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
	    METHOD_BIBOX,
	    METHOD_WHCA
	};

	BaseMethod m_base_method;
	sMultirobotSolutionCompressor::Encoding m_cnf_encoding;

	int m_whca_window_size;
	int m_cnf_level;
	sString m_solution_filename;
	sString m_input_filename;
	sString m_output_filename;
	sString m_pddl_domain_filename;
	sString m_pddl_problem_filename;
	sString m_cnf_filename;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);

    sResult resolve_MultirobotSolution_BIBOX(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __RESOLVER_MAIN_H__ */
