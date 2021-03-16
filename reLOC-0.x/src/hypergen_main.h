/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.21-robik                              */
/*                                                                            */
/*                  (C) Copyright 2011 - 2021 Pavel Surynek                   */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
/* hypergen_main.h / 0.21-robik_064                                           */
/*----------------------------------------------------------------------------*/
//
// Hyper-cube Instance Generator - main program.
//
// Generated a Multi Robot instance on a hyper-cube
//
/*----------------------------------------------------------------------------*/


#ifndef __HYPERGEN_MAIN_H__
#define __HYPERGEN_MAIN_H__

#include "reloc.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    struct sCommandParameters
    {
	sCommandParameters();
        /*--------------------------------*/

	bool m_disjoint;
	bool m_walk;
	int m_distance;

	int m_dimmension;
	int m_N_robots;
	int m_N_goals;
	int m_seed;
	int m_N_fixed;
	int m_cnf_level;

	int m_aisle_length;
	int m_min_aisle_length;
	int m_max_aisle_length;

	sString m_map_filename;
	sString m_cnf_filename;
	sString m_pddl_domain_filename;
	sString m_pddl_problem_filename;
	sString m_bgu_filename;
	sString m_multirobot_filename;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);

    sResult generate_GridInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __HYPERGEN_MAIN_H__ */
