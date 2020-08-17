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
/* gridgen_main.h / 0.21-robik_042                                            */
/*----------------------------------------------------------------------------*/
//
// Grid Instance Generator - main program.
//
// Generated a Multi Robot instance on a 4-connected grid with randomly
// placed obstacles.
//
/*----------------------------------------------------------------------------*/


#ifndef __GRIDGEN_MAIN_H__
#define __GRIDGEN_MAIN_H__

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

	int m_x_size;
	int m_y_size;
	int m_N_robots;
	int m_N_goals;
	int m_seed;
	int m_N_fixed;
	int m_range;
	double m_obstacle_probability;
	int m_N_obstacles;
	int m_cnf_level;
	int m_capacity;
	bool m_directed;

	sString m_map_filename;
	sString m_cnf_filename;
	sString m_pddl_domain_filename;
	sString m_pddl_problem_filename;
	sString m_bgu_filename;
	sString m_dibox_filename;
	sString m_multirobot_filename;

	sString m_usc_map_filename;
	sString m_usc_agents_filename;	
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);

    sResult generate_GridInstance(const sCommandParameters &parameters);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __GRIDGEN_MAIN_H__ */
