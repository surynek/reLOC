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
/* simplify_main.h / 0.21-robik_064                                           */
/*----------------------------------------------------------------------------*/
//
// Simplify - main program.
//
// CNF formula simplification by clause elimination.
//
/*----------------------------------------------------------------------------*/


#ifndef __SIMPLIFY_MAIN_H__
#define __SIMPLIFY_MAIN_H__

#include <vector>
#include <list>

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

	sString m_cnf_filename;
	sString m_simplified_cnf_filename;
	sString m_clause_label;
    };

    typedef std::vector<int> Clause_vector;

    struct sClause
    {
	sString m_clause_label;
	Clause_vector m_Literals;
    };

    typedef std::list<sClause> Formula_list;

/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);

    sResult simplify_Formula(const sCommandParameters &parameters);
    sResult read_Formula(FILE *file, Formula_list &formula);
    void write_Formula(FILE *file, const Formula_list &formula);
    sResult eliminate_Clauses(const Formula_list &formula, const sString &label, Formula_list &simplified_formula);

    void skip_CommentsDIMACS(FILE *file);
    void read_CommentsDIMACS(FILE *file, sString &clause_label);
    void print_Formula(const Formula_list &formula);


/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __SIMPLIFY_MAIN_H__ */
