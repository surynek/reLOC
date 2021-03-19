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
/* cnf_test.h / 0.22-robik_069                                                */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF production tools - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __CNF_TEST_H__
#define __CNF_TEST_H__

#include "reloc.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_indexed_identifier_1(void);
    void test_indexed_identifier_2(void);

    void test_indexed_state_identifier_1(void);
    void test_indexed_state_identifier_2(void);

    void test_clause_generator_1(void);
    void test_clause_generator_2(void);

    void test_difference_generator_1(void);
    void test_difference_generator_2(void);
    void test_difference_generator_3(void);

    void test_case_split_generator_1(void);

    void test_case_LEX_less_generator_1(void);

    void test_binary_tree_1(void);

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __CNF_TEST_H__ */
