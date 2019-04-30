/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                              reLOC 0.20-kruh                               */
/*                                                                            */
/*                      (C) Copyright 2019 Pavel Surynek                      */
/*                http://www.surynek.com | <pavel@surynek.com>                */
/*                                                                            */
/*                                                                            */
/*============================================================================*/
<<<<<<< HEAD
/* cnf_test.h / 0.20-kruh_055                                                 */
=======
/* cnf_test.h / 0.20-kruh_056                                                 */
>>>>>>> f57c68398eae7b055f31a50698a3a79978214a2b
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
