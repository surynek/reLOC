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
/* alldiff_test.h / 0.22-robik_091                                            */
/*----------------------------------------------------------------------------*/
//
// Dimacs CNF production tools - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __ALLDIFF_TEST_H__
#define __ALLDIFF_TEST_H__

#include "reloc.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_all_different_Standard(int N_Variables, int N_States, int range_size);
    void test_all_different_Bijection(int N_Variables, int N_States, int range_size);

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __ALLDIFF_TEST_H__ */
