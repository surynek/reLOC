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
/* reloc_test.h / 0.22-robik_074                                              */
/*----------------------------------------------------------------------------*/
//
// Relocation problem solving package - testing program.
//
/*----------------------------------------------------------------------------*/


#ifndef __RELOC_TEST_H__
#define __RELOC_TEST_H__

#include "reloc.h"

using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    void test_undirected_graph_1(void);
    void test_undirected_graph_2(int N_Vertices, double edge_prob);
    void test_undirected_graph_3(const sString &filename);
    void test_undirected_graph_4(void);
    void test_undirected_graph_5(void);

    void test_statistics_1(void);

    
/*----------------------------------------------------------------------------*/

} // namespace sReloc


#endif /* __RELOC_TEST_H__ */
