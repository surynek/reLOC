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
/* gecode_test.cpp / 0.20-kruh_050                                            */
/*----------------------------------------------------------------------------*/
//
// Gecode solver intergration - testing program.
//
/*----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>

#include "defs.h"

#include "gecode_test.h"


using namespace sReloc;


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    sGecodeTestSpace::sGecodeTestSpace(int sUNUSED(b), int sUNUSED(c))
	: m_a(*this, -10, 10)
	, m_b(*this, -10, 20)
	, m_c(*this, -5, 10)
    {
	rel(*this, m_b, IRT_NQ, m_a);
	rel(*this, m_a, IRT_EQ, m_c);
    }


/*----------------------------------------------------------------------------*/

    void test_gecode_solver_1(void)
    {
	printf("Gecode solver test 1 ...\n");
	printf("Gecode solver test 1 ... finished\n");
    }


/*----------------------------------------------------------------------------*/

} // namespace sReloc


/*----------------------------------------------------------------------------*/
// main program

int main(int sUNUSED(argc), char **sUNUSED(argv))
{
    test_gecode_solver_1();

    return 0;
}
