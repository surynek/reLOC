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
/* gecode_test.cpp / 0.20-kruh_055                                            */
=======
/* gecode_test.cpp / 0.20-kruh_051                                            */
>>>>>>> fa5fdfe3b98f4658d47e231f3f04e086da930ff8
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
