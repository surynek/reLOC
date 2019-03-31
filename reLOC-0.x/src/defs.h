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
/* defs.h / 0.20-kruh_055                                                     */
=======
/* defs.h / 0.20-kruh_051                                                     */
>>>>>>> fa5fdfe3b98f4658d47e231f3f04e086da930ff8
/*----------------------------------------------------------------------------*/
//
// Definitions for reLOC package.
//
/*----------------------------------------------------------------------------*/

#ifndef __DEFS_H__
#define __DEFS_H__

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>


/*----------------------------------------------------------------------------*/

namespace sReloc
{

#define sUNUSED(x)

#define sMIN(x,y) ((x) < (y) ? (x) : (y))
#define sMAX(x,y) ((x) > (y) ? (x) : (y))
#define sDFR(x,y) ((x) < (y) ? ((y) - (x)) : ((y) - (x)))
#define sABS(x) ((x) < 0 ? -(x) : (x))


/*----------------------------------------------------------------------------*/

#ifdef sDEBUG
  #define sASSERT(condition)                                                             \
    {                                                                                    \
      if (!(condition))							                 \
      {                                                                                  \
        printf("sASSERT: assertion failed (file: %s, line:%d).\n", __FILE__, __LINE__);  \
	exit(-1);						   	                 \
      }                                                                                  \
  }

#else
  #define sASSERT(condition)
#endif /* sDEBUG */


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __DEFS_H__ */
