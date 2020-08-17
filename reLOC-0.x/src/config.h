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
/* config.h / 0.21-robik_041                                                  */
/*----------------------------------------------------------------------------*/
//
// Configuration file for reLOC package - global settings.
//
/*----------------------------------------------------------------------------*/

#ifndef __CONFIG_H__
#define __CONFIG_H__


/*----------------------------------------------------------------------------*/

namespace sReloc
{


/*----------------------------------------------------------------------------*/

    static const int sDEFAULT_MINISAT_TIMEOUT     =       4;
    static const int sDEFAULT_MINISAT_UPPER_BOUND =      16;
    static const int sDEFAULT_N_PARALLEL_THREADS  =       4;
    static const int sDEFAULT_N_WHCA_ITERATIONS   =     128;
    
    static const int sDEFAULT_RANDOM_WALK_LENGTH  = 1048576;
//    static const int sDEFAULT_RANDOM_WALK_LENGTH  = 180;

//    static const int sDEFAULT_RANDOM_WALK_LENGTH  = 32768;
//    static const int sDEFAULT_RANDOM_WALK_LENGTH  = 131072;
//    static const int sDEFAULT_RANDOM_WALK_LENGTH  = 262144;


/*----------------------------------------------------------------------------*/

} // namespace sReloc

#endif /* __CONFIG_H__ */
