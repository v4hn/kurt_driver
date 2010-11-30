//****************************************************************************//
// File:          cp_arch.h                                                   //
// Description:   CANpie architecture definitions                             //
// Author:        Uwe Koppe                                                   //
// e-mail:        koppe@microcontrol.net                                      //
//                                                                            //
// Copyright (C) MicroControl GmbH & Co. KG                                   //
// 53842 Troisdorf - Germany                                                  //
// www.microcontrol.net                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
// Redistribution and use in source and binary forms, with or without         //
// modification, are permitted provided that the following conditions         //
// are met:                                                                   //
// 1. Redistributions of source code must retain the above copyright          //
//    notice, this list of conditions, the following disclaimer and           //
//    the referenced file 'COPYING'.                                          //
// 2. Redistributions in binary form must reproduce the above copyright       //
//    notice, this list of conditions and the following disclaimer in the     //
//    documentation and/or other materials provided with the distribution.    //
// 3. Neither the name of MicroControl nor the names of its contributors      //
//    may be used to endorse or promote products derived from this software   //
//    without specific prior written permission.                              //
//                                                                            //
// Provided that this notice is retained in full, this software may be        // 
// distributed under the terms of the GNU General Public License ("GPL")      //
// version 2 as distributed in the 'COPYING' file from the main directory     //
// of the linux kernel source.                                                //
//                                                                            //
//----------------------------------------------------------------------------//
//                                                                            //
// Date        History                                                        //
// ----------  -------------------------------------------------------------- //
// 23.07.2003  Initial version                                                //
//                                                                            //
//****************************************************************************//


#ifndef  _CP_ARCH_H_
#define  _CP_ARCH_H_


//------------------------------------------------------------------------------
// SVN  $Date: 2007-08-06 22:59:59 +0200 (Mon, 06 Aug 2007) $
// SVN  $Rev: 62 $ --- $Author: microcontrol $
//------------------------------------------------------------------------------



//-----------------------------------------------------------------------------
/*!   \file    cp_arch.h
**    \brief   CANpie architecute definitions
**
**
*/


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/
#include "compiler.h"      // compiler definitions
#include "cp_cc.h"         // definitions for CAN controller / target


/*----------------------------------------------------------------------------*\
** Definitions & Enumerations                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/

//-------------------------------------------------------------------
// The symbol CP_TARGET defines the target for the CANpie sources.
//
#ifndef CP_TARGET
#error  Target (Symbol CP_TARGET) is not defined! Check file cp_cc.h!
#endif


/*----------------------------------------------------------------------------*\
** Structures                                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/




/*----------------------------------------------------------------------------*/
/*!
** \struct  CpPortLinux_s   cp_arch.h
** \brief   Port structure for Linux
**
*/
struct CpPortLinux_s {

   /*!   logical CAN interface number,
   **    first index is 0, value -1 denotes not assigned
   */
   int32_t  slLogIf;

   /*!   physical CAN interface number,
   **    first index is 0, value -1 denotes not assigned
   */
   int32_t  slPhyIf;

   /*!   CAN message queue number,
   **    first index is 0, value -1 denotes not assigned
   */
   int32_t  slQueue;
};


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpPortEmbedded_s   cp_arch.h
** \brief   Port structure for embedded target
**
*/
struct CpPortEmbedded_s {

   /*!   physical CAN interface number,
   **    first CAN channel (index) is 0
   */
   _U08     ubPhyIf;

   /*!   logical CAN interface number,
   **    first index is 0
   */
   _U08     ubLogIf;

};


//---------------------------------------------------------------------
// Architecture definitions for Linux
//
#if CP_TARGET == CP_CC_LINUX

typedef struct CpPortLinux_s     _TsCpPort;
typedef int32_t                  _TvCpStatus;

#define CP_CAN_MSG_MACRO   1
#define CP_CAN_MSG_USER    0
#define CP_CAN_MSG_TIME    1

#define CP_SMALL_CODE      0
#define CP_STATISTIC       1

#endif


//---------------------------------------------------------------------
// Architecture definitions for ATMEL CANary
//
#if CP_TARGET == CP_CC_CC01

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_AUTOBAUD        0
#define CP_BUFFER_MAX      14
#define CP_CAN_MSG_MACRO   1
#define CP_SMALL_CODE      1
#endif

//---------------------------------------------------------------------
// Architecture definitions for Microchip 18Fxx8x
//
#if CP_TARGET == CP_CC_18Fxx8x

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_SMALL_CODE      1
#define CP_CAN_MSG_MACRO   1
#endif



//---------------------------------------------------------------------
// Architecture definitions for Fujitsu LX16 family (340)
//
#if CP_TARGET == CP_CC_LX340

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_AUTOBAUD        1
#define CP_BUFFER_MAX      16
#define CP_CAN_MSG_MACRO   1
#define CP_SMALL_CODE      1
#define CP_STATISTIC       1

#endif


//---------------------------------------------------------------------
// Architecture definitions for Infineon C16x family
//
#if CP_TARGET == CP_CC_C167

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_AUTOBAUD        0
#define CP_BUFFER_MAX      16
#define CP_CAN_MSG_MACRO   1
#define CP_SMALL_CODE      1
#define CP_STATISTIC       0

#endif

//---------------------------------------------------------------------
// Architecture definitions for Freescale ColdFire MCF523x
//
#if CP_TARGET == CP_CC_MCF523x

typedef struct CpPortEmbedded_s  _TsCpPort;
typedef _U16                     _TvCpStatus;

#define CP_AUTOBAUD        		0
#define CP_BUFFER_MAX      		14
#define CP_CAN_MSG_MACRO   		1
#define CP_SMALL_CODE      		0
#define CP_STATISTIC       		1

#define CP_GLOBAL_RCV_ENABLE		1
#define CP_GLOBAL_RCV_BUFFER		16
#define CP_GLOBAL_RCV_MASK			0x00000000
#endif

//---------------------------------------------------------------------
// Architecture definitions for Philips LPC21x9
//
#if (CP_TARGET == CP_CC_LPC2119) || (CP_TARGET == CP_CC_LPC2129)

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_SMALL_CODE      1
#define CP_CAN_MSG_MACRO   1
#endif


//---------------------------------------------------------------------
// Architecture definitions for ST STR712
//
#if CP_TARGET == CP_CC_STR712

typedef _U08               _TsCpPort;
typedef _U08               _TvCpStatus;

#define CP_SMALL_CODE      1
#define CP_CAN_MSG_MACRO   1
#endif

#endif   /* _CP_ARCH_H_   */

