//****************************************************************************//
// File:          cp_cc.h                                                     //
// Description:   Definitions for CAN controller chips / targets              //
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
// 29.07.2003  Initial version                                                //
//                                                                            //
//****************************************************************************//


#ifndef  _CP_CC_H_
#define  _CP_CC_H_


//------------------------------------------------------------------------------
// SVN  $Date: 2007-06-11 23:02:46 +0200 (Mon, 11 Jun 2007) $
// SVN  $Rev: 37 $ --- $Author: microcontrol $
//------------------------------------------------------------------------------


//-----------------------------------------------------------------------------
/*!   
** \file    cp_cc.h
** \brief   CANpie constant values for targets
**
**
*/


//-------------------------------------------------------//
// Generic                                               //
//-------------------------------------------------------//

//--- LINUX ---------------------------
#define CP_CC_LINUX           0x0010


//-------------------------------------------------------//
// Analog Devices    (0x08xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// ATMEL             (0x10xx)                            //
//-------------------------------------------------------//

//--- CANary, AT89C51CC01 -------------
#define CP_CC_CC01            0x1000

//--- CANary, AT89C51CC02 -------------
#define CP_CC_CC02            0x1001

//--- CANary, AT89C51CC03 -------------
#define CP_CC_CC03            0x1002


//-------------------------------------------------------//
// Bosch             (0x14xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Cygnal            (0x18xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Dallas            (0x1Cxx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Freescale         (0x1Dxx)                            //
//-------------------------------------------------------//
#define CP_CC_XGATE           0x1D00

#define CP_CC_MCF523x         0x1D01
   
//-------------------------------------------------------//
// Fujitsu           (0x20xx)                            //
//-------------------------------------------------------//

#define CP_CC_LX340           0x2010

//-------------------------------------------------------//
// Infineon          (0x24xx)                            //
//-------------------------------------------------------//

//--- Infineon C505 -------------------
#define CP_CC_C505            0x2400

//--- Infineon C515 -------------------
#define CP_CC_C515            0x2401

//--- Infineon C161 -------------------
#define CP_CC_C161            0x2402

//--- Infineon C164 -------------------
#define CP_CC_C164            0x2403

//--- Infineon C167 -------------------
#define CP_CC_C167            0x2404

//--- Infineon 81C90 ------------------
#define CP_CC_81C90           0x2405

//--- Infineon 81C91 ------------------
#define CP_CC_81C91           0x2406

//-------------------------------------------------------//
// Microchip         (0x28xx)                            //
//-------------------------------------------------------//
#define CP_CC_18Fxx8x         0x2810
#define CP_CC_18F6680         0x2812
#define CP_CC_18F8680         0x2814

//-------------------------------------------------------//
// Micronas          (0x2Cxx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Motorola          (0x30xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// National Semi.    (0x34xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// NEC               (0x38xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Philips           (0x3Cxx)                            //
//-------------------------------------------------------//

//--- Philips 82C200 ------------------
#define CP_CC_82C200          0x3C00

//--- Philips SJA1000 -----------------
#define CP_CC_SJA1000         0x3C01

//--- Philips 80C591 ------------------
#define CP_CC_80C591          0x3C02

//--- Philips 80C592 ------------------
#define CP_CC_80C592          0x3C03

#define CP_CC_LPC2119         0x3C10

#define CP_CC_LPC2129         0x3C11


//-------------------------------------------------------//
// Renesas           (0x40xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// ST                (0x44xx)                            //
//-------------------------------------------------------//

#define CP_CC_STR712          0x4420

//-------------------------------------------------------//
// Texas Instruments (0x48xx)                            //
//-------------------------------------------------------//

//-------------------------------------------------------//
// Toshiba           (0x4Cxx)                            //
//-------------------------------------------------------//



//----------------------------------------------------------------------------//
//                                                                            //
// Define the CANpie target here (or in the Makefile)                         //
//                                                                            //
//----------------------------------------------------------------------------//
#ifndef  CP_TARGET
#define  CP_TARGET   CP_CC_LINUX
#endif


#endif   /* _CP_CC_H_   */

