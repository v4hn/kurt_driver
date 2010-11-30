/*****************************************************************************\
*  CANpie                                                                     *
*                                                                             *
*  File        : compiler.h                                                   *
*  Description :                                                              *
*  Author      : Uwe Koppe                                                    *
*  e-mail      : koppe@microcontrol.net                                       *
*                                                                             *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - *
*                                                                             *
*   This program is free software; you can redistribute it and/or modify      *
*   it under the terms of the GNU General Public License as published by      *
*   the Free Software Foundation; either version 2 of the License, or         *
*   (at your option) any later version.                                       *
*                                                                             *
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - *
*                                                                             *
*  History                                                                    *
*  Vers.    Date        Comment                                         Aut.  *
*  -----    ----------  ---------------------------------------------   ----  *
*  0.1      04.12.1998  Initial version                                 UK    *
*  0.2      29.04.1999  New data type definitions,                      UK    *
*                       Machine dependant definitions                         *
*  1.0      15.06.2000  Final release                                   UK    *
*  1.1      07.09.2000  New symbol for const/code                       UK    *
*                                                                             *
\*****************************************************************************/

#ifndef  _COMPILER_H_
#define  _COMPILER_H_

//------------------------------------------------------------------------------
// SVN  $Date: 2007-08-08 22:38:20 +0200 (Wed, 08 Aug 2007) $
// SVN  $Rev: 64 $ --- $Author: microcontrol $
//------------------------------------------------------------------------------


#ifndef  FALSE
#define  FALSE  0
#endif

#ifndef  TRUE
#define  TRUE   1
#endif



//----------------------------------------------------------------------------//
// Target Machine / Compiler dependant definitions                            //
//                                                                            //
//----------------------------------------------------------------------------//



/*---------------------------------------------------------
** Microsoft Visual C/C++
**
*/
#ifdef _MSC_VER

#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: _MSC_VER
**-------------------------------------------------------*/

/*---------------------------------------------------------
** CVI, National Instruments
**
*/
#ifdef _CVI_

#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: _CVI_
**-------------------------------------------------------*/

/*---------------------------------------------------------
** LINUX, gcc
**
*/
#ifdef __linux__

#ifdef __KERNEL__
#include <linux/types.h>   // data types uint8_t ... uint32_t, kernel space
#else
#include <stdint.h>        // data types uint8_t ... uint32_t, user space
#endif

#define Cp_PREFIX 

#define  _CON     const
#define  _BIT     uint8_t
#define  _U08     uint8_t
#define  _S08     int8_t
#define  _U16     uint16_t
#define  _S16     int16_t
#define  _U32     uint32_t
#define  _S32     int32_t

#endif
/* End of definition: __linux__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** Keil C for 8051 cores
**
*/
#ifdef __C51__

#define  _CON
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: __C51__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** Keil C for C16x cores
**
*/
#ifdef __C166__

#define  _CON
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: __C51__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** Fujitsu C Compiler for 16LX series
**
*/
#ifdef __COMPILER_FCC907__

#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     signed char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: __COMPILER_FCC907S__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** Imagecraft C Compiler for Atmel AVR series
**
*/
#ifdef _AVR

#define  _CON
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     signed char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: _AVR
**-------------------------------------------------------*/


/*---------------------------------------------------------
** GNU C Compiler for Atmel AVR series
**
*/
#ifdef AVR

#define  _CON
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     signed char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: AVR
**-------------------------------------------------------*/


/*---------------------------------------------------------
** IAR ARM compiler
**
*/
#ifdef __ICCARM__

#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: __ICCARM__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** Hi-Tech PICC18
**
*/
#ifdef HI_TECH_C

#define  _CON
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: _HI_TECH_C
**-------------------------------------------------------*/

/*---------------------------------------------------------
** Metrowerks C Compiler for XGATE
**
*/
#ifdef __MWERKS__


#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif
/* End of definition: __MWERKS__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** GNU C for ARM controller
**
*/
#ifdef __GNUC__ 
#ifdef __arm__
#if __arm__ > 0

#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif   // __arm__ > 0
#endif   // __arm__
#endif   // __GNUC__ 
/* 
** End of definition: __GNUC__ and __arm__
**-------------------------------------------------------*/


/*---------------------------------------------------------
** GNU C for ColdFire controller
**
*/
#ifdef __GNUC__
#ifdef __mc68000__
#if __mc68000__ > 0

#define  _CON     const
#define  _BIT     unsigned char
#define  _U08     unsigned char
#define  _S08     char
#define  _U16     unsigned short
#define  _S16     short
#define  _U32     unsigned long
#define  _S32     long

#endif   // __mc68000__ > 0
#endif   // __mc68000__
#endif   // __GNUC__
/* 
** End of definition: __GNUC__ and __mc68000__
**-------------------------------------------------------*/

#ifndef  _U32
#error   Data types are not defined! Please check compiler definition.
#endif

#endif   /* _COMPILER_H_ */
