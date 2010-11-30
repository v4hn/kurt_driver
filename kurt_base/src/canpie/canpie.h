//****************************************************************************//
// File:          canpie.h                                                    //
// Description:   General CAN driver definitions and structures               //
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
// 04.12.1998  Initial version                                                //
// 29.04.1999  Changed structures, new data type definitions                  //
// 15.06.2000  moved defintions from cpmsg.h                                  //
// 06.11.2000  added new error codes, added missing buffer number             //
// 12.06.2007  update HDI and statistic structure                             //
//                                                                            //
//****************************************************************************//


#ifndef  _CANPIE_H_
#define  _CANPIE_H_


//------------------------------------------------------------------------------
// SVN  $Date: 2007-08-06 22:58:05 +0200 (Mon, 06 Aug 2007) $
// SVN  $Rev: 61 $ --- $Author: microcontrol $
//------------------------------------------------------------------------------


//-----------------------------------------------------------------------------
/*!   \file    canpie.h
**    \brief   CANpie constants, structures and enumerations
**
**    This file holds constants and structures used within CANpie.
**
*/


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/

#include "cp_arch.h"       // Architecture dependent definitions


/*----------------------------------------------------------------------------*\
** Definitions & Enumerations                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/


#ifndef  CP_CAN_MSG_TIME
#define  CP_CAN_MSG_TIME            0
#endif

#ifndef  CP_CAN_MSG_USER
#define  CP_CAN_MSG_USER            0
#endif

#ifndef  CP_CAN_MSG_MACRO
#define  CP_CAN_MSG_MACRO           0
#endif

#ifndef  CP_CHANNEL_MAX
#define  CP_CHANNEL_MAX             1
#endif

#define  CP_VERSION_MAJOR           1
#define  CP_VERSION_MINOR           93


/*-----------------------------------------------------------------------------
** CP_MASK_XXX
** mask for CAN message (RTR, Extended CAN)
*/


/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_STD_FRAME
**
** Mask for standard frame (11 bits)
*/
#define  CP_MASK_STD_FRAME 0x000007FF


/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_EXT_FRAME
**
** Mask for extended frame (29 bits)
*/
#define  CP_MASK_EXT_FRAME 0x1FFFFFFF


/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_EXT_BIT
**
** Set the EXT bit (extended frame) in the ubMsgCtrl field of
** the _TsCpCanMsg structure.
*/
#define  CP_MASK_EXT_BIT   0x01


/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_RTR_BIT
**
** Set the RTR bit (remote frame) in the ubMsgCtrl field of
** the _TsCpCanMsg structure.
*/
#define  CP_MASK_RTR_BIT   0x02


/*-------------------------------------------------------------------*/
/*!
** \def  CP_MASK_OVR_BIT
**
** Set the OVR bit (overrun) in the ubMsgCtrl field of
** the _TsCpCanMsg structure.
*/
#define  CP_MASK_OVR_BIT   0x04



/*----------------------------------------------------------------------------*/
/*!
** \enum    CpErr
** \brief   CANpie Error codes
**
**
*/
enum CpErr {

   /*!   No error (00dec / 00hex)
   */
   CpErr_OK = 0,

   /*!   Error not specified (01dec / 01hex)
   */
   CpErr_GENERIC,

   /*!   Hardware failure (02dec / 02hex)
   */
   CpErr_HARDWARE,

   /*!   Initialisation failure (03dec / 03hex)
   */
   CpErr_INIT_FAIL,

   /*!   Channel is initialized, ready to run (04dec / 04hex)
   */
   CpErr_INIT_READY,

   /*!    CAN channel was not initialized (05dec / 05hex)
   */
   CpErr_INIT_MISSING,

   /*!   Receive buffer is empty (05dec / 05hex)
   */
   CpErr_RCV_EMPTY,

   /*!   Receive buffer overrun (06dec / 06hex)
   */
   CpErr_OVERRUN,

   /*!   Transmit buffer is full (07dec / 07hex)
   */
   CpErr_TRM_FULL,

   /*!   CAN message has wrong format (10dec / 0Ahex)
   */
   CpErr_CAN_MESSAGE = 10,

   /*!   CAN identifier not valid (11dec / 0Bhex)
   */
   CpErr_CAN_ID,

   /*!   CAN data length code not valid (12dec / 0Chex)
   */
   CpErr_CAN_DLC,

   /*!   FIFO is empty (20dec / 14hex)
   */
   CpErr_FIFO_EMPTY = 20,

   /*!   Message is waiting in FIFO (21dec / 15hex)
   */
   CpErr_FIFO_WAIT,

   /*!   FIFO is full (22dec / 16hex)
   */
   CpErr_FIFO_FULL,

   /*!   FIFO size is out of range (23dec / 17hex)
   */
   CpErr_FIFO_SIZE,

   /*!   Parameter of FIFO function is out of range (24dec / 18hex)
   */
   CpErr_FIFO_PARM,

   /*!   Controller is in error passive (30dec / 1Ehex)
   */
   CpErr_BUS_PASSIVE = 30,

   /*!   Controller is in bus off (31dec / 1Fhex)
   */
   CpErr_BUS_OFF,

   /*!   Controller is in warning status (32dec / 20hex)
   */
   CpErr_BUS_WARNING,


   /*!   Channel out of range (40dec / 28hex)
   */
   CpErr_CHANNEL = 40,

   /*!   Register address out of range (41dec / 29hex)
   */
   CpErr_REGISTER,

   /*!   Baudrate out of range (42dec / 2Ahex)
   */
   CpErr_BAUDRATE,

   /*!   Buffer number out of range (43dec / 2Bhex)
   */
   CpErr_BUFFER,

   /*!   Parameter number out of range (44dec / 2Chex)
   */
   CpErr_PARAM,

   /*!   Function is not supported (50dec / 32hex)
   */
   CpErr_NOT_SUPPORTED = 50
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_FIFO
** \brief   FIFO Buffer numbers
*/
enum CP_FIFO {
   CP_FIFO_RCV = 0,
   CP_FIFO_TRM
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_CALLBACK
** \brief   Callback Return Codes
**
** These return values are used by the callback functions that can be
** installed by the function CpUserIntFunctions(). <p>
** \b Example
** \code
** _U08 MyCallback(_TsCpCanMsg * pCanMsgV)
** {
**    // Do something with IDs < 100
**    if( CpMacGetStdId(pCanMsgV) < 100)
**    {
**       //.....
**       return(CP_CALLBACK_PROCESSED)
**    }
**
**    // Put all other messages into the FIFO
**    return (CP_CALLBACK_PUSH_FIFO);
** }
** \endcode
** <br>


*/
enum CP_CALLBACK {

   /*!
   ** Message was processed by callback and is not inserted in the FIFO
   */
   CP_CALLBACK_PROCESSED = 0,

   /*!
   ** Message was processed by callback and is inserted in the FIFO
   */
   CP_CALLBACK_PUSH_FIFO
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_BAUD
** \brief   Fixed baudrates
**
** The values of the enumeration CP_BAUD are used as parameter for the
** functions CpUserBaudrate() and CpCoreBaudrate().
*/
enum CP_BAUD {
   /*!
   ** Baudrate 10 kBit/sec
   */
   CP_BAUD_10K = 0,

   /*!
   ** Baudrate 20 kBit/sec
   */
   CP_BAUD_20K,

   /*!
   ** Baudrate 50 kBit/sec
   */
   CP_BAUD_50K,

   /*!
   ** Baudrate 100 kBit/sec
   */
   CP_BAUD_100K,

   /*!
   ** Baudrate 125 kBit/sec
   */
   CP_BAUD_125K,

   /*!
   ** Baudrate 250 kBit/sec
   */
   CP_BAUD_250K,

   /*!
   ** Baudrate 500 kBit/sec
   */
   CP_BAUD_500K,

   /*!
   ** Baudrate 800 kBit/sec
   */
   CP_BAUD_800K,

   /*!
   ** Baudrate 1 MBit/sec
   */
   CP_BAUD_1M,

   /*!
   ** Use automatic baudrate detection
   */
   CP_BAUD_AUTO,

   CP_BAUD_MAX = CP_BAUD_AUTO
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_CHANNEL
** \brief   Channel definition
*/
enum CP_CHANNEL {
   CP_CHANNEL_1 = 0,
   CP_CHANNEL_2,
   CP_CHANNEL_3,
   CP_CHANNEL_4,
   CP_CHANNEL_5,
   CP_CHANNEL_6,
   CP_CHANNEL_7,
   CP_CHANNEL_8
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_MODE
** \brief   Mode of CAN controller
**
** These values are used as parameter for the function CpCoreCanMode() in
** order to change the state of the CAN controller.
*/
enum CP_MODE {
   /*!   Set controller in Stop mode (no reception / transmission possible)
   */
   CP_MODE_STOP = 0,

   /*!   Set controller into normal operation
   */
   CP_MODE_START,

   /*!   Set controller into listen-only mode
   */
   CP_MODE_LISTEN_ONLY,

   /*!   Set controller into Sleep mode
   */
   CP_MODE_SLEEP
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_STATE
** \brief   State of CAN controller
**
** These values are used as return value for the function CpCoreCanState().
*/
enum CP_STATE {
   /*!
   ** CAN controller is in stopped mode
   */
   CP_STATE_STOPPED  = 0,


   /*!
   ** CAN controller is in Sleep mode
   */
   CP_STATE_SLEEPING,


   /*!
   ** CAN controller is error active
   */
   CP_STATE_BUS_ACTIVE,


   /*!
   ** CAN controller is active, warning level is reached
   */
   CP_STATE_BUS_WARN,

   /*!
   ** CAN controller is error passive
   */
   CP_STATE_BUS_PASSIVE,

   /*!
   ** CAN controller went into Bus Off
   */
   CP_STATE_BUS_OFF,

   /*!
   ** General failure of physical layer detected (if supported by hardware)
   */
   CP_STATE_PHY_FAULT = 10,

   /*!
   ** Fault on CAN-H detected (Low Speed CAN)
   */
   CP_STATE_PHY_H,

   /*!
   ** Fault on CAN-L detected (Low Speed CAN)
   */
   CP_STATE_PHY_L

};

enum CP_ERR_TYPE {
   CP_ERR_TYPE_NONE   = 0,
   CP_ERR_TYPE_BIT0,
   CP_ERR_TYPE_BIT1,
   CP_ERR_TYPE_STUFF,
   CP_ERR_TYPE_FORM,
   CP_ERR_TYPE_CRC,
   CP_ERR_TYPE_ACK
};

/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_BUFFER
** \brief   Buffer definition
**
** The enumeration CP_BUFFER is used to define a message buffer inside a
** FullCAN controller. The index for the first buffer starts at 1.
*/
enum CP_BUFFER {
   CP_BUFFER_1 = 1,
   CP_BUFFER_2,
   CP_BUFFER_3,
   CP_BUFFER_4,
   CP_BUFFER_5,
   CP_BUFFER_6,
   CP_BUFFER_7,
   CP_BUFFER_8,
   CP_BUFFER_9,
   CP_BUFFER_10,
   CP_BUFFER_11,
   CP_BUFFER_12,
   CP_BUFFER_13,
   CP_BUFFER_14,
   CP_BUFFER_15
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    CP_BUFFER_DIR
** \brief   Buffer direction definition
*/
enum CP_BUFFER_DIR {
   /*!
   **    Buffer direction is receive
   */
   CP_BUFFER_DIR_RX = 0,

   /*!
   **    Buffer direction is transmit
   */
   CP_BUFFER_DIR_TX
};


/*----------------------------------------------------------------------------*\
** Structures                                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/


struct CpTime_s {
   _U32  ulSec1970;
   _U32  ulNanoSec;
};

typedef struct CpTime_s  _TsCpTime;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpCanMsg_s   canpie.h
** \brief   CAN message structure
**
** For transmission and reception of CAN messages a structure which holds
** all necessary informations is used. The structure has the following
** data fields:
*/
struct CpCanMsg_s {

   /*!   The identifier field may have 11 bits for standard frames
   **    (CAN specification 2.0A) or 29 bits for extended frames
   **    (CAN specification 2.0B). The three most significant bits
   **    are reserved (always read 0).
   */
   union {
      _U16  uwStd;
      _U32  ulExt;
   } tuMsgId;

   /*!   The data field has up to 8 bytes (64 bit) of message data.
   **    The number of used bytes is described via the structure
   **    member <b>ubDLC</b>.
   */
   union {
      /*!   byte access, array of 8 byte     */
      _U08  aubByte[8];

      /*!   word access, array of 4 words    */
      _U16  auwWord[4];
      _U32  aulLong[2];
   } tuMsgData;


   /*!   The data length code denotes the number of data bytes
   **    which are transmitted by a message.
   **    The possible value range for the data length code is
   **    from 0 to 8 (bytes).<p>
   */
   _U08  ubMsgDLC;


   /*!   The structure member <b>ubMsgCtrl</b> defines the
   **    different data frames (2.0A / 2.0B) and the RTR frames.
   **    <ul>
   **    <li>Bit 0: Extended Frame if set to 1, else Standard Frame
   **    <li>Bit 1: Remote Frame if set to 1, else Data Frame
   **    <li>Bit 2: Receiver Overrun if set to 1, else normal reception
   **    </ul>
   */
   _U08  ubMsgCtrl;


#if CP_CAN_MSG_TIME == 1
   /*!   The time stamp field defines the time when a CAN message
   **    was received by the CAN controller. This is an optional
   **    field (available if #CP_CAN_MSG_TIME is set to 1).
   */
   _TsCpTime tsMsgTime;
#endif

#if CP_CAN_MSG_USER == 1

   /*!   The field user data can hold a 32 bit value, which is
   **    defined by the user. This is an optional field
   **    (available if #CP_CAN_MSG_USER is set to 1).
   */
   _U32  ulMsgUser;
#endif

};



/*----------------------------------------------------------------------------*/
/*!
** \typedef _TsCpCanMsg
** \brief   CAN message structure
**
** For transmission and reception of CAN messages the structure CpCanMsg_s
** is used.
*/
typedef struct CpCanMsg_s  _TsCpCanMsg;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpStruct_HDI   canpie.h
** \brief   Hardware description interface
**
** The Hardware Description Interface provides a method to gather
** information about the CAN hardware and the functionality of the driver.
** All items in the structure CpStruct_HDI are constant and must be
** supplied by the designer of the CAN driver. The hardware description
** structure is available for every physical CAN channel.
*/

struct CpHdi_s {

  _U16 uwVersionNumber;

   /*!   Bit coded value that decribes the features of the CAN driver.
   **    <ul>
   **    <li>Bit 0/1: 0 = Standard Frame, 1 = Extended Frame passive,
   **        2 = Extended Frame active
   **    <li>Bit 2: 0 = BasicCAN, 1 = FullCAN
   **    <li>Bit 3: 0 = No IRQ Handler, 1 = Has IRQ Handler
   **    <li>Bit 4: 0 = No identifier filter, 1 = software identifier filter
   **    <li>Bit 5: 0 = No timestamp, 1 = has timestamp
   **    <li>Bit 6: 0 = No user data field, 1 = has user data field
   **    <li>Bit 7: reserved
   **    </ul>
   */
  _U16 uwSupportFlags;

   /*!   Constant value that identifies the used CAN controller
   **    chip. Possible values for this member are listed
   **    in the header file cp_cc.h
   */
  _U16 uwControllerType;

   /*!   Defines the number of the interrupt which is used.
   **    If the flag IRQHandler is set to 0, the value of
   **    IRQNumber will be undefined.
   */
  _U16 uwIRQNumber;


  _U16 uwBufferMax;
  _U16 uwRes;

  _U32 ulTimeStampRes;
  _U32 ulCanClock;
  _U32 ulBitrate;


};


/*----------------------------------------------------------------------------*/
/*!
** \typedef CpStruct_HDI
** \brief   Hardware description interface structure
**
** The structure CpHdi_s provides fields to gather information about
** the CAN hardware. This typedef makes it compatible to older versions
** of CANpie. Please use _TsCpHdi for new applications.
*/
typedef struct CpHdi_s  CpStruct_HDI;


/*----------------------------------------------------------------------------*/
/*!
** \typedef _TsCpHdi
** \brief   Hardware description interface structure
**
** The structure CpHdi_s provides fields to gather information about
** the CAN hardware.
*/
typedef struct CpHdi_s  _TsCpHdi;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpBitTiming_s   canpie.h
** \brief   Bit timing structure
**
*/
struct CpBitTiming_s {

   /*!   holds the value from enum CP_BAUD
   */
   _U08     ubBaudSel;

   /*!   Bit timing register 0
   */
   _U08     ubBtr0;

   /*!   Bit timing register 1
   */
   _U08     ubBtr1;

   /*!   Syncronisation jump width
   */
   _U08     ubSjw;


   /*!   Baudrate prescaler
   */
   _U08     ubBrp;
};


typedef struct CpBitTiming_s  _TsCpBitTiming;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpStats_s   canpie.h
** \brief   CAN statistic structure
**
*/
struct CpStatistic_s {

   /*!   Total number of received data & remote frames
   */
   _U32     ulRcvMsgCount;

   /*!   Total number of transmitted data & remote frames
   */
   _U32     ulTrmMsgCount;

   /*!   Total number of error frames
   */
   _U32     ulErrMsgCount;

};

typedef struct CpStatistic_s _TsCpStatistic;


/*----------------------------------------------------------------------------*/
/*!
** \struct  CpState_s   canpie.h
** \brief   CAN state structure
**
*/
struct CpState_s {

   /*!   CAN error state
   */
   _U08     ubCanErrState;

   /*!   Last error type occured
   */
   _U08     ubCanErrType;

   /*!   receive error counter
   */
   _U08     ubCanRcvErrCnt;
   
   /*!   transmit error counter
   */   
   _U08     ubCanTrmErrCnt;

};

typedef struct CpState_s _TsCpState;


#endif   /* _CANPIE_H_   */

