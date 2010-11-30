//****************************************************************************//
// File:          can_sock.h                                                  //
// Description:   Socket interface to CANpie kernel driver                    //
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
// 05.07.2004  Initial version                                                //
//                                                                            //
//****************************************************************************//


#ifndef  _CAN_SOCK_H_
#define  _CAN_SOCK_H_


//------------------------------------------------------------------------------
// SVN  $Date: 2007-07-29 22:03:46 +0200 (Sun, 29 Jul 2007) $
// SVN  $Rev: 49 $ --- $Author: microcontrol $
//------------------------------------------------------------------------------


//-----------------------------------------------------------------------------
/*!   
** \file    can_sock.h
**
** This file holds definitions and structures for the CAN socket interface.
** The address family AF_CAN ist defined here, but should be moved to the 
** header file <linux/socket.h>.
*/


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/

#include <linux/net.h>              // struct socket, net_proto_family
#include <linux/socket.h>
#include <linux/sockios.h>

#include <linux/types.h>

#include "canpie.h"




/*----------------------------------------------------------------------------*\
** Definitions                                                                **
**                                                                            **
\*----------------------------------------------------------------------------*/

//-------------------------------------------------------------------
// The address family should be defined in the header file 
// <linux/socket.h>. As long as this in not the standard in the
// kernel tree we define it here.
//
#ifndef  AF_CAN
#define  AF_CAN      29       // address family
#define  PF_CAN      AF_CAN
#endif


enum CAN_PROTO {
   CAN_PROTO_RAW = 0,
   CAN_PROTO_COM,          // CANopen master
   CAN_PROTO_MAX
};


/*----------------------------------------------------------------------------*/
/*!
** \enum    SIOC_CAN
** \brief   Socket I/O operations
**
** The following CAN specific socket I/O operations are defined. Please
** note that the number is limited to 16 specific opcodes (refer to
** sockios.h).  
*/
enum SIOC_CAN {
   SIOC_CAN_FIRST = SIOCDEVPRIVATE,
   
   /*!   Set baudrate of CAN controller 
   */
   SIOC_CAN_SET_BAUDRATE = SIOC_CAN_FIRST,
   
   /*!   Set bittiming of CAN controller 
   */
   SIOC_CAN_SET_BITTIMING,
   
   /*!   Set mode of CAN controller, refer to CpCoreCanMode() 
   */
   SIOC_CAN_SET_MODE,
   
   /*!   Get state of CAN controller, refer to CpCoreCanStatus() 
   */
   SIOC_CAN_GET_STATE,

   /*!   Get hardware info, refer to CpCoreHDI() 
   */
   SIOC_CAN_GET_HW_INFO,

   /*!   Get statistics, refer to CpCoreStatistic() 
   */
   SIOC_CAN_GET_STATS,

   //SIOC_CAN_GET_CHANNEL_MAX,
   
   SIOC_CAN_SET_FIFOSIZE,

      
   SIOC_CAN_LAST = SIOC_CAN_SET_FIFOSIZE
};


/*----------------------------------------------------------------------------*\
** Structures                                                                 **
**                                                                            **
\*----------------------------------------------------------------------------*/





/*----------------------------------------------------------------------------*/
/*!
** \struct  can_sock   can_sock.h
** \brief   Protocol dependent socket structure
**
** The can_sock structure provides the protocol dependent information
** for the CAN socket. 
*/
struct can_sock {

   /*!
   ** Backlink to the socket structure
   */
   struct sock *     ptsSock;
   
   /*!
   ** Link to the CAN port
   */   
   _TsCpPort *       ptsCanPort;

      
   /*!
   ** CAN identifier of this socket,
   ** setting CAN_FLAG_ALL ....
   */
   int32_t             can_id;

   /*!
   ** size of the receive FIFO in number of messages (NOT bytes)
   */
   int32_t             rcv_fifo_size;
   
   /*!
   ** size of the transmit FIFO in number of messages (NOT bytes)
   */
   int32_t             trm_fifo_size;
   
};


struct sockaddr_can {
   
   /*!
   ** address family (AF_CAN)
   */
   sa_family_t    can_family;

   
   /*!
   ** logical CAN interface number, compliant to CANpie 
   ** first CAN interface number starts at 0
   */
   int32_t        can_if;

   /* CAN_PROTO_RAW-specific */
   int32_t        can_id;
};

// #define can_sk(__sk) ((struct can_sock *) __sk)



/*----------------------------------------------------------------------------*\
** Function prototypes                                                        **
**                                                                            **
\*----------------------------------------------------------------------------*/


/*!
** \brief   initialize CAN socket interface
**
** This function registers the AF_CAN socket family.
*/
int   can_socket_init(void);



/*!
** \brief   release CAN socket interface
**
** This function releases the AF_CAN socket family.
*/
void  can_socket_cleanup(void);


#endif   /* _CAN_SOCK_H_ */
