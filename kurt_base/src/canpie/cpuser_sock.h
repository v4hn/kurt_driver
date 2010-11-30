#ifndef __CPUSER_SOCK_H__
#define __CPUSER_SOCK_H__

//****************************************************************************//
// File:          cpuser_sock.c                                               //
// Description:   CANpie user function implementation via sockets             //
// Author:        Marvin Nebel, Thorben Jensen                                //
// e-mail:        tjensen@uos.de                                              //
//                                                                            //
//============================================================================//
// This program is free software; you can redistribute it and/or modify       //
// it under the terms of the GNU Lesser General Public License as published   //
// by the Free Software Foundation; either version 2.1 of the License, or     //
// (at your option) any later version.                                        //
//============================================================================//
//                                                                            //
// Date        History                                                        //
// ----------  -------------------------------------------------------------- //
// 13.03.2008  Initial version                                                //
//                                                                            //
//****************************************************************************//

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/kernel.h>

#include "can_sock.h" // changed: can/af_can.h to can_sock.h
#include "cp_arch.h"

_TvCpStatus CpUserBaudrate(_TsCpPort * ptsPortV, _U08 ubBaudSelV);

_TvCpStatus CpUserDriverInit( _U08 ubLogIfV,
                              _U16 uwRcvFifoSizeV, _U16 uwTrmFifoSizeV,
                              _U16 uwTrmTimeoutV, _TsCpPort * ptsPortV);

_TvCpStatus CpUserDriverRelease(_TsCpPort * ptsPortV);

_TvCpStatus CpUserMsgRead( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV,
                           _U16 * puwMsgCntV);

_TvCpStatus CpUserMsgWrite(_TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV,
                           _U16 * puwMsgCntV);

#endif

