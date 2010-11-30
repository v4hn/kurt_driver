//****************************************************************************//
// File:          cpuser_sock.c                                               //
// Description:   CANpie user function implementation via sockets             //
// Author:        Uwe Koppe                                                   //
// e-mail:        koppe@microcontrol.net                                      //
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
// 24.06.2005  Initial version                                                //
//                                                                            //
//****************************************************************************//


//------------------------------------------------------------------------------
// CVS version information:
// $Id: cpuser_sock.c,v 1.2 2005/07/13 09:42:54 microcontrol Exp $
//------------------------------------------------------------------------------


/*----------------------------------------------------------------------------*\
** Include files                                                              **
**                                                                            **
\*----------------------------------------------------------------------------*/

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/kernel.h>
//#include <linux/module.h>
//#include "/usr/src/linux/kernel/printk.c"

#include "can_sock.h" // changed: can/af_can.h to can_sock.h
//#include <can/cpuser.h>
#include "cp_arch.h"

#include "cpuser_sock.h" // changed: inserted line

//#include "can.h"

/*----------------------------------------------------------------------------*\
** Definitions                                                                **
**                                                                            **
\*----------------------------------------------------------------------------*/



//----------------------------------------------------------------------------//
// CpUserBaudrate()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpUserBaudrate(_TsCpPort * ptsPortV, _U08 ubBaudSelV)
{
   _TvCpStatus       tuStatusT;
   _TsCpBitTiming    tsBitrateT;
   
   //----------------------------------------------------------------
   // check pointer to port structure
   //
   if(ptsPortV == 0L)
   {
      return(CpErr_PARAM);
   }

      
   tsBitrateT.ubBaudSel = ubBaudSelV;
   

   printf("[CpUserBaudrate] ... before call of ioctl %d \n", (ptsPortV->slQueue));
   if( ioctl(ptsPortV->slQueue, SIOC_CAN_SET_BAUDRATE, &tsBitrateT) < 0) // changed: SIOC_CAN_SET_BITRATE to SIOC_CAN_SET_BAUDRATE
   {
      tuStatusT = CpErr_BAUDRATE;
   }
   else
   {
      tuStatusT = CpErr_OK;
   }
   
   return(tuStatusT);
}


//----------------------------------------------------------------------------//
// CpUserDriverInit()                                                         //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpUserDriverInit( _U08 ubLogIfV, 
                              _U16 uwRcvFifoSizeV, _U16 uwTrmFifoSizeV, 
                              _U16 uwTrmTimeoutV, _TsCpPort * ptsPortV)
{
 
   int                  slSockDescT;
   _TvCpStatus          tuStatusT;
   struct sockaddr_can  tsSockAddrCanT;
   struct sockaddr   *  ptsSockAddrT;         
   //printf("%i \n", AF_CAN);
   
   //----------------------------------------------------------------
   // check pointer to port structure
   //
   if(ptsPortV == 0L)
   {
      return(CpErr_PARAM);
   }
   
   //printf("AF_CAN %i \n", AF_CAN);
   slSockDescT = socket(AF_CAN, SOCK_STREAM, 0);

   printf("slSockDescT: %i \n", slSockDescT);
   if(slSockDescT < 0)
   {
      tuStatusT = CpErr_INIT_FAIL;
      fprintf(stderr, "[After socket(.)] ... Can't open socket AF_CAN\n");
   }
   else
   {
      tsSockAddrCanT.can_family = AF_CAN;
      tsSockAddrCanT.can_if     = ubLogIfV;
      tsSockAddrCanT.can_id     = 0x00FFFFFF;
      ptsSockAddrT = (struct sockaddr*) (&tsSockAddrCanT);
   
      if(connect(slSockDescT, ptsSockAddrT, sizeof(tsSockAddrCanT)) < 0)
      {
         tuStatusT = CpErr_INIT_FAIL;
         fprintf(stderr, "Can't connect to socket AF_CAN\n");
      }
      else
      {
         tuStatusT = CpErr_OK;
         ptsPortV->slLogIf = ubLogIfV;
         ptsPortV->slQueue = slSockDescT;
      }
   
   }

   return(tuStatusT);
}                              


//----------------------------------------------------------------------------//
// CpUserDriverRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpUserDriverRelease(_TsCpPort * ptsPortV)               
{
   //----------------------------------------------------------------
   // check pointer to port structure
   //
   if(ptsPortV == 0L)
   {
      return(CpErr_PARAM);
   }
   
   //----------------------------------------------------------------
   // close the requested port
   //
   close(ptsPortV->slQueue);

   return(CpErr_OK);
}


//----------------------------------------------------------------------------//
// CpUserMsgRead()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpUserMsgRead( _TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV,   
                           _U16 * puwMsgCntV)
{
   
   _TvCpStatus          tuStatusT;
   int                  slReadResultT;          // result of read operation
   
   //----------------------------------------------------------------
   // check pointer to port structure
   //
   if(ptsPortV == 0L)
   {
      return(CpErr_PARAM);
   }
   
    
   slReadResultT = read( ptsPortV->slQueue, 
                         ptsCanMsgV, 
                         (*puwMsgCntV) * sizeof(_TsCpCanMsg)   );

   if(slReadResultT <= 0)
   {
      tuStatusT = CpErr_RCV_EMPTY;
   }
   else
   {
      *puwMsgCntV = slReadResultT / sizeof(_TsCpCanMsg);
      tuStatusT = CpErr_OK;
   }
   
   
   return(tuStatusT);
   


}                           


//----------------------------------------------------------------------------//
// CpUserMsgWrite()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
_TvCpStatus CpUserMsgWrite(_TsCpPort * ptsPortV, _TsCpCanMsg * ptsCanMsgV, 
                           _U16 * puwMsgCntV)
{
   _TvCpStatus          tuStatusT;
   int                  slWriteResultT;          // result of write operation
   
   //----------------------------------------------------------------
   // check pointer to port structure
   //
   if(ptsPortV == 0L)
   {
      return(CpErr_PARAM);
   }
   
   slWriteResultT = write(  ptsPortV->slQueue, 
                            ptsCanMsgV, 
                           (*puwMsgCntV) * sizeof(_TsCpCanMsg)   );

   if(slWriteResultT < 0)
   {
      tuStatusT = CpErr_TRM_FULL;
   }
   else
   {
      *puwMsgCntV = slWriteResultT / sizeof(_TsCpCanMsg);
      tuStatusT = CpErr_OK;
   }
   
   return(tuStatusT);
}                           


