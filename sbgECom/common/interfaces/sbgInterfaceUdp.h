/*!
 *	\file		sbgInterfaceUdp.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 February 2013
 *
 *	\brief		This file implements an UDP interface.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */

#ifndef __SBG_INTERFACE_UDP_H__
#define __SBG_INTERFACE_UDP_H__

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include "sbgInterface.h"

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

#define SBG_INTERFACE_UDP_PACKET_MAX_SIZE		(1400)

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Structure that stores all internal data used by the UDP interface.
 */
typedef struct _SbgInterfaceUdp
{
	void			*pUdpSocket;					/*!< The socket used to send and / or receive some UDP data. */
	sbgIpAddress	 remoteAddr;					/*!< IP address to send data to. */
	uint32			 remotePort;					/*!< Ethernet port to send data to. */
	uint32			 localPort;						/*!< Ethernet port on which the interface is listening. */
} SbgInterfaceUdp;

//----------------------------------------------------------------------//
//- Operations methods declarations                                    -//
//----------------------------------------------------------------------//

/*!
 *	Initialize an unconnected UDP interface for read and write operations.
 *	An UDP interface can send some data to an output ip address and port and read all received
 *	data on a input port.
 *	\param[in]	pHandle							Pointer on an allocated interface instance to initialize.
 *	\param[in]	remoteAddr						IP address to send data to.
 *	\param[in]	remotePort						Ethernet port to send data to.
 *	\param[in]	localPort						Ehternet port on which the interface is listening.
 *	\return										SBG_NO_ERROR if the interface has been created.
 */
SbgErrorCode sbgInterfaceUdpCreate(SbgInterface *pHandle, sbgIpAddress remoteAddr, uint32 remotePort, uint32 localPort);

/*!
 *	Destroy an interface initialized using sbgInterfaceUdpCreate.
 *	\param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 *	\return										SBG_NO_ERROR if the interface has been closed and released.
 */
SbgErrorCode sbgInterfaceUdpDestroy(SbgInterface *pHandle);

/*!
 *	Define if a socket can send broadcasted packets.
 *	\param[in]	pInterface						Pointer on a valid UDP interface created using sbgInterfaceUdpCreate.
 *	\param[in]	allowBroadcast					Set to true to allow this socket to send broadcasted UDP packets.
 *	\return										SBG_NO_ERROR if the allow broadcast status has been changed.
 */
SbgErrorCode sbgInterfaceUdpAllowBroadcast(SbgInterface *pHandle, bool allowBroadcast);

//----------------------------------------------------------------------//
//- Internal interfaces write/read implementations                     -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SbgErrorCode sbgInterfaceUdpWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SbgErrorCode sbgInterfaceUdpRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* __INTERFACE_UDP_H__ */
