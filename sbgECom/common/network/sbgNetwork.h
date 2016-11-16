/*!
 *	\file		sbgNetwork.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		15 September 2015
 *
 *	\brief		Useful methods for Network handling such as ip addresses.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2015, SBG Systems SAS. All rights reserved.
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

#ifndef __SBG_NETWORK_H__
#define __SBG_NETWORK_H__

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- General path definitions                                           -//
//----------------------------------------------------------------------//

/*!
 *	Build an IP V4 address in the form a.b.c.d
 *	\param[in]	a				First 8 bits IP address.
 *	\param[in]	b				Second 8 bits IP address.
 *	\param[in]	c				Third 8 bits IP address.
 *	\param[in]	d				Last 8 bits IP address.
 *	\return						An initialized IP address object.
 */
#define SBG_IP_ADDR(a, b, c, d) (((a&0xFF) << 24) | ((b&0xFF) << 16) | ((c&0xFF) << 8) | (d&0xFF))

/*!
 * Return the first 8 bits of an IP v4 address.
 * \param[in]	ipAddr				An sbgIpAddress to convert.
 * \return							The first 8 bits of the IP address.
 */
#define SBG_IP_ADDR_GET_A(ipAddr)		(((ipAddr) & 0xFF000000) >> 24)

/*!
 * Return the second 8 bits of an IP v4 address.
 * \param[in]	ipAddr				An sbgIpAddress to convert.
 * \return							The second 8 bits of the IP address.
 */
#define SBG_IP_ADDR_GET_B(ipAddr)		(((ipAddr) & 0x00FF0000) >> 16)

/*!
 * Return the third 8 bits of an IP v4 address.
 * \param[in]	ipAddr				An sbgIpAddress to convert.
 * \return							The third 8 bits of the IP address.
 */
#define SBG_IP_ADDR_GET_C(ipAddr)		(((ipAddr) & 0x0000FF00) >> 8)

/*!
 * Return the last 8 bits of an IP v4 address.
 * \param[in]	ipAddr				An sbgIpAddress to convert.
 * \return							The last 8 bits of the IP address.
 */
#define SBG_IP_ADDR_GET_D(ipAddr)		((ipAddr) & 0x000000FF)

//----------------------------------------------------------------------//
//- IP manipulation methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Convert an ip to a string of the form A.B.C.D
 * \param[in]	ipAddr						IP address to convert to a string.
 * \param[out]	pBuffer						Pointer on an allocated buffer than can hold ip address as a string.
 * \param[in]	maxSize						Maximum number of chars that can be stored in pBuffer including the NULL char.
 */
SBG_INLINE void sbgNetworkIpToString(sbgIpAddress ipAddr, char *pBuffer, size_t maxSize)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pBuffer);
	SBG_ASSERT(maxSize >= 16);

	SBG_UNUSED_PARAMETER(maxSize);

	//
	// Write the IP address
	//
	sprintf(pBuffer, "%u.%u.%u.%u", SBG_IP_ADDR_GET_A(ipAddr), SBG_IP_ADDR_GET_B(ipAddr), SBG_IP_ADDR_GET_C(ipAddr), SBG_IP_ADDR_GET_D(ipAddr));
}

/*!
 * Convert an ip address stored in a string of the form A.B.C.D to an sbgIpAddress object.
 * \param[in]	pBuffer						IP address as a string of the form A.B.C.D
 * \return									IP address to parsed from the string.
 */
SBG_INLINE sbgIpAddress sbgNetworkIpFromString(char *pBuffer)
{
	uint32	ipAddrA;
	uint32	ipAddrB;
	uint32	ipAddrC;
	uint32	ipAddrD;

	//
	// Check input arguments
	//
	SBG_ASSERT(pBuffer);

	//
	// Parse input arguments
	//
	sscanf(pBuffer, "%u.%u.%u.%u", &ipAddrA, &ipAddrB, &ipAddrC, &ipAddrD);

	//
	// Build the full IP address
	//
	return SBG_IP_ADDR(ipAddrA, ipAddrB, ipAddrC, ipAddrD);
}

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* __SBG_NETWORK_H__ */
