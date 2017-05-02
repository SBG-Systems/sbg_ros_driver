/*!
 *	\file		sbgInterface.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		10 December 2012
 *
 *	\brief		This file implements the base interface for all Serial and Ethernet ports.
 *
 *	An interface is used to provide a common API for both serial and ethernet ports.
 *	An interface can be opened/closed and some data can be written or read from it.
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

#ifndef __SBG_INTERFACE_H__
#define __SBG_INTERFACE_H__

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Structure definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Define the interface type.
 */
typedef enum _SbgInterfaceType
{
	SBG_IF_TYPE_UNKNOW,								/*!< The interface type is not defined. */
	SBG_IF_TYPE_SERIAL,								/*!< The interface is a serial com port. */
	SBG_IF_TYPE_ETH_UDP,							/*!< The interface is an UDP one. */
	SBG_IF_TYPE_ETH_TCP_IP,							/*!< The interface is an TCP/IP one. */
	SBG_IF_TYPE_FILE								/*!< The interface is a file. */
} SbgInterfaceType;

//----------------------------------------------------------------------//
//- Predefinitions                                                     -//
//----------------------------------------------------------------------//

/*!
 * Interface structure pre-definition.
 */
typedef struct _SbgInterface SbgInterface;

/*!
 * Handle that stores the internal interface handle (ie Serial or Ethernet)
 */
typedef void* SbgInterfaceHandle;

//----------------------------------------------------------------------//
//- Callbacks definitions                                              -//
//----------------------------------------------------------------------//

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if some bytes have been written successfully.
 */
typedef SbgErrorCode (*SbgInterfaceWriteFunc)(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if some bytes have been read successfully.
 */
typedef SbgErrorCode (*SbgInterfaceReadFunc)(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead);

/*!
 * Returns true if the interface is working correctly.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \return												true if this interface is working correctly.
 */
typedef bool (*SbgInterfaceIsValidFunc)(SbgInterface *pHandle);

//----------------------------------------------------------------------//
//- Structures definitions                                             -//
//----------------------------------------------------------------------//

/*!
 * Interface definition that stores methods used to communicate on the interface.
 */
struct _SbgInterface
{
	SbgInterfaceHandle			 handle;							/*!< Internal interface handle used to access the media. */
	SbgInterfaceType			 type;								/*!< The interface type. */

	SbgInterfaceWriteFunc		pWriteFunc;							/*!< Pointer on the method used to write some data to this interface. */
	SbgInterfaceReadFunc		pReadFunc;							/*!< Pointer on the method used to read some data to this interface. */
	SbgInterfaceIsValidFunc		pIsValidFunc;						/*!< Pointer on the method used to test if the interface is working correctly.*/
};

//----------------------------------------------------------------------//
//- Inline operations methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Initialize an interface strcture to zero.
 *	\param[in]	pHandle									Handle on an allocated interface to initialize to zero.
 */
SBG_INLINE void sbgInterfaceZeroInit(SbgInterface *pHandle)
{
	//
	//	Test input argument
	//
	SBG_ASSERT(pHandle);

	//
	// Initialize all fields to NULL
	//
	pHandle->handle			= NULL;
	pHandle->type			= SBG_IF_TYPE_UNKNOW;
	pHandle->pWriteFunc		= NULL;
	pHandle->pReadFunc		= NULL;
	pHandle->pIsValidFunc	= NULL;
}

/*!
 * Returns true if this interface seems to be up and running.
 * \param[in]	pHandle									Handle on an interface to test (if pHandle == NULL, retruns false).
 * \return												true if this interface seems to be up and running.
 */
SBG_INLINE bool sbgInterfaceIsValid(SbgInterface *pHandle)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pHandle);

	//
	// To be valid, an interface should have a valid name, type and read / write method
	// Optionally, we can also ask the interface to return a validity flag
	//
	if ( (pHandle->type != SBG_IF_TYPE_UNKNOW) && (pHandle->pWriteFunc) && (pHandle->pReadFunc) )
	{
		//
		// Check if this interface has a specific valid method
		//
		if (pHandle->pIsValidFunc)
		{
			//
			// Return the status of the interface specific valid method
			//
			return pHandle->pIsValidFunc(pHandle);
		}
		else
		{
			//
			// This interface is valid
			//
			return true;
		}
	}
	else
	{
		//
		// This interface is invalid
		//
		return false;
	}
}

/*!
 * Try to write some data to an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that contains the data to write
 * \param[in]	bytesToWrite							Number of bytes we would like to write.
 * \return												SBG_NO_ERROR if all bytes have been written successfully.
 */
SBG_INLINE SbgErrorCode sbgInterfaceWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite)
{
	//
	//	Test input arguments
	//
	SBG_ASSERT(pHandle);
	SBG_ASSERT(pBuffer);

	//
	// Call the correct write method according to the interface
	//
	return pHandle->pWriteFunc(pHandle, pBuffer, bytesToWrite);
}

/*!
 * Try to read some data from an interface.
 * \param[in]	pHandle									Valid handle on an initialized interface.
 * \param[in]	pBuffer									Pointer on an allocated buffer that can hold at least bytesToRead bytes of data.
 * \param[out]	pReadBytes								Pointer on an uint32 used to return the number of read bytes.
 * \param[in]	bytesToRead								Number of bytes we would like to read.
 * \return												SBG_NO_ERROR if no error occurs, please check the number of received bytes.
 */
SBG_INLINE SbgErrorCode sbgInterfaceRead(SbgInterface *pHandle, void *pBuffer, size_t *pReadBytes, size_t bytesToRead)
{
	//
	//	Test input arguments
	//
	SBG_ASSERT(pHandle);
	SBG_ASSERT(pBuffer);
	SBG_ASSERT(pReadBytes);

	//
	// Call the correct read method according to the interface
	//
	return pHandle->pReadFunc(pHandle, pBuffer, pReadBytes, bytesToRead);
}

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif /* __INTERFACE_H__ */
