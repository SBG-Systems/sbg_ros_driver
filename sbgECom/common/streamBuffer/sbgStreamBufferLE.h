/*!
 *	\file		sbgStreamBufferLE.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		17 February 2015
 *
 *	\brief		Specific method of stream buffer for little endian readings/writings.
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

#ifndef __SBG_STREAM_BUFFER_LE_H__
#define __SBG_STREAM_BUFFER_LE_H__

#include "sbgStreamBufferCommon.h"

//----------------------------------------------------------------------//
//- Read operations methods                                            -//
//----------------------------------------------------------------------//

/*!
 * Read an int16 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int16 sbgStreamBufferReadInt16LE(SbgStreamBuffer *pHandle)
{
	int16 bytesValues[2];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				bytesValues[0] = *((int16*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int16);

				return bytesValues[0];
			#else
				//
				// Read the each bytes
				//
				bytesValues[0] = *(pHandle->pCurrentPtr++);
				bytesValues[1] = *(pHandle->pCurrentPtr++);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return bytesValues[1] | (bytesValues[0] << 8);
				#else			
					return bytesValues[0] | (bytesValues[1] << 8);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an uint16 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint16 sbgStreamBufferReadUint16LE(SbgStreamBuffer *pHandle)
{
	uint16 bytesValues[2];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				bytesValues[0] = *((uint16*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint16);

				return bytesValues[0];
			#else
				//
				// Read the each bytes
				//
				bytesValues[0] = *(pHandle->pCurrentPtr++);
				bytesValues[1] = *(pHandle->pCurrentPtr++);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return bytesValues[1] | (bytesValues[0] << 8);
				#else
					return bytesValues[0] | (bytesValues[1] << 8);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an int24 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int32 sbgStreamBufferReadInt24LE(SbgStreamBuffer *pHandle)
{
	int32 bytesValues[3];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(int8))
		{
			//
			// Read the each bytes
			//
			bytesValues[0] = *(pHandle->pCurrentPtr++);
			bytesValues[1] = *(pHandle->pCurrentPtr++);
			bytesValues[2] = *(pHandle->pCurrentPtr++);
			
			//
			// Store data according to platform endianness
			//
			#if (SBG_CONFIG_BIG_ENDIAN == 1)
				return bytesValues[2] | (bytesValues[1] << 8) | (bytesValues[0] << 16);
			#else
				return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16);
			#endif	
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an uint24 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint32 sbgStreamBufferReadUint24LE(SbgStreamBuffer *pHandle)
{
	uint32 bytesValues[3];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(uint8))
		{
			//
			// Read the each bytes
			//
			bytesValues[0] = *(pHandle->pCurrentPtr++);
			bytesValues[1] = *(pHandle->pCurrentPtr++);
			bytesValues[2] = *(pHandle->pCurrentPtr++);
			
			//
			// Store data according to platform endianness
			//
			#if (SBG_CONFIG_BIG_ENDIAN == 1)
				return bytesValues[2] | (bytesValues[1] << 8) | (bytesValues[0] << 16);
			#else
				return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16);
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an int32 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int32 sbgStreamBufferReadInt32LE(SbgStreamBuffer *pHandle)
{
	int32 bytesValues[4];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				bytesValues[0] = *((int32*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int32);

				return bytesValues[0];
			#else
				//
				// Read the each bytes
				//
				bytesValues[0] = *(pHandle->pCurrentPtr++);
				bytesValues[1] = *(pHandle->pCurrentPtr++);
				bytesValues[2] = *(pHandle->pCurrentPtr++);
				bytesValues[3] = *(pHandle->pCurrentPtr++);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return bytesValues[3] | (bytesValues[2] << 8) | (bytesValues[1] << 16) | (bytesValues[0] << 24);
				#else
					return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an uint32 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint32 sbgStreamBufferReadUint32LE(SbgStreamBuffer *pHandle)
{
	uint32 bytesValues[4];

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				bytesValues[0] = *((uint32*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint32);

				return bytesValues[0];
			#else
				//
				// Read the each bytes
				//
				bytesValues[0] = *(pHandle->pCurrentPtr++);
				bytesValues[1] = *(pHandle->pCurrentPtr++);
				bytesValues[2] = *(pHandle->pCurrentPtr++);
				bytesValues[3] = *(pHandle->pCurrentPtr++);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return bytesValues[3] | (bytesValues[2] << 8) | (bytesValues[1] << 16) | (bytesValues[0] << 24);
				#else			
					return bytesValues[0] | (bytesValues[1] << 8) | (bytesValues[2] << 16) | (bytesValues[3] << 24);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0;
}

/*!
 * Read an int64 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE int64 sbgStreamBufferReadInt64LE(SbgStreamBuffer *pHandle)
{
	int64 lowPart;
	int64 highPart;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				lowPart = *((int64*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int64);

				return lowPart;
			#else
				//
				// Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
				//
				lowPart = sbgStreamBufferReadUint32LE(pHandle);
				highPart = sbgStreamBufferReadUint32LE(pHandle);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return (lowPart << 32) | highPart;
				#else			
					return lowPart | (highPart << 32);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0ll;
}

/*!
 * Read an uint64 from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE uint64 sbgStreamBufferReadUint64LE(SbgStreamBuffer *pHandle)
{
	uint64 lowPart;
	uint64 highPart;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				// Read the current value
				//
				lowPart = *((uint64*)pHandle->pCurrentPtr);

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint64);

				return lowPart;
			#else
				//
				// Read 64 bit value using two 32 bits read to avoid too much 64 bits operations
				//
				lowPart = sbgStreamBufferReadUint32LE(pHandle);
				highPart = sbgStreamBufferReadUint32LE(pHandle);

				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					return (lowPart << 32) | highPart;
				#else
					return lowPart | (highPart << 32);
				#endif
			#endif
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0ull;
}

/*!
 * Read an float from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE float sbgStreamBufferReadFloatLE(SbgStreamBuffer *pHandle)
{
	FloatNint floatInt;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(float))
		{
			//
			// Read the float as an uint32
			//
			floatInt.valU = sbgStreamBufferReadUint32LE(pHandle);

			//
			// Return the float using an union to avoid compiller cast
			//
			return floatInt.valF;
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0.0f;
}

/*!
 * Read an double from a stream buffer (Little endian version).
 * \param[in]	pHandle				Valid stream buffer handle that supports read operations.
 * \return							The read value or 0 if we have an error.
 */
SBG_INLINE double sbgStreamBufferReadDoubleLE(SbgStreamBuffer *pHandle)
{
	DoubleNint doubleInt;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(double))
		{
			//
			// Read the float as an uint64
			//
			doubleInt.valU = sbgStreamBufferReadUint64LE(pHandle);

			//
			// Return the double using an union to avoid compiller cast
			//
			return doubleInt.valF;
		}
		else
		{
			//
			// We have a buffer overflow so return 0
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// If we are here, it means we have an error so return 0
	//
	return 0.0;
}

//----------------------------------------------------------------------//
//- Write operations methods                                           -//
//----------------------------------------------------------------------//

/*!
 * Write an int16 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt16LE(SbgStreamBuffer *pHandle, int16 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int16))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
								//
				//	Write the value
				//
				*((int16*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int16);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an uint16 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint16LE(SbgStreamBuffer *pHandle, uint16 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint16))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				//	Write the value
				//
				*((uint16*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint16);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}


/*!
 * Write an int24 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt24LE(SbgStreamBuffer *pHandle, int32 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Make sure that the value is within 24 bit bonds
		//
		if ( (value >= SBG_MIN_INT_24) && (value <= SBG_MAX_INT_24) )
		{		
			//
			// Test if we can access this item
			//
			if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(int8))
			{
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
				#endif
			}
			else
			{
				//
				// We are accessing a data that is outside the stream buffer
				//
				pHandle->errorCode = SBG_BUFFER_OVERFLOW;
			}
		}
		else
		{
			//
			// The input value is not within a 24 bit integer bounds
			//
			pHandle->errorCode = SBG_INVALID_PARAMETER;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an uint24 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint24LE(SbgStreamBuffer *pHandle, uint32 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Make sure that the value is within 24 bit bonds
		//
		if (value <= SBG_MAX_UINT_24)
		{		
			//
			// Test if we can access this item
			//
			if (sbgStreamBufferGetSpace(pHandle) >= 3*sizeof(uint8))
			{
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
				#endif
			}
			else
			{
				//
				// We are accessing a data that is outside the stream buffer
				//
				pHandle->errorCode = SBG_BUFFER_OVERFLOW;
			}
		}
		else
		{
			//
			// The input value is not within a 24 bit integer bounds
			//
			pHandle->errorCode = SBG_INVALID_PARAMETER;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an int32 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt32LE(SbgStreamBuffer *pHandle, int32 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int32))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				//	Write the value
				//
				*((int32*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int32);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an uint32 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint32LE(SbgStreamBuffer *pHandle, uint32 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint32))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				//	Write the value
				//
				*((uint32*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint32);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an int64 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteInt64LE(SbgStreamBuffer *pHandle, int64 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(int64))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				//	Write the value
				//
				*((int64*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(int64);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an uint64 into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteUint64LE(SbgStreamBuffer *pHandle, uint64 value)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// Test if we can access this item
		//
		if (sbgStreamBufferGetSpace(pHandle) >= sizeof(uint64))
		{
			//
			// Test if the platform supports un-aligned access and if the endianness is the same
			//
			#if (SBG_CONFIG_UNALIGNED_ACCESS_AUTH == 1) && (SBG_CONFIG_BIG_ENDIAN == 0)
				//
				//	Write the value
				//
				*((uint64*)(pHandle->pCurrentPtr)) = value;

				//
				//	Increment the current pointer
				//
				pHandle->pCurrentPtr += sizeof(uint64);
			#else
				//
				// Store data according to platform endianness
				//
				#if (SBG_CONFIG_BIG_ENDIAN == 1)
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value);
				#else
					*(pHandle->pCurrentPtr++) = (uint8)(value);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 8);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 16);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 24);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 32);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 40);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 48);
					*(pHandle->pCurrentPtr++) = (uint8)(value >> 56);
				#endif
			#endif
		}
		else
		{
			//
			// We are accessing a data that is outside the stream buffer
			//
			pHandle->errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	return pHandle->errorCode;
}

/*!
 * Write an float into a stream buffer (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteFloatLE(SbgStreamBuffer *pHandle, float value)
{
	FloatNint floatInt;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// We use an union to avoid compiler cast
		//
		floatInt.valF = value;

		//
		// Write this float as an uint32
		//
		return sbgStreamBufferWriteUint32LE(pHandle, floatInt.valU);
	}

	return pHandle->errorCode;
}

/*!
 * Write an double into a stream buffer. (Little Endian Version).
 * \param[in]	pHandle				Valid stream buffer handle that supports write operations.
 * \param[in]	value				The value to write.
 * \return							SBG_NO_ERROR if the value has been successfully written.
 */
SBG_INLINE SbgErrorCode sbgStreamBufferWriteDoubleLE(SbgStreamBuffer *pHandle, double value)
{
	DoubleNint doubleInt;

	//
	// Check input parameters
	//
	SBG_ASSERT(pHandle);

	//
	// Test if we haven't already an error
	//
	if (pHandle->errorCode == SBG_NO_ERROR)
	{
		//
		// We use an union to avoid compiler cast
		//
		doubleInt.valF = value;

		//
		// Write this float as an uint64
		//
		return sbgStreamBufferWriteUint64LE(pHandle, doubleInt.valU);
	}

	return pHandle->errorCode;
}

#endif /* __SBG_STREAM_BUFFER_LE_H__ */
