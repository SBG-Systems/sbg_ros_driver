/*!
 *	\file		sbgSwap.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		14 January 2013
 *
 *	\brief		Set of functions used to swap numbers.
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

#ifndef __SBG_SWAP_H__
#define __SBG_SWAP_H__

//----------------------------------------------------------------------//
//- Header (open extern C block)                                       -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif
	
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Internal swap functions                                            -//
//----------------------------------------------------------------------//

/*!
 *	Swap a uint16 number.
 *	\param[in]	x					The uint16 to swap.
 *	\return							The swapped value.
 */
SBG_INLINE uint16 sbgSwap16(uint16 x)
{
	return ((x<<8)|(x>>8));
}

/*!
 *	Swap a uint32 number.
 *	\param[in]	x					The uint32 to swap.
 *	\return							The swapped value.
 */
SBG_INLINE uint32 sbgSwap32(uint32 x)
{
	return ((x << 24) | ((x << 8) & (0xFF0000)) | ((x >> 8) & (0xFF00)) | (x >> 24));
}

/*!
 *	Swap a uint64 number.
 *	\param[in]	x					The uint64 to swap.
 *	\return							The swapped value.
 */
SBG_INLINE uint64 sbgSwap64(uint64 x)
{
	uint32 hi, lo;

	//
	// Separate into high and low 32-bit values
	//
	lo = (uint32)(x&0xFFFFFFFF);
	x >>= 32;
	hi = (uint32)(x&0xFFFFFFFF);

	//
	// Swap each part and rebuild our 64 bit vale
	//
	x = sbgSwap32(lo);
	x <<= 32;
	x |= sbgSwap32(hi);

	return x;
}

/*!
 * Swap a float number.
 * \param[in]	val					The float to swap.
 * \return							The swapped value.
 */
SBG_INLINE float sbgSwapFloat(float val)
{
	FloatNint	tmpFloat;

	//
	// We use a union to do the type punning
	//
	tmpFloat.valF = val;
	tmpFloat.valU = sbgSwap32(tmpFloat.valU);

	//
	// Return the swapped float
	//
	return tmpFloat.valF;
}

/*!
 * Swap a double number.
 * \param[in]	val					The double to swap.
 * \return							The swapped value.
 */
SBG_INLINE double sbgSwapDouble(double val)
{
	DoubleNint	tmpDouble;

	//
	// We use a union to do the type punning
	//
	tmpDouble.valF = val;
	tmpDouble.valU = sbgSwap64(tmpDouble.valU);

	//
	// Return the swapped double
	//
	return tmpDouble.valF;
}

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif
