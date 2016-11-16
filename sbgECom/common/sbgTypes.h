/*!
 *	\file		sbgTypes.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that defines all scalar types.
 *	
 *	The platform endianness should be defined here.
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
#ifndef __SBG_TYPES_H__
#define __SBG_TYPES_H__

//----------------------------------------------------------------------//
//- Limits definitions                                                 -//
//----------------------------------------------------------------------//
#define SBG_MIN_INT_8					(-128)
#define SBG_MAX_INT_8					(127)
#define SBG_MAX_UINT_8					(255)

#define SBG_MIN_INT_16					(-32768)
#define SBG_MAX_INT_16					(32767)
#define SBG_MAX_UINT_16					(65535)

#define SBG_MIN_INT_24					(-8388608l)
#define SBG_MAX_INT_24					(8388607l)
#define SBG_MAX_UINT_24					(16777216ul)

#define SBG_MIN_INT_32					(-2147483648l)
#define SBG_MAX_INT_32					(2147483647l)
#define SBG_MAX_UINT_32					(4294967295ul)

#define SBG_MIN_INT_64					(-9223372036854775808ll)
#define SBG_MAX_INT_64					(9223372036854775807ll)
#define SBG_MAX_UINT_64					(0xFFFFFFFFFFFFFFFFull)

//----------------------------------------------------------------------//
//- Scalar types definitions                                           -//
//----------------------------------------------------------------------//
typedef unsigned	char			uint8;		//  8 bits
typedef unsigned	short			uint16;		// 16 bits
typedef unsigned	int				uint32;		// 32 bits
typedef unsigned	int				bool32;		// Boolean that has a fixed size of 32 bits.
typedef unsigned	long long int	uint64;		// 64 bits

typedef signed		char			int8;		//  8 bits
typedef signed		short			int16;		// 16 bits
typedef signed		int				int32;		// 32 bits
typedef signed		long long int	int64;		// 64 bits

// Redefine boolean only if needed (we are using a C compiler)
#ifndef __cplusplus
	typedef	unsigned	char		bool;	//  8 bits
#endif

//----------------------------------------------------------------------//
//- Misc types definitions                                             -//
//----------------------------------------------------------------------//
typedef uint32						sbgIpAddress;					/*!< Define an IP address as an uint32. */

//------------------------------------------------------------------//
//- Type punning safe conversion unions                            -//
//------------------------------------------------------------------//

/*!
 * Used to get a uint32 from a uint8 array.
 */
typedef union _Uint8PtrToUint32Ptr
{
	uint8	*m_pointerUint8;				/*!< Set the address used to access the uint32. */
	uint32	*m_pointerUint32;				/*!< Store the unint32 value. */
} Uint8PtrToUint32Ptr;

/*!
 * Union used to convert a buffer or 2 unit8 two's complement values to a int16
 */
typedef union _Uint8ToInt16
{
	int16	value;
	uint8	buffer[2];
} Uint8ToInt16;

/*!
 * Union that allows type punning (access to a floating point number bits)
 */
typedef union _FloatNint
{
	float valF;
	int32 valI;
	uint32 valU;
} FloatNint;

/*!
 * Union that allows type punning (access to a double number bits)
 */
typedef union _DoubleNint
{
	double valF;
	uint64 valU;
	int64 valI;
} DoubleNint;

/*!
 * Structure that splits a 64bits
 */
typedef struct _Split64
{
	uint32 high;
	uint32 low;
} Split64;

/*!
 * This structure defines a date
 */
typedef struct _DateStructure
{
	uint16	year;
	uint8	month;
	uint8	day;
} DateStructure;

#endif	/* __SBG_TYPES_H__ */
