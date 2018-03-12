/*!
 *	\file		sbgDefines.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that contains all common definitions.
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
#ifndef __SBG_DEFINES_H__
#define __SBG_DEFINES_H__

//----------------------------------------------------------------------//
//- Global definitions                                                 -//
//----------------------------------------------------------------------//
#ifndef SBG_DISABLE
	#define SBG_DISABLE	(0)
#endif

#ifndef SBG_ENABLE
	#define SBG_ENABLE	(1)
#endif

#ifndef FALSE
    #define FALSE        (0)
#endif

#ifndef TRUE
    #define TRUE        (1)
#endif

#ifndef NULL
	#define NULL		(0)
#endif

#ifndef SBG_INVALID_HANDLE
	#define SBG_INVALID_HANDLE	(0x00000000u)
#endif

#ifndef SBG_NOT_FOUND
	#define SBG_NOT_FOUND		(0xFFFFFFFFu)
#endif

#ifndef SBG_UNDEFINED
	#define SBG_UNDEFINED		(0xFFFFFFFFu)
#endif

#ifndef SBG_ARRAY_SIZE
	#define SBG_ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#endif

/*!
 *	__BASE_FILE__ is gcc specific
 */
#ifndef __GNUC__
#ifndef __BASE_FILE__
	#define __BASE_FILE__ __FILE__
#endif
#endif

#ifdef __cplusplus
	#define SBG_DELETE(p)		if (p){delete (p); (p) = NULL;}
	#define SBG_DELETE_ARRAY(p)	if (p){delete[] (p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#else
	#define SBG_DELETE			if (p){free(p); (p) = NULL;}
	#define SBG_DELETE_ARRAY	if (p){free(p); (p) = NULL;}
	#define SBG_FREE(p)			if (p){free(p); (p) = NULL;}
	#define SBG_FREE_ARRAY(p)	if (p){free(p); (p) = NULL;}
#endif

//----------------------------------------------------------------------//
//- Compiller definitions                                              -//
//----------------------------------------------------------------------//

/*!
 *	Macro used to abstract the compiler specific inline keyword.
 */
#ifndef SBG_INLINE
	#if defined(_WIN32) || defined(WIN32)
		#define SBG_INLINE			__inline
	#else
		#define SBG_INLINE          static inline
	#endif
#endif

/*!
 *	Macro used to avoid compiler warning when a variable is not used.
 */
#ifndef SBG_UNUSED_PARAMETER
	#define SBG_UNUSED_PARAMETER(x)		(void)(x)
#endif

//----------------------------------------------------------------------//
//- Macro used to defined packed structures                            -//
//----------------------------------------------------------------------//

/*!
 * This macro is used to define a new section of packed structures.
 * All structures defined after this macro will be packed.
 */
#ifdef __GNUC__
	#define SBG_BEGIN_PACKED()
#elif defined(_MSC_VER)
	#define SBG_BEGIN_PACKED()	__pragma(pack(push, 1))
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * This macro is used to specify that a structure is packed.
 */
#ifdef __GNUC__
	#define SBG_PACKED			__attribute__((packed))
#elif defined(_MSC_VER)
	#define SBG_PACKED
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

/*!
 * This macro is used to close the section of packed structures and return to the default packing.
 */
#ifdef __GNUC__
	#define SBG_END_PACKED()
#elif defined(_MSC_VER)
	#define SBG_END_PACKED()		__pragma(pack(pop))
#else
	#error you must byte-align these structures with the appropriate compiler directives
#endif

//----------------------------------------------------------------------//
//- Deprecation definitions				                               -//
//----------------------------------------------------------------------//

/*!
 *	Macro used to indicate that a function is deprecated.
 */
#ifdef __GNUC__
	#define SBG_DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
	#define SBG_DEPRECATED(func) __declspec(deprecated) func
#else
	//#warning "WARNING: You need to implement SBG_DEPRECATED for this compiler"
	#define SBG_DEPRECATED(func) func
#endif

/*!
 *	Macro used to indicate that a macro is deprecated.
 */
#ifdef __GNUC__
	#define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#elif defined(_MSC_VER)
#define SBG_DEPRECATED_MACRO(func) __pragma(deprecated(func))
#else
	//#warning "WARNING: You need to implement SBG_DEPRECATED_MACRO for this compiler"
	#define SBG_DEPRECATED_MACRO(func) func
#endif

//----------------------------------------------------------------------//
//- Basic maths definitions                                            -//
//----------------------------------------------------------------------//
#ifndef SBG_PI
	#define SBG_PI 3.14159265358979323846
#endif

#ifndef SBG_PI_F
	#define SBG_PI_F 3.14159265358979323846f
#endif

/*!
 *	Returns the maximum between a and b
 *	\param[in]	a					First operand.
 *	\param[in]	b					Second operand.
 *	\return							The maximum between a and b.
 */
#ifndef sbgMax
	#define sbgMax(a,b)            (((a) > (b)) ? (a) : (b))
#endif

/*!
 *	Returns the minimum between a and b
 *	\param[in]	a					First operand.
 *	\param[in]	b					Second operand.
 *	\return							The minimum between a and b.
 */
#ifndef sbgMin
	#define sbgMin(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/*!
 * Clamp a value between minValue and maxValue ie minValue <= value <= maxValue
 * \param[in]	value				First operand.
 * \param[in]	minValue			First operand.
 * \param[in]	maxValue			Second operand.
 * \return							The clamped value.
 */
#ifndef sbgClamp
	#define sbgClamp(value, minValue, maxValue)            (((value) < (minValue))?(minValue): ((value) > (maxValue)?maxValue:value))
#endif

/*!
 *	Convert an angle from radians to degrees using double precision.
 *	\param[in]	angle				The angle to convert in radians.
 *	\return							The converted angle in degrees.
 */
SBG_INLINE double sbgRadToDegD(double angle)
{
	return angle * 180.0 / SBG_PI;
}

/*!
 *	Convert an angle from degrees to radians using double precision.
 *	\param[in]	angle				The angle to convert in degrees.
 *	\return							The converted angle in radians.
 */
SBG_INLINE double sbgDegToRadD(double angle)
{
	return angle * SBG_PI / 180.0;
}

/*!
 *	Convert an angle from radians to degrees using single (float) precision.
 *	\param[in]	angle				The angle to convert in radians.
 *	\return							The converted angle in degrees.
 */
SBG_INLINE float sbgRadToDegF(float angle)
{
	return angle * 180.0f / SBG_PI_F;
}

/*!
 *	Convert an angle from degrees to radians using single (float) precision.
 *	\param[in]	angle				The angle to convert in degrees.
 *	\return							The converted angle in radians.
 */
SBG_INLINE float sbgDegToRadF(float angle)
{
	return angle * SBG_PI_F / 180.0f;
}

#endif	/* __SBG_DEFINES_H__ */
