/*!
 *	\file		sbgDebug.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Define and handle error logging for the SBG Systems common C library.
 *
 *	The methods defined here should be implemented in sbgPlatform.h/sbgPlatform.c
 *	according to your platform and needs.
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
#ifndef __SBG_DEBUG_H__
#define __SBG_DEBUG_H__

#include "platform/sbgConfig.h"

//----------------------------------------------------------------------//
//- Errors and warning definitions                                     -//
//----------------------------------------------------------------------//

/*!
 *	Enum that identify the type of error / warning that has been thrown.
 */
typedef enum _SbgDebugLogType
{
	SBG_DEBUG_LOG_TYPE_ERROR,					/*!< The message to log is an error. */
	SBG_DEBUG_LOG_TYPE_WARNING,					/*!< The message to log is a warning. */
	SBG_DEBUG_LOG_TYPE_INFO,					/*!< The message to log is an information. */
	SBG_DEBUG_LOG_TYPE_VERBOSE					/*!< The message to log is a verbose information. */
} SbgDebugLogType;

//----------------------------------------------------------------------//
//- Errors and warning macros                                          -//
//----------------------------------------------------------------------//

/*!
 *	Run time assert that is raised if the expression is false.
 *	\param[in]	expression						The boolean expression to test, the execution is interrupted if the expression is evaluated as false.
 */
#if SBG_CONFIG_ENABLE_ASSERT == SBG_ENABLE
	#define SBG_ASSERT(expression)				SBG_PLATFORM_ASSERT(expression)
#else
	#define SBG_ASSERT(expression)				((void)0)
#endif

/*!
 *	Log an error with its associated message.
 *	\param[in]	errorCode						The error code that has thrown this error.
 *	\param[in]	format							String litteral for the associated error message (you can use printf like string formating).
 */
#if SBG_CONFIG_ENABLE_LOG_ERROR == SBG_ENABLE
	#define SBG_LOG_ERROR(errorCode, format, ...)		sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_TYPE_ERROR, errorCode, format, ##__VA_ARGS__)
#else
	#define SBG_LOG_ERROR(errorCode, format, ...)		((void)0)
#endif

/*!
 *	Log a warning with its associated message.
 *	\param[in]	errorCode						The error code that has thrown this warning.
 *	\param[in]	format							String litteral for the associated warning message (you can use printf like string formating).
 */
#if SBG_CONFIG_ENABLE_LOG_WARNING == SBG_ENABLE
	#define SBG_LOG_WARNING(errorCode, format, ...)		sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_TYPE_WARNING, errorCode, format, ##__VA_ARGS__)
#else
	#define SBG_LOG_WARNING(errorCode, format, ...)		((void)0)
#endif

/*!
 *	Log an information message.
 *	\param[in]	format							String litteral for the information message (you can use printf like string formating).
 */
#if SBG_CONFIG_ENABLE_LOG_INFO == SBG_ENABLE
	#define SBG_LOG_INFO(format, ...)					sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_TYPE_INFO, SBG_NO_ERROR, format, ##__VA_ARGS__)
#else
	#define SBG_LOG_INFO(format, ...)					((void)0)
#endif
/*!
 *	Log an information message only in verbose mode
 *	\param[in]	format							String litteral for the information message (you can use printf like string formating).
 */
#if SBG_CONFIG_ENABLE_LOG_VERBOSE == SBG_ENABLE
	#define SBG_LOG_VERBOSE(format, ...)				sbgPlatformDebugLogMsg((const char*)__BASE_FILE__, (const char*)__FUNCTION__, __LINE__, SBG_DEBUG_LOG_TYPE_VERBOSE, SBG_NO_ERROR, format, ##__VA_ARGS__)
#else
	#define SBG_LOG_VERBOSE(format, ...)				((void)0)
#endif

#endif	/* __SBG_DEBUG_H__ */
