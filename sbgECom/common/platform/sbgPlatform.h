/*!
 *	\file		sbgPlatform.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Header file that contains all platform specific definitions.
 *
 *	This file should be modified to each targeted platform.
 *	For example, all common headers should be included from this file.
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
#ifndef __SBG_PLATFORM_H__
#define __SBG_PLATFORM_H__

#include "../sbgTypes.h"
#include "../sbgErrorCodes.h"
#include "../sbgDebug.h"

//----------------------------------------------------------------------//
//- Add here any additional includes you want to share                 -//
//----------------------------------------------------------------------//

// We would like to use the standard C library
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

// We need the standard assert header as we use it to implement the SBG_ASSERT macro
#include <assert.h>

//----------------------------------------------------------------------//
//- Specific timing methods to reimplement for your platform           -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
uint32 sbgGetTime(void);

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
void sbgSleep(uint32 ms);

//----------------------------------------------------------------------//
//- Specific logging methods to reimplement for your platform          -//
//----------------------------------------------------------------------//

/*!
 *	The method is called when one of the SBG_LOG_ERROR, SBG_LOG_WARNING, SBG_LOG_INFO or SBG_LOG_VERBOSE is called.
 *	It logs an error message with debug information and support a variable list of arguments
 *	\param[in]	pFileName					File name where the error occurred.
 *	\param[in]	pFunctionName				Function name where the error occurred.
 *	\param[in]	line						Line number where the error occurred.
 *	\param[in]	logType						Define if we have an error, a warning, an info or a verbose log.
 *	\param[in]	errorCode					The error code associated with the message.
 *	\param[in]	pFormat						The error message that will be used with the variable list of arguments.
 */
void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32 line, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...);

//----------------------------------------------------------------------//
//- Errors and warning managment                                       -//
//----------------------------------------------------------------------//

/*!
 *	Run time assert that is raised if the expression is false.
 *	\param[in]	expression						The boolean expression to test, the execution is interrupted if the expression is evaluated as false.
 */
#define SBG_PLATFORM_ASSERT(expression)		assert((expression))

#endif	/* __SBG_PLATFORM_H__ */
