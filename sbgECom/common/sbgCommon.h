/*!
 *	\file		sbgCommon.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		17 March 2015
 *
 *	\brief		Main header file for SBG Systems common C library.
 *
 *	All files / projets that would like to use the SBG Systems common C library
 *	should include this file.
 *
 *	Be aware that this file doesn't include SBG Systems common C library components
 *	but just defines all types and environements such as error logging.
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
#ifndef __SBG_COMMON_H__
#define __SBG_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- Include platform specific headers                                  -//
//----------------------------------------------------------------------//
#include "platform/sbgPlatform.h"
#include "platform/sbgConfig.h"

//----------------------------------------------------------------------//
//- Include headers needed to setup the environement                   -//
//----------------------------------------------------------------------//
#include "sbgTypes.h"
#include "sbgErrorCodes.h"
#include "sbgDefines.h"
#include "sbgDebug.h"
#include "version/sbgVersion.h"

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//
#ifdef __cplusplus
}
#endif

#endif	/* __SBG_COMMON_H__ */
