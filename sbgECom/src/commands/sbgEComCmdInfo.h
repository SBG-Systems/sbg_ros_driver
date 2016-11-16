/*!
 *	\file		sbgEComCmdInfo.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to device info.
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
#ifndef __SBG_ECOM_CMD_INFO_H__
#define __SBG_ECOM_CMD_INFO_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Device info definitions											   -//
//----------------------------------------------------------------------//

/* Misc */
#define SBG_ECOM_INFO_PRODUCT_CODE_LENGTH			(32)

//----------------------------------------------------------------------//
//- Device Info structure											   -//
//----------------------------------------------------------------------//

/*!
 * Helper structure to retrieve device info.
 */
typedef struct _SbgEComDeviceInfo
{
	uint8	productCode[SBG_ECOM_INFO_PRODUCT_CODE_LENGTH];	/*!< Human readable Product Code. */
	uint32	serialNumber;									/*!< Device serial number */
	uint32	calibationRev;									/*!< Calibration data revision */
	uint16	calibrationYear;								/*!< Device Calibration Year */
	uint8	calibrationMonth;								/*!< Device Calibration Month */
	uint8	calibrationDay;									/*!< Device Calibration Day */
	uint32	hardwareRev;									/*!< Device hardware revision */
	uint32	firmwareRev;									/*!< Firmware revision */
} SbgEComDeviceInfo;

//----------------------------------------------------------------------//
//- Info commands				                                       -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the device information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pInfo						A pointer to a structure to hold device information.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGetInfo(SbgEComHandle *pHandle, SbgEComDeviceInfo *pInfo);

#endif
