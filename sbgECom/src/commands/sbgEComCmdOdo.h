/*!
 *	\file		sbgEComCmdOdo.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to Odometer module.
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
#ifndef __SBG_ECOM_CMD_ODO_H__
#define __SBG_ECOM_CMD_ODO_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Odometer definitions											   -//
//----------------------------------------------------------------------//

//----------------------------------------------------------------------//
//- Odometer configuration											   -//
//----------------------------------------------------------------------//

/*!
 * Holds all necessary information for Odometer module parameter configuration.
 */
typedef struct _SbgEComOdoConf
{
	float		gain;						/*!< Odometer's gain in pulses / meter. */
	uint8		gainError;					/*!< User gain average error in % */
	bool		reverseMode;				/*!< Whether the odometer is in reverse mode or not. */
} SbgEComOdoConf;

/*!
 * Holds all necessary information for Odometer module data rejection.
 */
typedef struct _SbgEComOdoRejectionConf
{
	SbgEComRejectionMode	velocity;		/*!< Rejection mode for velocity. */
} SbgEComOdoRejectionConf;

//----------------------------------------------------------------------//
//- Odometer commands												   -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the odometer module configuration.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pOdometerConf				Pointer to a SbgEComOdoConf struct to hold configuration of the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetConf(SbgEComHandle *pHandle, SbgEComOdoConf *pOdometerConf);

/*!
 *	Set the odometer module configuration.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pOdometerConf				Pointer to a SbgEComOdoConf struct holding configuration for the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetConf(SbgEComHandle *pHandle, const SbgEComOdoConf *pOdometerConf);

/*!
 *	Retrieve the odometer lever arms.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	leverArm					Array of three values, one for each axis.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetLeverArm(SbgEComHandle *pHandle, float leverArm[3]);

/*!
 *	Set the odometer lever arms.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	leverArm					Array of three values, one for each axis.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetLeverArm(SbgEComHandle *pHandle, const float leverArm[3]);

/*!
 *	Retrieve the rejection configuration of the odometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pRejectConf					Pointer to a SbgEComOdoRejectionConf struct to hold rejection configuration of the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetRejection(SbgEComHandle *pHandle, SbgEComOdoRejectionConf *pRejectConf);

/*!
 *	Set the rejection configuration of the odometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pRejectConf					Pointer to a SbgEComOdoRejectionConf struct holding rejection configuration for the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetRejection(SbgEComHandle *pHandle, const SbgEComOdoRejectionConf *pRejectConf);

#endif
