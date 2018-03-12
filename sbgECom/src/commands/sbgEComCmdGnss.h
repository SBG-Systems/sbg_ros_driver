/*!
 *	\file		sbgEComCmdGnss.h
 *  \author		SBG Systems (Maxime Renaudet)
 *	\date		11 June 2014
 *
 *	\brief		This file implements SbgECom commands related to GNSS module.
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
#ifndef __SBG_ECOM_CMD_GNSS_H__
#define __SBG_ECOM_CMD_GNSS_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- GNSS definitions												   -//
//----------------------------------------------------------------------//

/*! 
 * This enum defines the different GNSS model IDs available in standard
 */
typedef enum _SbgEComGnssModelsStdIds
{
	SBG_ECOM_GNSS_MODEL_UBLOX_GPS_GLONASS		= 101,		/*!< Used on Ellipse-N to setup the internal GNSS in GPS+GLONASS */
	SBG_ECOM_GNSS_MODEL_NMEA					= 102,		/*!< Default mode for Ellipse-E connection to external GNSS */
	SBG_ECOM_GNSS_MODEL_UBLOX_GPS_BEIDOU		= 103,		/*!< Used on Ellipse-N to setup the internal GNSS in GPS+BEIDOU */
	SBG_ECOM_GNSS_MODEL_UBLOX_EXTERNAL			= 104,		/*!< Used on Ellipse-E to setup a connection to ublox in read only mode. */
	SBG_ECOM_GNSS_MODEL_NOVATEL_EXTERNAL		= 106,		/*!< Used on Ellipse-E to setup a connection to Novatel receiver in read only mode. */
	SBG_ECOM_GNSS_MODEL_NOVATEL_INTERNAL		= 107,		/*!< Used on Ellipse-D by default */
	SBG_ECOM_GNSS_MODEL_SEPTENTRIO_EXTERNAL		= 109		/*!< Used on Ellipse-E to setup a connection to Septentrio receiver in read only mode. */
} SbgEComGnssModelsStdIds;

//----------------------------------------------------------------------//
//- GNSS configuration												   -//
//----------------------------------------------------------------------//

/*!
 * Holds all necessary information for GNSS module alignment.
 */
typedef struct _SbgEComGnssAlignmentInfo
{
	float	leverArmX;						/*!< GNSS antenna lever arm in IMU X axis in meters */
	float	leverArmY;						/*!< GNSS antenna lever arm in IMU Y axis in meters */
	float	leverArmZ;						/*!< GNSS antenna lever arm in IMU Z axis in meters */
	float	pitchOffset;					/*!< Pitch offset for dual antenna GNSS in rad */
	float	yawOffset;						/*!< Yaw offset for dual antenna GNSS in rad */
	float	antennaDistance;				/*!< Distance between two GNSS antennas in meters */
} SbgEComGnssAlignmentInfo;

/*!
 * Holds all necessary information for GNSS module data rejection.
 */
typedef struct _SbgEComGnssRejectionConf
{
	SbgEComRejectionMode	position;		/*!< Rejection mode for position. */
	SbgEComRejectionMode	velocity;		/*!< Rejection mode for velocity. */
	SbgEComRejectionMode	hdt;			/*!< Rejection mode for true heading. */
} SbgEComGnssRejectionConf;

//----------------------------------------------------------------------//
//- GNSS public commands		                                       -//
//----------------------------------------------------------------------//

/*!
 *	Set GNSS error model id.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	id							Model ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModelId(SbgEComHandle *pHandle, uint32 id);

/*!
 *	Retrieve GNSS error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo);

/*!
 *  DEPRECATED FUNCTION. Please use sbgEComCmdGnss1SetModelId instead
 *	Set the error model for the first GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SBG_DEPRECATED(SbgErrorCode sbgEComCmdGnss1SetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size));

/*!
 *	Retrieve the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf);

/*!
 *	Set the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf);

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf);

/*!
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf);

#endif
