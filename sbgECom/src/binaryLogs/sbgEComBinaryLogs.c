#include "sbgEComBinaryLogs.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Communication protocol operations                                  -//
//----------------------------------------------------------------------//

/*!
 *	Parse an incoming log and fill the output union.
 *	\param[in]	msgClass					Received message class
 *	\param[in]	msg							Received message ID
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output union that stores parsed data.
 */
SbgErrorCode sbgEComBinaryLogParse(SbgEComClass msgClass, SbgEComMsgId msg, const void *pPayload, size_t payloadSize, SbgBinaryLogData *pOutputData)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	SbgStreamBuffer		inputStream;

	//
	// Check input parameters
	//
	SBG_ASSERT(pPayload);
	SBG_ASSERT(payloadSize > 0);
	SBG_ASSERT(pOutputData);

	//
	// Handle the different classes of messages differently
	//
	if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_0)
	{
		//
		// Create an input stream buffer that points to the frame payload so we can easily parse it's content
		//
		sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

		//
		// Parse the incoming log according to its type
		//
		switch (msg)
		{
		case SBG_ECOM_LOG_STATUS:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseStatusData(&inputStream, &pOutputData->statusData);
			break;
		case SBG_ECOM_LOG_IMU_DATA:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseImuData(&inputStream, &pOutputData->imuData);
			break;
		case SBG_ECOM_LOG_EKF_EULER:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfEulerData(&inputStream, &pOutputData->ekfEulerData);
			break;
		case SBG_ECOM_LOG_EKF_QUAT:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfQuatData(&inputStream, &pOutputData->ekfQuatData);
			break;
		case SBG_ECOM_LOG_EKF_NAV:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseEkfNavData(&inputStream, &pOutputData->ekfNavData);
			break;
		case SBG_ECOM_LOG_SHIP_MOTION:
		case SBG_ECOM_LOG_SHIP_MOTION_HP:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseShipMotionData(&inputStream, &pOutputData->shipMotionData);
			break;
		case SBG_ECOM_LOG_ODO_VEL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseOdometerData(&inputStream, &pOutputData->odometerData);
			break;
		case SBG_ECOM_LOG_UTC_TIME:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseUtcData(&inputStream, &pOutputData->utcData);
			break;
		case SBG_ECOM_LOG_GPS1_VEL:
		case SBG_ECOM_LOG_GPS2_VEL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsVelData(&inputStream, &pOutputData->gpsVelData);
			break;
		case SBG_ECOM_LOG_GPS1_POS:
		case SBG_ECOM_LOG_GPS2_POS:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsPosData(&inputStream, &pOutputData->gpsPosData);
			break;
		case SBG_ECOM_LOG_GPS1_HDT:
		case SBG_ECOM_LOG_GPS2_HDT:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsHdtData(&inputStream, &pOutputData->gpsHdtData);
			break;
		case SBG_ECOM_LOG_GPS1_RAW:
		case SBG_ECOM_LOG_GPS2_RAW:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseGpsRawData(&inputStream, &pOutputData->gpsRawData);
			break;
		case SBG_ECOM_LOG_MAG:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseMagData(&inputStream, &pOutputData->magData);
			break;
		case SBG_ECOM_LOG_MAG_CALIB:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseMagCalibData(&inputStream, &pOutputData->magCalibData);
			break;
		case SBG_ECOM_LOG_DVL_BOTTOM_TRACK:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDvlData(&inputStream, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_DVL_WATER_TRACK:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDvlData(&inputStream, &pOutputData->dvlData);
			break;
		case SBG_ECOM_LOG_PRESSURE:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParsePressureData(&inputStream, &pOutputData->pressureData);
			break;
		case SBG_ECOM_LOG_USBL:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseUsblData(&inputStream, &pOutputData->usblData);
			break;
		case SBG_ECOM_LOG_EVENT_A:
		case SBG_ECOM_LOG_EVENT_B:
		case SBG_ECOM_LOG_EVENT_C:
		case SBG_ECOM_LOG_EVENT_D:
		case SBG_ECOM_LOG_EVENT_E:
			//
			// Parse all events markers logs
			//
			errorCode = sbgEComBinaryLogParseEvent(&inputStream, &pOutputData->eventMarker);
			break;
		case SBG_ECOM_LOG_DEBUG_0:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseDebug0Data(&inputStream, &pOutputData->debug0Data);
			break;
		default:
			//
			// This log isn't handled
			//
			errorCode = SBG_ERROR;
		}
	}
	else if (msgClass == SBG_ECOM_CLASS_LOG_ECOM_1)
	{
		//
		// Create an input stream buffer that points to the frame payload so we can easily parse it's content
		//
		sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

		//
		// Parse the message depending on the message ID
		//
		switch ((SbgEComLog1)msg)
		{
		case SBG_ECOM_LOG_FAST_IMU_DATA:
			//
			// Parse this binary log
			//
			errorCode = sbgEComBinaryLogParseFastImuData(&inputStream, &pOutputData->fastImuData);
			break;
		default:
			//
			// This log isn't handled
			//
			errorCode = SBG_ERROR;
		}
	}
	else
	{
		//
		// Unhandled message class
		//
		errorCode = SBG_ERROR;
	}
	
	return errorCode;
}
