#include "sbgEComBinaryLogImu.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_IMU_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseImuData(SbgStreamBuffer *pInputStream, SbgLogImuData *pOutputData)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pInputStream);
	SBG_ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status				= sbgStreamBufferReadUint16LE(pInputStream);
				
	pOutputData->accelerometers[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->accelerometers[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->accelerometers[2]	= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->gyroscopes[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->gyroscopes[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->gyroscopes[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->temperature		= sbgStreamBufferReadFloatLE(pInputStream);

	pOutputData->deltaVelocity[0]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->deltaVelocity[1]	= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->deltaVelocity[2]	= sbgStreamBufferReadFloatLE(pInputStream);
				
	pOutputData->deltaAngle[0]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->deltaAngle[1]		= sbgStreamBufferReadFloatLE(pInputStream);
	pOutputData->deltaAngle[2]		= sbgStreamBufferReadFloatLE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_IMU_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteImuData(SbgStreamBuffer *pOutputStream, const SbgLogImuData *pInputData)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pOutputStream);
	SBG_ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
				
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->accelerometers[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->gyroscopes[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->gyroscopes[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->gyroscopes[2]);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->temperature);

	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaVelocity[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaVelocity[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaVelocity[2]);
				
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaAngle[0]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaAngle[1]);
	sbgStreamBufferWriteFloatLE(pOutputStream, pInputData->deltaAngle[2]);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}

/*!
 * Parse data for the SBG_ECOM_LOG_FAST_IMU_DATA message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseFastImuData(SbgStreamBuffer *pInputStream, SbgLogFastImuData *pOutputData)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pInputStream);
	SBG_ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp			= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->status				= sbgStreamBufferReadUint16LE(pInputStream);
				
	pOutputData->accelerometers[0]	= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.01f;
	pOutputData->accelerometers[1]	= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.01f;
	pOutputData->accelerometers[2]	= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.01f;

	pOutputData->gyroscopes[0]		= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.001f;
	pOutputData->gyroscopes[1]		= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.001f;
	pOutputData->gyroscopes[2]		= (float)sbgStreamBufferReadInt16LE(pInputStream) * 0.001f;

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_FAST_IMU_DATA message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteFastImuData(SbgStreamBuffer *pOutputStream, const SbgLogFastImuData *pInputData)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pOutputStream);
	SBG_ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->status);
				
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->accelerometers[0] * 100.0f));
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->accelerometers[1] * 100.0f));
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->accelerometers[2] * 100.0f));
		
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->gyroscopes[0] * 1000.0f));
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->gyroscopes[1] * 1000.0f));
	sbgStreamBufferWriteInt16LE(pOutputStream, (int16)(pInputData->gyroscopes[2] * 1000.0f));

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
