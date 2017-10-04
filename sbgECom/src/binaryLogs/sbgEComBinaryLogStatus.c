#include "sbgEComBinaryLogStatus.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 * Parse data for the SBG_ECOM_LOG_STATUS message and fill the corresponding structure.
 * \param[in]	pInputStream				Input stream buffer to read the payload from.
 * \param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 * \return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseStatusData(SbgStreamBuffer *pInputStream, SbgLogStatusData *pOutputData)
{
	//
	// Check input arguments
	//
	SBG_ASSERT(pInputStream);
	SBG_ASSERT(pOutputData);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->generalStatus	= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->reserved1		= sbgStreamBufferReadUint16LE(pInputStream);
	pOutputData->comStatus		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->aidingStatus	= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->reserved2		= sbgStreamBufferReadUint32LE(pInputStream);
	pOutputData->reserved3		= sbgStreamBufferReadUint16LE(pInputStream);

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_STATUS message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteStatusData(SbgStreamBuffer *pOutputStream, const SbgLogStatusData *pInputData)
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
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->generalStatus);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->reserved1);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->comStatus);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->aidingStatus);
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->reserved2);
	sbgStreamBufferWriteUint16LE(pOutputStream, pInputData->reserved3);

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
