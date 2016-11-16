#include "sbgEComBinaryLogDebug.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_DEBUG_0 message and fill the corresponding structure.
 *	\param[in]	pInputStream				Input stream buffer to read the payload from.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseDebug0Data(SbgStreamBuffer *pInputStream, SbgLogDebug0Data *pOutputData)
{
	uint32			i;

	//
	// Check input arguments
	//
	SBG_ASSERT(pInputStream);
	SBG_ASSERT(pOutputData);
	
	//
	// Read the frame payload and return
	//
	pOutputData->timeStamp = sbgStreamBufferReadUint32LE(pInputStream);

	//
	// Read each value in the array
	//
	for (i = 0; i < 64; i++)
	{
		pOutputData->data[i] = sbgStreamBufferReadUint32LE(pInputStream);
	}

	//
	// Return if any error has occurred while parsing the frame
	//
	return sbgStreamBufferGetLastError(pInputStream);
}

/*!
 * Write data for the SBG_ECOM_LOG_DEBUG_0 message to the output stream buffer from the provided structure.
 * \param[out]	pOutputStream				Output stream buffer to write the payload to.
 * \param[in]	pInputData					Pointer on the input structure that stores data to write.
 * \return									SBG_NO_ERROR if the message has been generated in the provided buffer.
 */
SbgErrorCode sbgEComBinaryLogWriteDebug0Data(SbgStreamBuffer *pOutputStream, const SbgLogDebug0Data *pInputData)
{
	uint32	i;

	//
	// Check input arguments
	//
	SBG_ASSERT(pOutputStream);
	SBG_ASSERT(pInputData);

	//
	// Write the frame payload
	//
	sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->timeStamp);

	//
	// Write each value in the array
	//
	for (i = 0; i < 64; i++)
	{
		sbgStreamBufferWriteUint32LE(pOutputStream, pInputData->data[i]);
	}

	//
	// Return if any error has occurred while writing the frame
	//
	return sbgStreamBufferGetLastError(pOutputStream);
}
