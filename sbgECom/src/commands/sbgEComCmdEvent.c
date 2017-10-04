#include "sbgEComCmdEvent.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Event commands		                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of a Sync In.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncInId					The id of the sync whose configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComSyncInConf to contain the current configuration of the sync in.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncInGetConf(SbgEComHandle *pHandle, SbgEComSyncInId syncInId, SbgEComSyncInConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint32				receivedSize;
	uint8				outputBuffer;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointers are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Send the command with syncInId as a 1-byte payload
			//
			outputBuffer = (uint8)syncInId;
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF, &outputBuffer, sizeof(uint8));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a SBG_ECOM_CMD_SYNC_IN_CONF command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned the id of the sync, then the sensitivity and the delay at last.
					//
					syncInId = (SbgEComSyncInId)sbgStreamBufferReadUint8LE(&inputStream);					
					pConf->sensitivity = (SbgEComSyncInSensitivity)sbgStreamBufferReadUint8LE(&inputStream);
					pConf->delay = sbgStreamBufferReadInt32LE(&inputStream);

					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Null pointer
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Set the configuration of a Sync In.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncInId					The id of the sync whose configuration is to be set.
 *	\param[in]	pConf						Pointer to a SbgEComSyncInConf that contains the new configuration for the sync in.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncInSetConf(SbgEComHandle *pHandle, SbgEComSyncInId syncInId, const SbgEComSyncInConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)syncInId);			
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pConf->sensitivity);
			sbgStreamBufferWriteInt32LE(&outputStream, pConf->delay);

			//
			// Send the message over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), (uint32)sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_IN_CONF, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Null pointer.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the configuration of a Sync Out.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncOutId					The id of the sync whose configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComSyncOutConf to contain the current configuration of the sync out.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncOutGetConf(SbgEComHandle *pHandle, SbgEComSyncOutId syncOutId, SbgEComSyncOutConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint32				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	uint8				reserved;
	SbgStreamBuffer		inputStream;
	uint8				outputBuffer;

	//
	// Test that the input pointers are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Send the command with syncOutId as a 1-byte payload
			//
			outputBuffer = (uint8)syncOutId;
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF, &outputBuffer, sizeof(uint8));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a SBG_ECOM_CMD_SYNC_OUT_CONF command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned the id of the sync, then a reserved field, the output function, polarity and the duration at last.
					//
					syncOutId = (SbgEComSyncOutId)sbgStreamBufferReadUint8LE(&inputStream);
					reserved = sbgStreamBufferReadUint8LE(&inputStream);
					pConf->outputFunction = (SbgEComSyncOutFunction)sbgStreamBufferReadUint16LE(&inputStream);
					pConf->polarity = (SbgEComSyncOutPolarity)sbgStreamBufferReadUint8LE(&inputStream);
					pConf->duration = sbgStreamBufferReadUint32LE(&inputStream);

					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Null pointer
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Set the configuration of a Sync Out.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	syncOutId					The id of the sync whose configuration is to be set.
 *	\param[in]	pConf						Pointer to a SbgEComSyncOutConf that contains the new configuration for the sync Out.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdSyncOutSetConf(SbgEComHandle *pHandle, SbgEComSyncOutId syncOutId, const SbgEComSyncOutConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)syncOutId);
			sbgStreamBufferWriteUint8LE(&outputStream, 0);
			sbgStreamBufferWriteUint16LE(&outputStream, (uint16)pConf->outputFunction);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pConf->polarity);
			sbgStreamBufferWriteUint32LE(&outputStream, pConf->duration);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), (uint32)sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_SYNC_OUT_CONF, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Null pointer
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}
