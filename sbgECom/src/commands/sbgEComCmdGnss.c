#include "sbgEComCmdGnss.h"
#include <streamBuffer/sbgStreamBuffer.h>
#include "transfer/sbgEComTransfer.h"

//----------------------------------------------------------------------//
//- GNSS private commands                                              -//
//----------------------------------------------------------------------//
/*!
 *	Set GNSS error model id.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	id							Model ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetModelId(SbgEComHandle *pHandle, uint32 id, SbgEComCmd cmdId)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericSetModelId(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, id);
}

/*!
 *	Retrieve GNSS error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo, SbgEComCmd cmdId)
{
	//
	// Call generic function with specific command name
	//
	return sbgEComCmdGenericGetModelInfo(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, pModelInfo);
}

/*!
 *  DEPRECATED FUNCTION. Please use sbgEComCmdGnssSetModelId instead
 *	Set the error model for the first GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size, SbgEComCmd cmdId)
{
	//
	// Call function that handle data transfer
	//
	return sbgEComTransferSend(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, pBuffer, size);
}

/*!
 *	Retrieve the alignment configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pAlignConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command only since this is a no-payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					//
					pAlignConf->leverArmX = sbgStreamBufferReadFloatLE(&inputStream);
					pAlignConf->leverArmY = sbgStreamBufferReadFloatLE(&inputStream);
					pAlignConf->leverArmZ = sbgStreamBufferReadFloatLE(&inputStream);
					pAlignConf->pitchOffset = sbgStreamBufferReadFloatLE(&inputStream);
					pAlignConf->yawOffset = sbgStreamBufferReadFloatLE(&inputStream);
					pAlignConf->antennaDistance = sbgStreamBufferReadFloatLE(&inputStream);

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
 *	Set the alignment configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pAlignConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->leverArmX);
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->leverArmY);
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->leverArmZ);
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->pitchOffset);
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->yawOffset);
			sbgStreamBufferWriteFloatLE(&outputStream, pAlignConf->antennaDistance);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, pHandle->cmdDefaultTimeOut);

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
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssGetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pRejectConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command only since this is a no-payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a SBG_ECOM_CMD_GNSS_1_REJECT_MODES command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					//
					pRejectConf->position	= (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);
					pRejectConf->velocity	= (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);
					sbgStreamBufferReadUint8LE(&inputStream);													// Skipped for backward compatibility
					pRejectConf->hdt		= (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\param[in]	cmdId						Command to send. Can be used to configure either GNSS 1 or 2
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnssSetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf, SbgEComCmd cmdId)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pRejectConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pRejectConf->position);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pRejectConf->velocity);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)SBG_ECOM_NEVER_ACCEPT_MODE);		// Reserved parameter
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pRejectConf->hdt);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, cmdId, pHandle->cmdDefaultTimeOut);

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
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}


//----------------------------------------------------------------------//
//- GNSS public commands		                                       -//
//----------------------------------------------------------------------//

/*!
 *	Set GNSS 1 error model id.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	id							Model ID to set
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModelId(SbgEComHandle *pHandle, uint32 id)
{
	return sbgEComCmdGnssSetModelId(pHandle, id, SBG_ECOM_CMD_GNSS_1_MODEL_ID);
}

/*!
 *	Retrieve GNSS 1 error model information.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pMotionProfileInfo			Pointer to a SbgEComModelInfo to contain the current GNSS error model info.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetModelInfo(SbgEComHandle *pHandle, SbgEComModelInfo *pModelInfo)
{
	return sbgEComCmdGnssGetModelInfo(pHandle, pModelInfo, SBG_ECOM_CMD_GNSS_1_MODEL_ID);
}

/*!
 *  DEPRECATED FUNCTION. Please use sbgEComCmdGnss1SetModelId instead
 *	Set the error model for a GNSS module
 *	The new configuration will only be applied after SBG_ECOM_CMD_SETTINGS_ACTION (01) command is issued, with SBG_ECOM_SAVE_SETTINGS parameter.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the error model buffer.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetModel(SbgEComHandle *pHandle, const void *pBuffer, uint32 size)
{
	return sbgEComCmdGnssSetModel(pHandle, pBuffer, size, SBG_ECOM_CMD_GNSS_1_SET_MODEL);
}

/*!
 *	Retrieve the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct to hold alignment configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetLeverArmAlignment(SbgEComHandle *pHandle, SbgEComGnssAlignmentInfo *pAlignConf)
{
	return sbgEComCmdGnssGetLeverArmAlignment(pHandle, pAlignConf, SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT);
}

/*!
 *	Set the lever arm and alignment configuration of the gnss 1 module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pAlignConf					Pointer to a SbgEComGnssAlignmentInfo struct holding alignment configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetLeverArmAlignment(SbgEComHandle *pHandle, const SbgEComGnssAlignmentInfo *pAlignConf)
{
	return sbgEComCmdGnssSetLeverArmAlignment(pHandle, pAlignConf, SBG_ECOM_CMD_GNSS_1_LEVER_ARM_ALIGNMENT);
}

/*!
 *	Retrieve the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct to hold rejection configuration of the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1GetRejection(SbgEComHandle *pHandle, SbgEComGnssRejectionConf *pRejectConf)
{
	return sbgEComCmdGnssGetRejection(pHandle, pRejectConf, SBG_ECOM_CMD_GNSS_1_REJECT_MODES);
}

/*!
 *	Set the rejection configuration of the gnss module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pAlignConf					Pointer to a SbgEComGnssRejectionConf struct holding rejection configuration for the gnss module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdGnss1SetRejection(SbgEComHandle *pHandle, const SbgEComGnssRejectionConf *pRejectConf)
{
	return sbgEComCmdGnssSetRejection(pHandle, pRejectConf, SBG_ECOM_CMD_GNSS_1_REJECT_MODES);
}
