#include "sbgEComCmdOdo.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Odometer commands	                                               -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the odometer module configuration.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pOdometerConf				Pointer to a SbgEComOdoConf struct to hold configuration of the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetConf(SbgEComHandle *pHandle, SbgEComOdoConf *pOdometerConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pOdometerConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command only since this is a no-payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a SBG_ECOM_CMD_ODO_CONF command
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
					pOdometerConf->gain = sbgStreamBufferReadFloatLE(&inputStream);
					pOdometerConf->gainError = sbgStreamBufferReadUint8LE(&inputStream);
					pOdometerConf->reverseMode = sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the odometer module configuration.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pOdometerConf				Pointer to a SbgEComOdoConf struct holding configuration for the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetConf(SbgEComHandle *pHandle, const SbgEComOdoConf *pOdometerConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pOdometerConf))
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
			sbgStreamBufferWriteFloatLE(&outputStream, pOdometerConf->gain);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pOdometerConf->gainError);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pOdometerConf->reverseMode);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_CONF, pHandle->cmdDefaultTimeOut);

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
 *	Retrieve the odometer lever arms.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	leverArm					Array of three values, one for each axis.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetLeverArm(SbgEComHandle *pHandle, float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointer are valid
	//
	if (pHandle)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command only since this is a no-payload command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a correct answer
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
					leverArm[0] = sbgStreamBufferReadFloatLE(&inputStream);
					leverArm[1] = sbgStreamBufferReadFloatLE(&inputStream);
					leverArm[2] = sbgStreamBufferReadFloatLE(&inputStream);

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
 *	Set the odometer lever arms.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	leverArm					Array of three values, one for each axis.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetLeverArm(SbgEComHandle *pHandle, const float leverArm[3])
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		outputStream;

	//
	// Test that the input pointer are valid
	//
	if (pHandle)
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
			sbgStreamBufferWriteFloatLE(&outputStream, leverArm[0]);
			sbgStreamBufferWriteFloatLE(&outputStream, leverArm[1]);
			sbgStreamBufferWriteFloatLE(&outputStream, leverArm[2]);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_LEVER_ARM, pHandle->cmdDefaultTimeOut);

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
 *	Retrieve the rejection configuration of the odometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pRejectConf					Pointer to a SbgEComOdoRejectionConf struct to hold rejection configuration of the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoGetRejection(SbgEComHandle *pHandle, SbgEComOdoRejectionConf *pRejectConf)
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
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a correct answer
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
					pRejectConf->velocity = (SbgEComRejectionMode)sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the rejection configuration of the odometer module.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pRejectConf					Pointer to a SbgEComOdoRejectionConf struct holding rejection configuration for the odometer module.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdOdoSetRejection(SbgEComHandle *pHandle, const SbgEComOdoRejectionConf *pRejectConf)
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
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pRejectConf->velocity);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_ODO_REJECT_MODE, pHandle->cmdDefaultTimeOut);

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
