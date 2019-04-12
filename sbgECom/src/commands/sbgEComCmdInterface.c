#include "sbgEComCmdInterface.h"
#include <streamBuffer/sbgStreamBuffer.h>

//----------------------------------------------------------------------//
//- Interface commands                                                 -//
//----------------------------------------------------------------------//

/*!
 *	Retrieve the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	interfaceId					The interface from which the configuration is to be retrieved.
 *	\param[out]	pConf						Pointer to a SbgEComInterfaceConf struct to hold configuration of the interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceGetUartConf(SbgEComHandle *pHandle, SbgEComPortId interfaceId, SbgEComInterfaceConf *pConf)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;
	uint8				outputBuffer;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pConf))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command and the interfaceId as a 1-byte payload
			//
			outputBuffer = (uint8)interfaceId;
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_UART_CONF, &outputBuffer, sizeof(uint8));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_UART_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received correctly the answer
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read parameters
					// First is returned interfaceId, then baud rate and the mode at last.
					//
					interfaceId = (SbgEComPortId)sbgStreamBufferReadUint8LE(&inputStream);
					pConf->baudRate = sbgStreamBufferReadUint32LE(&inputStream);
					pConf->mode = (SbgEComPortMode)sbgStreamBufferReadUint8LE(&inputStream);

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
 *	Set the configuration of one of the interfaces.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	interfaceId					The interface from which the configuration is to be retrieved.
 *	\param[in]	pConf						Pointer to a SbgEComInterfaceConf struct that holds the new configuration for the interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceSetUartConf(SbgEComHandle *pHandle, SbgEComPortId interfaceId, const SbgEComInterfaceConf *pConf)
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
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Init stream buffer for output
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));

			//
			// Build payload
			//
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)interfaceId);
			sbgStreamBufferWriteUint32LE(&outputStream, pConf->baudRate);
			sbgStreamBufferWriteUint8LE(&outputStream, (uint8)pConf->mode);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_UART_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_UART_CONF, pHandle->cmdDefaultTimeOut);

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
 *	Retrieve the configuration the CAN interface.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[out]	pBitRate					The bitrate of the CAN interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceGetCanConf(SbgEComHandle *pHandle, SbgEComCanBitRate *pBitrate)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	size_t				receivedSize;
	uint8				receivedBuffer[SBG_ECOM_MAX_BUFFER_SIZE];
	SbgStreamBuffer		inputStream;

	//
	// Test that the input pointer are valid
	//
	if ((pHandle) && (pBitrate))
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < pHandle->numTrials; trial++)
		{
			//
			// Send the command with no payload
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_BUS_CONF, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_BUS_CONF, receivedBuffer, &receivedSize, sizeof(receivedBuffer), pHandle->cmdDefaultTimeOut);

				//
				// Test if we have received a SBG_ECOM_CMD_CAN_BUS_CONF command
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize stream buffer to read parameters
					//
					sbgStreamBufferInitForRead(&inputStream, receivedBuffer, receivedSize);

					//
					// Read bit rate returned by the device
					//
					*pBitrate = (SbgEComCanBitRate)sbgStreamBufferReadUint16LE(&inputStream);

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
 *	Set the configuration of the CAN interface.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	bitRate						The bitrate of the CAN interface.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdInterfaceSetCanConf(SbgEComHandle *pHandle, SbgEComCanBitRate bitrate)
{
	SbgErrorCode		errorCode = SBG_NO_ERROR;
	uint32				trial;
	uint8				outputBuffer[2];
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
			// Build the payload
			//
			sbgStreamBufferInitForWrite(&outputStream, outputBuffer, sizeof(outputBuffer));
			sbgStreamBufferWriteUint16LE(&outputStream, bitrate);

			//
			// Send the payload over ECom
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_BUS_CONF, sbgStreamBufferGetLinkedBuffer(&outputStream), sbgStreamBufferGetLength(&outputStream));

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_CAN_BUS_CONF, pHandle->cmdDefaultTimeOut);

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
