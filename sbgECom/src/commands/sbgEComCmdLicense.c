#include "sbgEComCmdLicense.h"
#include "transfer/sbgEComTransfer.h"

//----------------------------------------------------------------------//
//- License commands                                                   -//
//----------------------------------------------------------------------//

/*!
 *	Upload and apply a new license to a device.
 *	The device will reboot automatically to use the new license.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the license.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComCmdLicenseApply(SbgEComHandle *pHandle, const void *pBuffer, size_t size)
{
	//
	// Call function that handle data transfer
	//
	return sbgEComTransferSend(pHandle, SBG_ECOM_CLASS_LOG_CMD_0, SBG_ECOM_CMD_LICENSE_APPLY, pBuffer, size);
}
