#include "sbgPlatform.h"
#include <stdarg.h>
#include <time.h>

//----------------------------------------------------------------------//
//- Include specific header for WIN32 and UNIX platforms               -//
//----------------------------------------------------------------------//
#ifdef WIN32
	#include <windows.h>
#else
	#include <unistd.h>
#endif

//----------------------------------------------------------------------//
//- Specific timing methods to reimplement for your platform           -//
//----------------------------------------------------------------------//

/*!
 *	Returns the current time in ms.
 *	\return				The current time in ms.
 */
uint32 sbgGetTime(void)
{
	//
	// Return the current time in ms
	//
	return clock() / (CLOCKS_PER_SEC / 1000);
}

/*!
 *	Sleep for the specified number of ms.
 *	\param[in]	ms		Number of millisecondes to wait.
 */
void sbgSleep(uint32 ms)
{
	//
	// Implementation valid for both WIN and UNIX systems
	//
	#ifdef WIN32
		Sleep(ms);
	#else
		usleep(ms*1000);
	#endif
}

//----------------------------------------------------------------------//
//- Specific logging methods to reimplement for your platform          -//
//----------------------------------------------------------------------//

/*!
 *	The method is called when one of the SBG_LOG_ERROR, SBG_LOG_WARNING, SBG_LOG_INFO or SBG_LOG_VERBOSE is called.
 *	It logs an error message with debug information and support a variable list of arguments
 *	\param[in]	pFileName					File name where the error occurred.
 *	\param[in]	pFunctionName				Function name where the error occurred.
 *	\param[in]	line						Line number where the error occurred.
 *	\param[in]	logType						Define if we have an error, a warning, an info or a verbose log.
 *	\param[in]	errorCode					The error code associated with the message.
 *	\param[in]	pFormat						The error message that will be used with the variable list of arguments.
 */
void sbgPlatformDebugLogMsg(const char *pFileName, const char *pFunctionName, uint32 line, SbgDebugLogType logType, SbgErrorCode errorCode, const char *pFormat, ...)
{
	char errorMsg[256];
	va_list args;

	//
	// Initialize the list of variable arguments on the latest function argument
	//
	va_start(args, pFormat);

	//
	// Generate the error message string
	//
	vsprintf(errorMsg, pFormat, args);

	//
	// Close the list of variable arguments
	//
	va_end(args);

	

	//
	// Log the correct message according to the log type
	//
	switch (logType)
	{
	case SBG_DEBUG_LOG_TYPE_ERROR:
		//
		// Write the complete error messages
		//
		fprintf(stderr, "*** ERROR ***\t[%s] File: %s Func: %s(%u): %s\n\r", sbgErrorCodeToString(errorCode), pFileName, pFunctionName, line, errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_WARNING:
		//
		// Write the complete warning messages
		//
		fprintf(stderr, "*** WARNING ***\t[%s] File: %s Func: %s(%u): %s\n\r", sbgErrorCodeToString(errorCode), pFileName, pFunctionName, line, errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_INFO:
		//
		// Write the complete information messages
		//
		fprintf(stderr, "*** INFO ***\tFile: %s Func: %s(%u): %s\n\r", pFileName, pFunctionName, line, errorMsg);
		break;
	case SBG_DEBUG_LOG_TYPE_VERBOSE:
		//
		// Write the complete verbose messages
		//
		fprintf(stderr, "*** VERBOSE ***\tFile: %s Func: %s(%u): %s\n\r", pFileName, pFunctionName, line, errorMsg);
		break;
	default:
		//
		// Write the complete unknown type messages
		//
		fprintf(stderr, "*** UNKNOWN ***\t[%s] File: %s Func: %s(%u): %s\n\r", sbgErrorCodeToString(errorCode), pFileName, pFunctionName, line, errorMsg);
	}
}
