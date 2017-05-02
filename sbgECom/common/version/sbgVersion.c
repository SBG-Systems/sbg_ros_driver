#include "sbgVersion.h"
#include <sbgCommon.h>

//----------------------------------------------------------------------//
//- Version encoding / decoding methods                                -//
//----------------------------------------------------------------------//

/*!
 *	Fill a SbgVersion structure based on an uint32 that stores the 'compiled' version information.
 *	\param[in]	encodedVersion											The version information stored within a uint32.
 *	\param[out]	pVersionInfo											Pointer on an allocated SbgVersion structure to fill.
 */
void sbgVersionDecode(uint32 encodedVersion, SbgVersion *pVersionInfo)
{
	//
	// Check input parameters
	//
	SBG_ASSERT(pVersionInfo);

	//
	// Test if we have a software version scheme
	//
	if (encodedVersion&SBG_VERSION_SOFT_SCHEME)
	{
		//
		// We have a software scheme, decode it
		//
		pVersionInfo->softwareScheme = true;

		//
		// Decode the software scheme fields
		//
		pVersionInfo->qualifier	= (SbgVersionQualifier)((encodedVersion >> SBG_VERSION_SOFT_SCHEME_QUALIFIER_SHIFT) & SBG_VERSION_SOFT_SCHEME_QUALIFIER_MASK);
		pVersionInfo->major		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_MAJOR_SHIFT) & SBG_VERSION_SOFT_SCHEME_MAJOR_MASK;
		pVersionInfo->minor		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_MINOR_SHIFT) & SBG_VERSION_SOFT_SCHEME_MINOR_MASK;
		pVersionInfo->build		= (encodedVersion >> SBG_VERSION_SOFT_SCHEME_BUILD_SHIFT) & SBG_VERSION_SOFT_SCHEME_BUILD_MASK;

		//
		// Set the revision to zero as it's not used
		//
		pVersionInfo->rev		= 0;
	}
	else
	{
		//
		// We have a basic scheme, decode it
		//
		pVersionInfo->softwareScheme = false;

		//
		// Decode the software scheme fields
		//
		pVersionInfo->major		= (encodedVersion >> 24) & 0xFF;
		pVersionInfo->minor		= (encodedVersion >> 16) & 0xFF;
		pVersionInfo->rev		= (encodedVersion >>  8) & 0xFF;
		pVersionInfo->build		= (encodedVersion >>  0) & 0xFF;

		//
		// Set the qualifier to zero
		//
		pVersionInfo->qualifier	= SBG_VERSION_QUALIFIER_DEV;
	}
}

/*!
 *	Construct a uint32 containing a version information based on a SbgVersion structure.
 *	\param[in]	pVersionInfo											Pointer on a read only version structure to encode.
 *	\return																The encoded version information on an uint32 or 0 if an error has occurred.
 */
uint32 sbgVersionEncode(const SbgVersion *pVersionInfo)
{
	uint32	encodedVersion = 0;

	//
	// Check input parameter
	//
	SBG_ASSERT(pVersionInfo);

	//
	// Test if we have a software scheme or a basic one
	//
	if (pVersionInfo->softwareScheme)
	{
		//
		// We have a software, scheme, so test that the version is valid
		//
		SBG_ASSERT((pVersionInfo->major <= 63) && (pVersionInfo->minor <= 63) && (pVersionInfo->rev == 0));

		//
		// Indicate that we have a software version scheme
		//
		encodedVersion = SBG_VERSION_SOFTWARE(pVersionInfo->major, pVersionInfo->minor, pVersionInfo->build, pVersionInfo->qualifier);
	}
	else
	{
		//
		// We have a basic version scheme so check parameter validty
		//
		SBG_ASSERT(pVersionInfo->major <= 127);

		//
		// Encode the basic version information
		//
		encodedVersion = SBG_VERSION_BASIC(pVersionInfo->major, pVersionInfo->minor, pVersionInfo->rev, pVersionInfo->build);
	}

	return encodedVersion;
}

//----------------------------------------------------------------------//
//- Version comparaison methods                                        -//
//----------------------------------------------------------------------//

/*!
 *	Compare two version information structures and return if the version A is greater, less or equal than the version B.
 *	The computation is roughly result = version A - version B
 *	We can define how far we will check if the version A is greater than the version B.
 *	For example, we can only check the major or major and minor fields.
 *	\param[in]	pVersionA					The first version to compare.
 *	\param[in]	pVersionB					The second version to compare.
 *	\param[in]	thresold					The comparaison thresold to know if we are checking the major, minor, revision, build, ...
 *	\return									A positive value if the version A is greater than B, a negative one if version A is less than B and 0 if the two versions are the same (according to the thresold).
 */
int32 sbgVersionCompare(const SbgVersion *pVersionA, const SbgVersion *pVersionB, SbgVersionCmpThresold thresold)
{
	int32	result;

	//
	// Check input parameter
	//
	SBG_ASSERT((pVersionA) && (pVersionB));

	//
	// Check that the two versions are using the same scheme
	//
	SBG_ASSERT(pVersionA->softwareScheme == pVersionB->softwareScheme);

	//
	// Do the comparaison according to the selected thresold
	// Start by compairing the major field
	//
	result = pVersionA->major - pVersionB->major;

	//
	// Test if we have to also compare the minor field
	//
	if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_MINOR) || (thresold == SBG_VERSION_CMP_THRESOLD_REVISION) || (thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
	{
		//
		// Update the result using the minor indication
		//
		result = pVersionA->minor - pVersionB->minor;

		//
		// Test if we have to also compare the revision field (for basic version only)
		//
		if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_REVISION) || (thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
		{
			//
			// Test if we have a software scheme or a basic scheme version
			//
			if (pVersionA->softwareScheme)
			{
				//
				// We have a software scheme so set the result to 0
				//
				result = 0;
			}
			else
			{
				//
				// We have a basic scheme so we can compare the revision field
				//
				result = pVersionA->rev - pVersionB->rev;
			}

			//
			// Test if we have to also compare the build field
			//
			if ( (result == 0) &&  ((thresold == SBG_VERSION_CMP_THRESOLD_BUILD) || (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER)) )
			{
				//
				// Compare the build field
				//
				result = pVersionA->build - pVersionB->build;

				//
				// Test if we have to also compare the qualifier field
				//
				if ( (result == 0) && (thresold == SBG_VERSION_CMP_THRESOLD_QUALIFIER) )
				{
					//
					// Test if we have a software scheme
					//
					if (pVersionA->softwareScheme)
					{
						//
						// We have a software scheme so set the result to 0
						//
						result = pVersionA->qualifier - pVersionB->qualifier;
					}
					else
					{
						//
						// We have a basic scheme so set the result to 0
						//
						result = 0;
					}
				}
			}
		}
	}

	return result;
}

//----------------------------------------------------------------------//
//- Version to string methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Convert a version information to a human readable string.
 *	\param[in]	pVersionInfo											Pointer on a read only version structure to convert to a human readable string.
 *	\param[out]	pBuffer													Buffer to store the version as a human readable null terminated string.
 *	\param[in]	sizeOfBuffer											Maximum buffer size including the null terminated char.
 *	\return																SBG_NO_ERROR if the version information has been converted to a string.
 *																		SBG_BUFFER_OVERFLOW is the buffer isn't big enough to store the converted version string.
 */
SbgErrorCode sbgVersionToString(const SbgVersion *pVersionInfo, char *pBuffer, uint32 sizeOfBuffer)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;

	//
	// Check input parameters
	//
	SBG_ASSERT((pVersionInfo) && (pBuffer) && (sizeOfBuffer>0));

	//
	// Test if we have a basic or software version scheme
	//
	if (pVersionInfo->softwareScheme)
	{
		//
		// We have a software version scheme
		// Test that the buffer is big enough to store the generated string (31.31.65535-hotfix)
		//
		if (sizeOfBuffer >= 19)
		{
			//
			// Generate the string version
			//
			sprintf(pBuffer, "%u.%u.%u-", pVersionInfo->major, pVersionInfo->minor, pVersionInfo->build);

			//
			// Append the qualifier
			//
			switch (pVersionInfo->qualifier)
			{
			case SBG_VERSION_QUALIFIER_DEV:
				strcat(pBuffer, "dev");
				break;
			case SBG_VERSION_QUALIFIER_ALPHA:
				strcat(pBuffer, "alpha");
				break;
			case SBG_VERSION_QUALIFIER_BETA:
				strcat(pBuffer, "beta");
				break;
			case SBG_VERSION_QUALIFIER_RC:
				strcat(pBuffer, "rc");
				break;
			case SBG_VERSION_QUALIFIER_STABLE:
				strcat(pBuffer, "stable");
				break;
			case SBG_VERSION_QUALIFIER_HOT_FIX:
				strcat(pBuffer, "hotfix");
				break;
			default:
				break;
			}
		}
		else
		{
			//
			// Output buffer is to small
			//
			errorCode = SBG_BUFFER_OVERFLOW;
		}

	}
	else
	{
		//
		// We have a basic version scheme, generate the string
		// Test that the buffer is big enough to store the generated string
		//
		if (sizeOfBuffer >= 16)
		{
			//
			// Generate the string version
			//
			sprintf(pBuffer, "%u.%u.%u.%u", pVersionInfo->major, pVersionInfo->minor, pVersionInfo->rev, pVersionInfo->build);
		}
		else
		{
			//
			// Output buffer is to small
			//
			errorCode = SBG_BUFFER_OVERFLOW;
		}
	}

	//
	// Return status of operation
	//
	return errorCode;
}

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	version													Encoded version value to to convert to a human readable string.
 *	\param[out]	pBuffer													Buffer to store the version as a human readable null terminated string.
 *	\param[in]	sizeOfBuffer											Maximum buffer size including the null terminated char.
 *	\return																SBG_NO_ERROR if the version information has been converted to a string.
 *																		SBG_BUFFER_OVERFLOW is the buffer isn't big enough to store the converted version string.
 */
SbgErrorCode sbgVersionToStringEncoded(uint32 version, char *pBuffer, uint32 sizeOfBuffer)
{
	SbgVersion	versionInfo;

	//
	// Decode the versions
	//
	sbgVersionDecode(version, &versionInfo);

	//
	// Return the version as a string
	//
	return sbgVersionToString(&versionInfo, pBuffer, sizeOfBuffer);
}

//----------------------------------------------------------------------//
//- String to version methods                                          -//
//----------------------------------------------------------------------//

/*!
 *	Convert a human readable string to a version structure.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersionInfo											Pointer to a version structure to store the parsed version info.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SbgErrorCode sbgVersionFromString(const char *pVersionStr, SbgVersion *pVersionInfo)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	char			qualifierStr[32];
	uint32			major;
	uint32			minor;
	uint32			rev;
	uint32			build;

	//
	// Check input parameters
	//
	SBG_ASSERT((pVersionStr) && (pVersionInfo));

	//
	// Zero init the returned version struct
	//
	memset(pVersionInfo, 0x00, sizeof(SbgVersion));

	//
	// Try to parse a basic version
	//
	if (sscanf(pVersionStr, "%u.%u.%u.%u", &major, &minor, &rev, &build) == 4)
	{
		//
		// We have read successfully a basic version
		//
		pVersionInfo->softwareScheme = false;

		//
		// Fill the version numbers
		//
		pVersionInfo->major	= (uint8)major;
		pVersionInfo->minor	= (uint8)minor;
		pVersionInfo->rev	= (uint8)rev;
		pVersionInfo->build	= (uint8)build;
	}
	else if (sscanf(pVersionStr, "%u.%u.%u-%s", &major, &minor, &build, qualifierStr) == 4)
	{
		//
		// We have read successfully a software scheme version
		//
		pVersionInfo->softwareScheme = true;

		//
		// Fill the version numbers
		//
		pVersionInfo->major	= (uint8)major;
		pVersionInfo->minor	= (uint8)minor;
		pVersionInfo->build	= (uint16)build;

		//
		// Also convert the qualifier
		//
		if (!strcmp(qualifierStr, "dev"))
		{
			//
			// Dev qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_DEV;
		}
		else if (!strcmp(qualifierStr, "alpha"))
		{
			//
			// Alpha qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_ALPHA;
		}
		else if (!strcmp(qualifierStr, "beta"))
		{
			//
			// Beta qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_BETA;
		}
		else if (!strcmp(qualifierStr, "rc"))
		{
			//
			// Release Candidate qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_RC;
		}
		else if (!strcmp(qualifierStr, "stable"))
		{
			//
			// Stable qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_STABLE;
		}
		else if (!strcmp(qualifierStr, "hotfix"))
		{
			//
			// Hot Fix qualifier
			//
			pVersionInfo->qualifier = SBG_VERSION_QUALIFIER_HOT_FIX;
		}
		else
		{
			//
			// Unknown qualifier
			//
			errorCode = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// Invalid format
		//
		errorCode = SBG_INVALID_PARAMETER;
	}
	
	return errorCode;
}

/*!
 *	Convert an encoded version number to a human readable string.
 *	\param[in]	pVersionStr												The string containing the version to convert.
 *	\param[out]	pVersion												Returned encoded version value parsed from the string.
 *	\return																SBG_NO_ERROR if the version information has been converted from a string.
 */
SbgErrorCode sbgVersionFromStringEncoded(const char *pVersionStr, uint32 *pVersion)
{
	SbgErrorCode	errorCode = SBG_NO_ERROR;
	SbgVersion		versionInfo;

	//
	// Check input parameters
	//
	SBG_ASSERT((pVersionStr) && (pVersion));

	//
	// Parse the version from a string
	//
	errorCode = sbgVersionFromString(pVersionStr, &versionInfo);

	//
	// Test that no error has occurred
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Encode the version and return it
		//
		*pVersion = sbgVersionEncode(&versionInfo);
	}
	else
	{
		//
		// An error has occurred so return a zero version
		//
		*pVersion = 0;
	}

	//
	// Return error
	//
	return errorCode;
}
