// File header
#include "sbg_ros_helpers.h"

// SBG headers
#include <sbgDefines.h>

// STL headers
#include <cmath>

float sbg::helpers::wrapAnglePi(float angle_rad)
{
  if (angle_rad > SBG_PI_F)
  {
    return (SBG_PI_F * 2.0f - fmodf(angle_rad, SBG_PI_F * 2.0f));
  }

  if (angle_rad < -SBG_PI_F)
  {
    return (SBG_PI_F * 2.0f + fmodf(angle_rad, SBG_PI_F * 2.0f));
  }

  return angle_rad;
}

float sbg::helpers::wrapAngle360(float angle_deg)
{
  float wrapped_angle_deg = angle_deg;

  if ( (wrapped_angle_deg < -360.0f) || (wrapped_angle_deg > 360.0f) )
  {
    wrapped_angle_deg = fmodf(wrapped_angle_deg, 360.0f);
  }

  if (wrapped_angle_deg < 0.0f)
  {
    wrapped_angle_deg = 360.0f + wrapped_angle_deg;
  }

  return wrapped_angle_deg;
}

uint32_t sbg::helpers::getNumberOfDaysInYear(uint16_t year)
{
  if (isLeapYear(year))
  {
    return 366;
  }
  else
  {
    return 365;
  }
}

uint32_t sbg::helpers::getNumberOfDaysInMonth(uint16_t year, uint8_t month_index)
{
  if ((month_index == 4) || (month_index == 6) || (month_index == 9) || (month_index == 11))
  {
    return 30;
  }
  else if ((month_index == 2))
  {
    if (isLeapYear(year))
    {
        return 29;
    }
    else
    {
        return 28;
    }
  }
  else
  {
    return 31;
  }
}

bool sbg::helpers::isLeapYear(uint16_t year)
{
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

int32_t sbg::helpers::getUtcOffset(bool first_valid_utc, uint32_t gps_tow, uint8_t sec)
{
  constexpr int32_t   DEFAULT_UTC_OFFSET  = 18;   /*!< Driver default GPS to UTC offset in seconds. */
  int32_t             utcOffset;

  if (first_valid_utc)
  {
    // Compute the leap second: GPS = UTC + utcOffset
    utcOffset = (gps_tow/1000)%60 - sec;

    if (utcOffset < 0)
    {
      utcOffset = 60 + utcOffset;
    }
  }
  else
  {
    utcOffset = DEFAULT_UTC_OFFSET;
  }

  return utcOffset;
}

sbg::helpers::NmeaGGAQuality sbg::helpers::convertSbgGpsTypeToNmeaGpsType(SbgEComGpsPosType sbg_gps_type)
{
  sbg::helpers::NmeaGGAQuality  nmeaQuality = NmeaGGAQuality::INVALID;

  switch (sbg_gps_type)
  {
    case SBG_ECOM_POS_NO_SOLUTION:
      nmeaQuality = NmeaGGAQuality::INVALID;
      break;

    case SBG_ECOM_POS_UNKNOWN_TYPE:
    case SBG_ECOM_POS_SINGLE:
    case SBG_ECOM_POS_FIXED:
      nmeaQuality = NmeaGGAQuality::SINGLE;
      break;

    case SBG_ECOM_POS_PSRDIFF:
    case SBG_ECOM_POS_SBAS:
    case SBG_ECOM_POS_OMNISTAR:
      nmeaQuality = NmeaGGAQuality::DGPS;
      break;

    case SBG_ECOM_POS_PPP_FLOAT:
    case SBG_ECOM_POS_PPP_INT:
      nmeaQuality = NmeaGGAQuality::PPS;
      break;

    case SBG_ECOM_POS_RTK_INT:
      nmeaQuality = NmeaGGAQuality::RTK_FIXED;
      break;

    case SBG_ECOM_POS_RTK_FLOAT:
      nmeaQuality = NmeaGGAQuality::RTK_FLOAT;
      break;
  }

  return nmeaQuality;
}

sbg::SbgVector3d sbg::helpers::convertLLAtoECEF(double latitude, double longitude, double altitude)
{
  //
  // Conversion from Geodetic coordinates to ECEF is based on World Geodetic System 1984 (WGS84).
  // Radius are expressed in meters, and latitude/longitude in radian.
  //
  static constexpr double EQUATORIAL_RADIUS = 6378137.0;
  static constexpr double POLAR_RADIUS = 6356752.314245;

  double latitude_rad = sbgDegToRadD(latitude);
  double longitude_rad = sbgDegToRadD(longitude);
  double compute_cte = pow(POLAR_RADIUS, 2) / pow(EQUATORIAL_RADIUS, 2);
  double eccentricity = 1.0 - compute_cte;
  double prime_vertical_radius = EQUATORIAL_RADIUS / sqrt(1.0 - (pow(eccentricity, 2) * pow(sin(latitude_rad), 2)));

  sbg::SbgVector3d ecef_vector((prime_vertical_radius + altitude) * cos(latitude_rad) * cos(longitude_rad),
                               (prime_vertical_radius + altitude) * cos(latitude_rad) * sin(longitude_rad),
                               fma(compute_cte, prime_vertical_radius, altitude) * sin(latitude_rad));
  return ecef_vector;
}