// File header
#include "sbg_utm.h"

// STL headers
#include <cmath>

using sbg::Utm;

//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

Utm::Utm(double latitude, double longitude)
{
  init(latitude, longitude);
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

bool Utm::isInit() const
{
  return is_init_;
}

int Utm::getZoneNumber() const
{
  return zone_number_;
}

double Utm::getMeridian() const
{
  return meridian_;
}

char Utm::getLetterDesignator() const
{
  return letter_designator_;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void Utm::init(double latitude, double longitude)
{
  zone_number_ = computeZoneNumber(latitude, longitude);
  letter_designator_ = computeLetterDesignator(latitude);
  meridian_ = computeMeridian();
  is_init_ = true;
}

void Utm::clear()
{
  is_init_ = false;
  meridian_ = {};
  zone_number_ = {};
  letter_designator_ = {};
}

/*
 * Originally written by Chuck Gantz - chuck.gantz@globalstar.com
 */
std::array<double, 2> Utm::computeEastingNorthing(double latitude, double longitude) const
{
  constexpr double RADIANS_PER_DEGREE = M_PI / 180.0;

  // WGS84 Parameters
  constexpr double WGS84_A = 6378137.0;        // major axis
  constexpr double WGS84_E = 0.0818191908;     // first eccentricity

  // UTM Parameters
  constexpr double UTM_K0 = 0.9996;              // scale factor
  constexpr double UTM_E2 = (WGS84_E * WGS84_E); // e^2

  double long_origin;
  double ecc_prime_squared;
  double n, t, c, a, m;

  // Make sure the longitude is between -180.00 .. 179.9
  double long_temp = (longitude + 180) - int((longitude + 180) / 360) * 360 - 180;

  double lat_rad = latitude * RADIANS_PER_DEGREE;
  double long_rad = long_temp * RADIANS_PER_DEGREE;
  double long_origin_rad;

  // +3 puts origin in middle of zone
  long_origin = (zone_number_ - 1) * 6 - 180 + 3;
  long_origin_rad = long_origin * RADIANS_PER_DEGREE;

  ecc_prime_squared = (UTM_E2)/(1 - UTM_E2);

  n = WGS84_A/sqrt(1 - UTM_E2 * sin(lat_rad) * sin(lat_rad));
  t = tan(lat_rad) * tan(lat_rad);
  c = ecc_prime_squared * cos(lat_rad) * cos(lat_rad);
  a = cos(lat_rad) * (long_rad-long_origin_rad);

  m = WGS84_A * ((1 - UTM_E2/4 - 3*UTM_E2*UTM_E2/64    - 5*UTM_E2*UTM_E2*UTM_E2/256)   * lat_rad
              -  (3 * UTM_E2/8 + 3*UTM_E2*UTM_E2/32    + 45*UTM_E2*UTM_E2*UTM_E2/1024) * sin(2*lat_rad)
                               + (15*UTM_E2*UTM_E2/256 + 45*UTM_E2*UTM_E2*UTM_E2/1024) * sin(4*lat_rad)
                               - (35*UTM_E2*UTM_E2*UTM_E2/3072)                        * sin(6*lat_rad));

  double utm_easting = (double)(UTM_K0*n*(a+(1-t+c)*a*a*a/6
                       + (5-18*t+t*t+72*c-58*ecc_prime_squared)*a*a*a*a*a/120)
                       + 500000.0);

  double utm_northing = (double)(UTM_K0*(m+n*tan(lat_rad)*(a*a/2+(5-t+9*c+4*c*c)*a*a*a*a/24
                        + (61-58*t+t*t+600*c-330*ecc_prime_squared)*a*a*a*a*a*a/720)));


  if (latitude < 0)
  {
    utm_northing += 10000000.0; //10000000 meter offset for southern hemisphere
  }

  std::array<double, 2> easting_northing{utm_easting, utm_northing};
  return (easting_northing);
}

int Utm::computeZoneNumber(double latitude, double longitude)
{
  int zone_number;

  // Make sure the longitude is between -180.00 .. 179.9
  double long_temp = (longitude + 180) - int((longitude + 180) / 360) * 360 - 180;

  zone_number = int((long_temp + 180) / 6) + 1;

  if ( latitude >= 56.0 && latitude < 64.0 && long_temp >= 3.0 && long_temp < 12.0 )
  {
    zone_number = 32;
  }

  // Special zones for Svalbard
  if ( latitude >= 72.0 && latitude < 84.0 )
  {
    if ( long_temp >= 0.0  && long_temp < 9.0 )
    {
      zone_number = 31;
    }
    else if ( long_temp >= 9.0  && long_temp < 21.0 )
    {
      zone_number = 33;
    }
    else if ( long_temp >= 21.0 && long_temp < 33.0 )
    {
      zone_number = 35;
    }
    else if ( long_temp >= 33.0 && long_temp < 42.0 )
    {
      zone_number = 37;
    }
  }

  return zone_number;
}

/*
 * Originally written by Chuck Gantz - chuck.gantz@globalstar.com
 */
char Utm::computeLetterDesignator(double latitude)
{
  char letter_designator;

  if ((84 >= latitude) && (latitude >= 72))
  {
    letter_designator = 'X';
  }
  else if ((72 > latitude) && (latitude >= 64))
  {
    letter_designator = 'W';
  }
  else if ((64 > latitude) && (latitude >= 56))
  {
    letter_designator = 'V';
  }
  else if ((56 > latitude) && (latitude >= 48))
  {
    letter_designator = 'U';
  }
  else if ((48 > latitude) && (latitude >= 40))
  {
    letter_designator = 'T';
  }
  else if ((40 > latitude) && (latitude >= 32))
  {
    letter_designator = 'S';
  }
  else if ((32 > latitude) && (latitude >= 24))
  {
    letter_designator = 'R';
  }
  else if ((24 > latitude) && (latitude >= 16))
  {
    letter_designator = 'Q';
  }
  else if ((16 > latitude) && (latitude >= 8))
  {
    letter_designator = 'P';
  }
  else if ((8 > latitude) && (latitude >= 0))
  {
    letter_designator = 'N';
  }
  else if ((0 > latitude) && (latitude >= -8))
  {
    letter_designator = 'M';
  }
  else if ((-8 > latitude) && (latitude >= -16))
  {
    letter_designator = 'L';
  }
  else if ((-16 > latitude) && (latitude >= -24))
  {
    letter_designator = 'K';
  }
  else if ((-24 > latitude) && (latitude >= -32))
  {
    letter_designator = 'J';
  }
  else if ((-32 > latitude) && (latitude >= -40))
  {
    letter_designator = 'H';
  }
  else if ((-40 > latitude) && (latitude >= -48))
  {
    letter_designator = 'G';
  }
  else if ((-48 > latitude) && (latitude >= -56))
  {
    letter_designator = 'F';
  }
  else if ((-56 > latitude) && (latitude >= -64))
  {
    letter_designator = 'E';
  }
  else if ((-64 > latitude) && (latitude >= -72))
  {
    letter_designator = 'D';
  }
  else if ((-72 > latitude) && (latitude >= -80))
  {
    letter_designator = 'C';
  }
  else
  {
    // 'Z' is an error flag, the Latitude is outside the UTM limits
    letter_designator = 'Z';
  }
  return letter_designator;
}

double Utm::computeMeridian() const
{
  return (zone_number_ == 0) ? 0.0 : (zone_number_ - 1) * 6.0 - 177.0;
}
