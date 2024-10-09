/*!
*  \file         sbg_utm.h
*  \author       SBG Systems
*  \date         25/08/2023
*
*  \brief        Implement simple UTM projections
*
*  \section CodeCopyright Copyright Notice
*  MIT License
*
*  Copyright (c) 2023 SBG Systems
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in all
*  copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*  SOFTWARE.
*/

#ifndef SBG_UTM_H
#define SBG_UTM_H

// STL headers
#include <array>

namespace sbg
{

/*!
 * Class for Utm related data
 */
class Utm final
{
  public:

    //---------------------------------------------------------------------//
    //- Constructor                                                       -//
    //---------------------------------------------------------------------//

    /*!
     * Default constructor.
     */
    Utm() = default;

    /*!
     * Constructor.
     *
     * \param[in] latitude                  Latitude in degree [-90 to +90].
     * \param[in] longitude                 Longitude in degree [-180 to +180].
     */
    Utm(double latitude, double longitude);

    //---------------------------------------------------------------------//
    //- Parameters                                                        -//
    //---------------------------------------------------------------------//

    /*!
     * Returns if the UTM zone has been initialized.
     *
     * \return        True if the UTM zone has been initialized.
     */
    bool isInit() const;

    /*!
     * Returns UTM zone number.
     *
     * \return        Zone number.
     */
    int getZoneNumber() const;

    /*!
     * Returns UTM meridian.
     *
     * \return        Meridian in degree.
     */
    double getMeridian() const;

    /*!
     * Returns UTM letter designator.
     *
     * \return        Letter designator.
     */
    char getLetterDesignator() const;

    //---------------------------------------------------------------------//
    //- Operations                                                        -//
    //---------------------------------------------------------------------//

    /*!
     * Initialize UTM zone.
     *
     * \param[in] latitude                  Latitude in degree [-90 to +90].
     * \param[in] longitude                 Longitude in degree [-180 to +180].
     */
    void init(double latitude, double longitude);

    /*!
     * Reset the instance to uninitialized UTM zone.
     */
    void clear();

    /*!
     * Convert latitude, longitude, to easting and northing.
     *
     * \param[in] latitude                Latitude, in degrees [-90 to +90].
     * \param[in] longitude               Longitude, in degrees [-180 to +180].
     * \return                            Array containing easting then northing in meters.
     *
     */
    std::array<double, 2> computeEastingNorthing(double latitude, double longitude) const;

  private:

    /*!
     * Convert latitude and longitude to an UTM zone number.
     *
     * \param[in] latitude                Latitude, in degrees.
     * \param[in] longitude               Longitude, in degrees.
     * \return                            UTM zone number.
     */
    static int computeZoneNumber(double latitude, double longitude);

    /*!
     * Get UTM letter designator for the given latitude.
     *
     * \param[in] latitude                Latitude, in degrees.
     * \return                            UTM letter designator.
     */
    static char computeLetterDesignator(double latitude);

    /*!
     * Compute UTM zone meridian from UTM zone number.
     *
     * \return                            Meridian angle, in degrees.
     */
    double computeMeridian() const;

    bool      is_init_ = false;
    double    meridian_{};
    int       zone_number_{};
    char      letter_designator_{};
};

}

#endif // SBG_UTM_H
