/*!
*	\file         message_wrapper.h
*	\author       SBG Systems
*	\date         13/03/2020
*	 
*	\brief        Handle creation of messages.
*
*   Methods to create ROS messages from given data. 
*	 
*	\section CodeCopyright Copyright Notice
*	MIT License
*	 
*	Copyright (c) 2020 SBG Systems
*	 
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*	 
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*	 
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/

#ifndef SBG_ROS_MESSAGE_WRAPPER_H
#define SBG_ROS_MESSAGE_WRAPPER_H

// SbgECom headers
#include <sbgEComLib.h>
#include <sbgEComIds.h>

// Sbg header
#include <sbg_matrix3.h>

// ROS headers
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/NavSatFix.h>

// SbgRos message headers
#include "sbg_driver/SbgStatus.h"
#include "sbg_driver/SbgUtcTime.h"
#include "sbg_driver/SbgImuData.h"
#include "sbg_driver/SbgEkfEuler.h"
#include "sbg_driver/SbgEkfQuat.h"
#include "sbg_driver/SbgEkfNav.h"
#include "sbg_driver/SbgShipMotion.h"
#include "sbg_driver/SbgMag.h"
#include "sbg_driver/SbgMagCalib.h"
#include "sbg_driver/SbgGpsVel.h"
#include "sbg_driver/SbgGpsPos.h"
#include "sbg_driver/SbgGpsHdt.h"
#include "sbg_driver/SbgGpsRaw.h"
#include "sbg_driver/SbgOdoVel.h"
#include "sbg_driver/SbgEvent.h"
#include "sbg_driver/SbgImuShort.h"
#include "sbg_driver/SbgAirData.h"

namespace sbg
{
/*!
 * Class to wrap the SBG logs into ROS messages.
 */
class MessageWrapper
{
private:

  ros::Time               m_ros_processing_time_;
  sbg_driver::SbgUtcTime  m_last_sbg_utc_;
  bool                    m_first_valid_utc_;

  //---------------------------------------------------------------------//
  //- Internal methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Create a geometry message Vector3 from a raw input vector.
   * 
   * \template  T                   Numeric template type.
   * \param[in] p_array             Raw input vector.
   * \param[in] array_size          Raw vector size, should be defined as 3.
   * \return                        GeometryMsg Vector3.
   */
  template <typename T>
  const geometry_msgs::Vector3 createGeometryVector3(const T* p_array, size_t array_size) const
  {
    assert(array_size == 3);

    geometry_msgs::Vector3 geometry_vector;

    geometry_vector.x = p_array[0];
    geometry_vector.y = p_array[1];
    geometry_vector.z = p_array[2];

    return geometry_vector;
  };

  /*!
   * Create a ROS message header.
   * 
   * \param[in] device_timestamp    SBG device timestamp (in microseconds).
   * \param[in] ref_frame_id    	Frame ID.
   * \return                        ROS header message.
   */
  const std_msgs::Header createRosHeader(uint32_t device_timestamp, const std::string& ref_frame_id) const;

  /*!
   * Compute corrected ROS time for the device timestamp.
   * 
   * \param[in] device_timestamp    SBG device timestamp (in microseconds).
   * \return                        ROS time.
   */
  const ros::Time computeCorrectedRosTime(uint32_t device_timestamp) const;

  /*!
   * Create SBG-ROS Ekf status message.
   * 
   * \param[in] ekf_status          SBG Ekf status.
   * \return                        Ekf status message.
   */
  const sbg_driver::SbgEkfStatus createEkfStatusMessage(uint32_t ekf_status) const;

  /*!
   * Create SBG-ROS GPS Position status message.
   * 
   * \param[in] ref_log_gps_pos     SBG GPS position log.
   * \return                        GPS Position status.
   */
  const sbg_driver::SbgGpsPosStatus createGpsPosStatusMessage(const SbgLogGpsPos& ref_log_gps_pos) const;

  /*!
   * Create SBG-ROS GPS Velocity status message.
   * 
   * \param[in] ref_log_gps_vel     SBG GPS Velocity log.
   * \return                        GPS Velocity status.
   */
  const sbg_driver::SbgGpsVelStatus createGpsVelStatusMessage(const SbgLogGpsVel& ref_log_gps_vel) const;

  /*!
   * Create a SBG-ROS IMU status message.
   * 
   * \param[in] sbg_imu_status      SBG IMU status.
   * \return                        IMU status message.
   */
  const sbg_driver::SbgImuStatus createImuStatusMessage(uint16_t sbg_imu_status) const;

  /*!
   * Create a SBG-ROS Magnetometer status message.
   * 
   * \param[in] ref_log_mag         SBG Magnetometer log.
   * \return                        Magnetometer status message.
   */
  const sbg_driver::SbgMagStatus createMagStatusMessage(const SbgLogMag& ref_log_mag) const;

  /*!
   * Create a SBG-ROS Ship motion status message.
   * 
   * \param[in] ref_log_ship_motion SBG Ship motion log.
   * \return                        ship motion status message.
   */
  const sbg_driver::SbgShipMotionStatus createShipMotionStatusMessage(const SbgLogShipMotionData& ref_log_ship_motion) const;

  /*!
   * Create a SBG-ROS aiding status message.
   *
   * \param[in] ref_log_status      SBG status log.
   * \return                        Aiding status message.
   */
  const sbg_driver::SbgStatusAiding createStatusAidingMessage(const SbgLogStatusData& ref_log_status) const;

  /*!
   * Create a SBG-ROS com status message.
   *
   * \param[in] ref_log_status      SBG status log.
   * \return                        Com status message.
   */
  const sbg_driver::SbgStatusCom createStatusComMessage(const SbgLogStatusData& ref_log_status) const;

  /*!
   * Create a SBG-ROS general status message.
   *
   * \param[in] ref_log_status      SBG status log.
   * \return                        General status message.
   */
  const sbg_driver::SbgStatusGeneral createStatusGeneralMessage(const SbgLogStatusData& ref_log_status) const;

  /*!
   * Create a SBG-ROS UTC time status message.
   * 
   * \param[in] ref_log_utc         SBG UTC data log.
   * \return                        UTC time status message.
   */
  const sbg_driver::SbgUtcTimeStatus createUtcStatusMessage(const SbgLogUtcData& ref_log_utc) const;

  /*!
   * Get the number of days in the year.
   *
   * \param[in] year                Year to get the number of days.
   * \return                        Number of days in the year.
   */
  uint32_t getNumberOfDaysInYear(uint16_t year) const;

  /*!
   * Get the number of days of the month index.
   * 
   * \param[in] year                Year.
   * \param[in] month_index         Month index [1..12].
   * \return                        Number of days in the month.
   */
  uint32_t getNumberOfDaysInMonth(uint16_t year, uint8_t month_index) const;

  /*!
   * Check if the given year is a leap year.
   * 
   * \param[in] year                Year to check.
   * \return                        True if the year is a leap year.
   */
  bool isLeapYear(uint16_t year) const;

  /*!
   * Convert the UTC time to an Epoch time.
   * 
   * \param[in] ref_sbg_utc_msg     SBG-ROS UTC message.
   * \return                        Converted Epoch time (in s).
   */
  const ros::Time convertUtcTimeToEpoch(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const;

  /*!
   * Create a SBG-ROS air data status message.
   * 
   * \param[in] ref_sbg_air_data    SBG AirData log.
   * \return                        SBG-ROS air data status message.
   */
  const sbg_driver::SbgAirDataStatus createAirDataStatusMessage(const SbgLogAirData& ref_sbg_air_data) const;
  
  /*!
   * Create a ROS standard TwistStamped message.
   * 
   * \param[in] body_vel            SBG Body velocity vector.
   * \param[in] ref_sbg_air_data    SBG IMU message.
   * \param[in] ref_frame_id    	Frame ID.
   * \return                        SBG TwistStamped message.
   */
  const geometry_msgs::TwistStamped createRosTwistStampedMessage(const sbg::SbgVector3f& body_vel, const sbg_driver::SbgImuData& ref_sbg_imu_msg, const std::string& ref_frame_id) const;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  MessageWrapper(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Set the wrapper processing ROS time.
   * This method is call on the SbgDevice periodic handle, in order to have the same processing time for the messages.
   * 
   * \param[in] ref_ros_time        ROS processing time to set.
   */
  void setRosProcessingTime(const ros::Time& ref_ros_time);

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Create a SBG-ROS Ekf Euler message.
   * 
   * \param[in] ref_log_ekf_euler   SBG Ekf Euler log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Ekf Euler message.
   */
  const sbg_driver::SbgEkfEuler createSbgEkfEulerMessage(const SbgLogEkfEulerData& ref_log_ekf_euler, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Ekf Navigation message.
   * 
   * \param[in] ref_log_ekf_nav     SBG Ekf Navigation log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Ekf Navigation message.
   */
  const sbg_driver::SbgEkfNav createSbgEkfNavMessage(const SbgLogEkfNavData& ref_log_ekf_nav, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Ekf Quaternion message.
   * 
   * \param[in] ref_log_ekf_quat    SBG Ekf Quaternion log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Ekf Quaternion message.
   */
  const sbg_driver::SbgEkfQuat createSbgEkfQuatMessage(const SbgLogEkfQuatData& ref_log_ekf_quat, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS event message.
   * 
   * \param[in] ref_log_event       SBG event log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Event message.
   */
  const sbg_driver::SbgEvent createSbgEventMessage(const SbgLogEvent& ref_log_event, const std::string& ref_frame_id) const;

  /*!
   * Create SBG-ROS GPS-HDT message.
   * 
   * \param[in] ref_log_gps_hdt     SBG GPS HDT log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        GPS HDT message.
   */
  const sbg_driver::SbgGpsHdt createSbgGpsHdtMessage(const SbgLogGpsHdt& ref_log_gps_hdt, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS GPS-Position message.
   * 
   * \param[in] ref_log_gps_pos     SBG GPS Position log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        GPS Position message.
   */
  const sbg_driver::SbgGpsPos createSbgGpsPosMessage(const SbgLogGpsPos& ref_log_gps_pos, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS GPS raw message.
   * 
   * \param[in] ref_log_gps_raw     SBG GPS raw log.
   * \return                        GPS raw message.
   */
  const sbg_driver::SbgGpsRaw createSbgGpsRawMessage(const SbgLogGpsRaw& ref_log_gps_raw) const;

  /*!
   * Create a SBG-ROS GPS Velocity message.
   * 
   * \param[in] ref_log_gps_vel     SBG GPS Velocity log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        GPS Velocity message.
   */
  const sbg_driver::SbgGpsVel createSbgGpsVelMessage(const SbgLogGpsVel& ref_log_gps_vel, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Imu data message.
   * 
   * \param[in] ref_log_imu_data    SBG Imu data log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Imu data message.
   */
  const sbg_driver::SbgImuData createSbgImuDataMessage(const SbgLogImuData& ref_log_imu_data, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Magnetometer message.
   * 
   * \param[in] ref_log_mag         SBG Magnetometer log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Magnetometer message.
   */
  const sbg_driver::SbgMag createSbgMagMessage(const SbgLogMag& ref_log_mag, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Magnetometer calibration message.
   * 
   * \param[in] ref_log_mag_calib   SBG Magnetometer calibration log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Magnetometer calibration message.
   */
  const sbg_driver::SbgMagCalib createSbgMagCalibMessage(const SbgLogMagCalib& ref_log_mag_calib, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Odometer velocity message.
   * 
   * \param[in] ref_log_odo         SBG Odometer log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Odometer message.
   */
  const sbg_driver::SbgOdoVel createSbgOdoVelMessage(const SbgLogOdometerData& ref_log_odo, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Shipmotion message.
   * 
   * \param[in] ref_log_ship_motion SBG Ship motion log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Ship motion message.
   */
  const sbg_driver::SbgShipMotion createSbgShipMotionMessage(const SbgLogShipMotionData& ref_log_ship_motion, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS status message from a SBG status log.	
   *                                                       	
   * \param[in] ref_log_status      SBG status log.        	
   * \param[in] ref_frame_id        Frame ID.
   * \return                        Status message.        	
   */                                                      	
  const sbg_driver::SbgStatus createSbgStatusMessage(const 	SbgLogStatusData& ref_log_status, const std::string& ref_frame_id) const;
                                                           	
  /*!                                                      	
   * Create a SBG-ROS UTC time message from a SBG UTC log. 	
   *
   * \param[in] ref_log_utc         SBG UTC log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        UTC time message.                  
   */
  const sbg_driver::SbgUtcTime createSbgUtcTimeMessage(const SbgLogUtcData& ref_log_utc, const std::string& ref_frame_id);

  /*!
   * Create a SBG-ROS Air data message from a SBG log.
   * 
   * \param[in] ref_air_data_log    SBG AirData log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        SBG-ROS airData message.
   */
  const sbg_driver::SbgAirData createSbgAirDataMessage(const SbgLogAirData& ref_air_data_log, const std::string& ref_frame_id) const;

  /*!
   * Create a SBG-ROS Short Imu message.
   * 
   * \param[in] ref_short_imu_log   SBG Imu short log.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        SBG-ROS Imu short message.
   */
  const sbg_driver::SbgImuShort createSbgImuShortMessage(const SbgLogImuShort& ref_short_imu_log, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard IMU message from SBG messages.
   * 
   * \param[in] ref_sbg_imu_msg     SBG-ROS IMU message.
   * \param[in] ref_sbg_quat_msg    SBG_ROS Quaternion message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard IMU message.
   */
   const sensor_msgs::Imu createRosImuMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgEkfQuat& ref_sbg_quat_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard Temperature message from SBG message.
   * 
   * \param[in] ref_sbg_imu_msg     SBG-ROS IMU message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard Temperature message.
   */
  const sensor_msgs::Temperature createRosTemperatureMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard MagneticField message from SBG message.
   * 
   * \param[in] ref_sbg_mag_msg     SBG-ROS Mag message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard Mag message.
   */
  const sensor_msgs::MagneticField createRosMagneticMessage(const sbg_driver::SbgMag& ref_sbg_mag_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard TwistStamped message from SBG messages.
   * 
   * \param[in] ref_sbg_ekf_euler_msg   SBG-ROS Ekf Euler message.
   * \param[in] ref_sbg_ekf_nav_msg     SBG-ROS Ekf Nav message.
   * \param[in] ref_sbg_imu_msg         SBG-ROS IMU message.
   * \param[in] ref_frame_id            Frame ID.
   * \return                            ROS standard TwistStamped message.
   */
  const geometry_msgs::TwistStamped createRosTwistStampedMessage(const sbg_driver::SbgEkfEuler& ref_sbg_ekf_vel_msg, const sbg_driver::SbgEkfNav& ref_sbg_ekf_nav_msg, const sbg_driver::SbgImuData& ref_sbg_imu_msg, const std::string& ref_frame_id) const;
  
  /*!
   * Create a ROS standard TwistStamped message from SBG messages.
   * 
   * \param[in] ref_sbg_ekf_quat_msg    SBG-ROS Ekf Quaternion message.
   * \param[in] ref_sbg_ekf_nav_msg     SBG-ROS Ekf Nav message.
   * \param[in] ref_sbg_imu_msg         SBG-ROS IMU message.
   * \param[in] ref_frame_id            Frame ID.
   * \return                            ROS standard TwistStamped message.
   */
  const geometry_msgs::TwistStamped createRosTwistStampedMessage(const sbg_driver::SbgEkfQuat& ref_sbg_ekf_vel_msg, const sbg_driver::SbgEkfNav& ref_sbg_ekf_nav_msg, const sbg_driver::SbgImuData& ref_sbg_imu_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard PointStamped message from SBG messages.
   * 
   * \param[in] ref_sbg_ekf_msg     SBG-ROS EkfNav message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard PointStamped message (ECEF).
   */
  const geometry_msgs::PointStamped createRosPointStampedMessage(const sbg_driver::SbgEkfNav& ref_sbg_ekf_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard timeReference message for a UTC time.
   * 
   * \param[in] ref_sbg_utc_msg     SBG-ROS UTC message.
   * \return                        ROS standard timeReference message.
   */
  const sensor_msgs::TimeReference createRosUtcTimeReferenceMessage(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const;

  /*!
   * Create a ROS standard NavSatFix message from a Gps message.
   * 
   * \param[in] ref_sbg_gps_msg     SBG-ROS GPS position message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard NavSatFix message.
   */
  const sensor_msgs::NavSatFix createRosNavSatFixMessage(const sbg_driver::SbgGpsPos& ref_sbg_gps_msg, const std::string& ref_frame_id) const;

  /*!
   * Create a ROS standard FluidPressure message.
   * 
   * \param[in] ref_sbg_air_msg     SBG-ROS AirData message.
   * \param[in] ref_frame_id        Frame ID.
   * \return                        ROS standard fluid pressure message.
   */
  const sensor_msgs::FluidPressure createRosFluidPressureMessage(const sbg_driver::SbgAirData& ref_sbg_air_msg, const std::string& ref_frame_id) const;
};
}

#endif // SBG_ROS_MESSAGE_WRAPPER_H
