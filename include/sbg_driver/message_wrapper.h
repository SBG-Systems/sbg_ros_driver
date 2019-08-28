#ifndef SBG_ROS_MESSAGE_WRAPPER_H
#define SBG_ROS_MESSAGE_WRAPPER_H

// SbgECom headers
extern "C"
{
  #include <sbgEComLib.h>
  #include <sbgEComIds.h>
}

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
#include "sbg_driver/SbgPressure.h"

namespace sbg
{
/*!
 * Class to wrap the SBG logs into ROS messages.
 */
class MessageWrapper
{
private:

  //---------------------------------------------------------------------//
  //- GPS time definitions                                              -//
  //---------------------------------------------------------------------//

  static const int32_t SBG_GPS_TIME_OF_WEEK_MS_MAX  = (60l*60l*24l*7l*1000l-1l);
  static const int32_t SBG_GPS_TIME_OF_WEEK_MS_HALF = (60l*60l*24l*7l*1000l/2l-1l);

  ros::Time m_ros_processing_time_;
  int32_t   m_leap_seconds_;

  //---------------------------------------------------------------------//
  //- Internal methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Create a ROS time.
   * 
   * \param[in] device_timestamp    SBG device timestamp.
   * \return                        ROS time.
   */
  const ros::Time createRosTime(uint32 device_timestamp) const;

  /*!
   * Create a ROS Vector3 from a float array.
   * 
   * \param[in] p_float_array       Array of float data.
   * \return                        ROS Vector3.
   */
  const geometry_msgs::Vector3 createRosVector3(const float* p_float_array) const;

  /*!
   * Create a ROS Vector3 from a double array.
   * 
   * \param[in] p_double_array      Array of double data.
   * \return                        ROS Vector3.
   */
  const geometry_msgs::Vector3 createRosVector3(const double* p_double_array) const;

  /*!
   * Create SBG-ROS Ekf status message.
   * 
   * \param[in] ekf_status          SBG Ekf status.
   * \return                        Ekf status message.
   */
  const sbg_driver::SbgEkfStatus createEkfStatusMessage(uint32 ekf_status) const;

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
   * \param[in] ref_log_imu         SBG IMU data log.
   * \return                        IMU status message.
   */
  const sbg_driver::SbgImuStatus createImuStatusMessage(const SbgLogImuData& ref_log_imu) const;

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
  uint32_t convertUtcTimeToEpoch(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const;

  /*!
   * Get the UTC time of week.
   *
   * \param[in] ref_sbg_utc_msg     SBG-ROS UTC message.
   * \return                        UTC time of week.
   */
  uint32_t getUtcTimeOfWeek(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const;

  /*!
   * Compute the correct time of week from Utc and Gps time.
   * 
   * \param[in] ref_sbg_utc_msg     SBG-ROS UTC message.
   * \return                        Computed time of week (in ms).
   */
  uint32_t computeTimeOfWeek(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const;

  /*!
   * Compute the GPS time of week delta.
   * 
   * \param[in] gps_time_of_week_A  First GPS time of week (in ms).
   * \param[in] gps_time_of_week_B  First GPS time of week (in ms).
   * \return                        Delta GPS time of week (in ms).
   */
  int32 computeGpsTimeOfWeekDelta(uint32_t gps_time_of_week_A, uint32_t gps_time_of_week_B) const;

  /*!
   * Add an offset to a GPS time of week.
   * 
   * \param[in] gps_time_of_week    GPS time of week.
   * \param[in] offset              Offset to apply (in ms).
   * \return                        Gps time of week (in ms).
   */
  uint32_t addOffsetGpsTimeOfWeek(uint32_t gps_time_of_week, int32 offset) const;   
 
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
   * 
   * \param[in] ref_ros_time        ROS processing time to set.
   */
  void setRosProcessingTime(const ros::Time& ref_ros_time);

  /*!
   * Set the leap seconds.
   * 
   * \param[in] leap_seconds        Know leap seconds for Utc/Gps time.
   */
  void setLeapSeconds(int32 leap_seconds);

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Create a SBG-ROS Ekf Euler message.
   * 
   * \param[in] ref_log_ekf_euler   SBG Ekf Euler log.
   * \return                        Ekf Euler message.
   */
  const sbg_driver::SbgEkfEuler createSbgEkfEulerMessage(const SbgLogEkfEulerData& ref_log_ekf_euler) const;

  /*!
   * Create a SBG-ROS Ekf Navigation message.
   * 
   * \param[in] ref_log_ekf_nav     SBG Ekf Navigation log.
   * \return                        Ekf Navigation message.
   */
  const sbg_driver::SbgEkfNav createSbgEkfNavMessage(const SbgLogEkfNavData& ref_log_ekf_nav) const;

  /*!
   * Create a SBG-ROS Ekf Quaternion message.
   * 
   * \param[in] ref_log_ekf_quat    SBG Ekf Quaternion log.
   * \return                        Ekf Quaternion message.
   */
  const sbg_driver::SbgEkfQuat createSbgEkfQuatMessage(const SbgLogEkfQuatData& ref_log_ekf_quat) const;

  /*!
   * Create a SBG-ROS event message.
   * 
   * \param[in] ref_log_event       SBG event log.
   * \return                        Event message.
   */
  const sbg_driver::SbgEvent createSbgEventMessage(const SbgLogEvent& ref_log_event) const;

  /*!
   * Create SBG-ROS GPS-HDT message.
   * 
   * \param[in] ref_log_gps_hdt     SBG GPS HDT log.
   * \return                        GPS HDT message.
   */
  const sbg_driver::SbgGpsHdt createSbgGpsHdtMessage(const SbgLogGpsHdt& ref_log_gps_hdt) const;

  /*!
   * Create a SBG-ROS GPS-Position message.
   * 
   * \param[in] ref_log_gps_pos     SBG GPS Position log.
   * \return                        GPS Position message.
   */
  const sbg_driver::SbgGpsPos createSbgGpsPosMessage(const SbgLogGpsPos& ref_log_gps_pos) const;

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
   * \return                        GPS Velocity message.
   */
  const sbg_driver::SbgGpsVel createSbgGpsVelMessage(const SbgLogGpsVel& ref_log_gps_vel) const;

  /*!
   * Create a SBG-ROS Imu data message.
   * 
   * \param[in] ref_log_imu_data    SBG Imu data log.
   * \return                        Imu data message.
   */
  const sbg_driver::SbgImuData createSbgImuDataMessage(const SbgLogImuData& ref_log_imu_data) const;

  /*!
   * Create a SBG-ROS Magnetometer message.
   * 
   * \param[in] ref_log_mag         SBG Magnetometer log.
   * \return                        Magnetometer message.
   */
  const sbg_driver::SbgMag createSbgMagMessage(const SbgLogMag& ref_log_mag) const;

  /*!
   * Create a SBG-ROS Magnetometer calibration message.
   * 
   * \param[in] ref_log_mag_calib   SBG Magnetometer calibration log.
   * \return                        Magnetometer calibration message.
   */
  const sbg_driver::SbgMagCalib createSbgMagCalibMessage(const SbgLogMagCalib& ref_log_mag_calib) const;

  /*!
   * Create a SBG-ROS Odometer velocity message.
   * 
   * \param[in] ref_log_odo         SBG Odometer log.
   * \return                        Odometer message.
   */
  const sbg_driver::SbgOdoVel createSbgOdoVelMessage(const SbgLogOdometerData& ref_log_odo) const;

  /*!
   * Create a SBG-ROS pressure message.
   * 
   * \param[in] ref_log_pressure    SBG Pressure log.
   * \return                        Pressure message.
   */
  const sbg_driver::SbgPressure createSbgPressureMessage(const SbgLogPressureData& ref_log_pressure) const;

  /*!
   * Create a SBG-ROS Shipmotion message.
   * 
   * \param[in] ref_log_ship_motion SBG Ship motion log.
   * \return                        Ship motion message.
   */
  const sbg_driver::SbgShipMotion createSbgShipMotionMessage(const SbgLogShipMotionData& ref_log_ship_motion) const;

  /*!
   * Create a SBG-ROS status message from a SBG status log.
   *
   * \param[in] ref_log_status      SBG status log.
   * \return                        Status message.
   */
  const sbg_driver::SbgStatus createSbgStatusMessage(const SbgLogStatusData& ref_log_status) const;

  /*!
   * Create a SBG-ROS UTC time message from a SBG UTC log.
   *
   * \param[in] ref_log_utc         SBG UTC log.
   * \return                        UTC time message.                  
   */
  const sbg_driver::SbgUtcTime createSbgUtcTimeMessage(const SbgLogUtcData& ref_log_utc) const;

  /*!
   * Create a ROS standard IMU message from SBG messages.
   * 
   * \param[in] ref_sbg_imu_msg     SBG-ROS IMU message.
   * \param[in] ref_sbg_quat_msg    SBG_ROS Quaternion message.
   * \return                        ROS standard IMU message.
   */
  const sensor_msgs::Imu createRosImuMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgEkfQuat& ref_sbg_quat_msg) const;

  /*!
   * Create a ROS standard Temperature message from SBG message.
   * 
   * \param[in] ref_sbg_imu_msg     SBG-ROS IMU message.
   * \return                        ROS standard Temperature message.
   */
  const sensor_msgs::Temperature createRosTemperatureMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg) const;

  /*!
   * Create a ROS standard MagneticField message from SBG message.
   * 
   * \param[in] ref_sbg_mag_msg     SBG-ROS Mag message.
   * \return                        ROS standard Mag message.
   */
  const sensor_msgs::MagneticField createRosMagneticMessage(const sbg_driver::SbgMag& ref_sbg_mag_msg) const;

  /*!
   * Create a ROS standard FluidPressure message from SBG message.
   * 
   * \param[in] ref_sbg_press_msg   SBG-ROS Fluid message.
   * \return                        ROS standard FuildPressure message.
   */
  const sensor_msgs::FluidPressure createRosFluidPressureMessage(const sbg_driver::SbgPressure& ref_sbg_press_msg) const;

  /*!
   * Create a ROS standard TwistStamped message from SBG messages.
   * 
   * \param[in] ref_sbg_imu_msg     SBG-ROS IMU message.
   * \param[in] ref_p_sbg_imu_msg   SBG-ROS IMU previous message.
   * \return                        ROS standard TwistStamped message.
   */
  const geometry_msgs::TwistStamped createRosTwistStampedMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgImuData& ref_p_sbg_imu_msg) const;

  /*!
   * Create a ROS standard PointStamped message from SBG messages.
   * 
   * \param[in] ref_sbg_ekf_msg     SBG-ROS EkfNav message.
   * \return                        ROS standard PointStamped message (ECEF).
   */
  const geometry_msgs::PointStamped createRosPointStampedMessage(const sbg_driver::SbgEkfNav& ref_sbg_ekf_msg) const;

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
   * \return                        ROS standard NavSatFix message.
   */
  const sensor_msgs::NavSatFix createRosNavSatFixMessage(const sbg_driver::SbgGpsPos& ref_sbg_gps_msg) const;
};
}

#endif