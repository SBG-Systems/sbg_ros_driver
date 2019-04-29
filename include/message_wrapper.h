#ifndef SBG_ROS_MESSAGE_WRAPPER_H
#define SBG_ROS_MESSAGE_WRAPPER_H

#include "ros/ros.h"

extern "C"
{
  #include <sbgEComLib.h>
  #include <sbgEComIds.h>
}

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

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  MessageWrapper(void);

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
};
}

#endif