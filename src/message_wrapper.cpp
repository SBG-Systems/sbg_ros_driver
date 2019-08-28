#include "message_wrapper.h"

using sbg::MessageWrapper;

/*!
 * Class to wrap the SBG logs into ROS messages.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

MessageWrapper::MessageWrapper(void)
{

}

//---------------------------------------------------------------------//
//- Internal methods                                                  -//
//---------------------------------------------------------------------//

const ros::Time MessageWrapper::createRosTime(uint32 device_timestamp) const
{
  return m_ros_processing_time_;
}

const geometry_msgs::Vector3 MessageWrapper::createRosVector3(const float* p_float_array) const
{
  geometry_msgs::Vector3 vector3_message;

  vector3_message.x = p_float_array[0];
  vector3_message.y = p_float_array[1];
  vector3_message.z = p_float_array[2];

  return vector3_message;
}

const geometry_msgs::Vector3 MessageWrapper::createRosVector3(const double* p_double_array) const
{
  geometry_msgs::Vector3 vector3_message;

  vector3_message.x = p_double_array[0];
  vector3_message.y = p_double_array[1];
  vector3_message.z = p_double_array[2];

  return vector3_message;
}

const sbg_driver::SbgEkfStatus MessageWrapper::createEkfStatusMessage(uint32 ekf_status) const
{
  sbg_driver::SbgEkfStatus ekf_status_message;

  ekf_status_message.solution_mode    = sbgEComLogEkfGetSolutionMode(ekf_status);
  ekf_status_message.attitude_valid   = (ekf_status & SBG_ECOM_SOL_ATTITUDE_VALID) != 0;
  ekf_status_message.heading_valid    = (ekf_status & SBG_ECOM_SOL_HEADING_VALID) != 0;
  ekf_status_message.velocity_valid   = (ekf_status & SBG_ECOM_SOL_VELOCITY_VALID) != 0;
  ekf_status_message.position_valid   = (ekf_status & SBG_ECOM_SOL_POSITION_VALID) != 0;

  ekf_status_message.vert_ref_used    = (ekf_status & SBG_ECOM_SOL_VERT_REF_USED) != 0;
  ekf_status_message.mag_ref_used     = (ekf_status & SBG_ECOM_SOL_MAG_REF_USED) != 0;
  ekf_status_message.gps1_vel_used    = (ekf_status & SBG_ECOM_SOL_GPS1_VEL_USED) != 0;
  ekf_status_message.gps1_pos_used    = (ekf_status & SBG_ECOM_SOL_GPS1_POS_USED) != 0;
  ekf_status_message.gps1_course_used = (ekf_status & SBG_ECOM_SOL_GPS1_HDT_USED) != 0;
  ekf_status_message.gps1_hdt_used    = (ekf_status & SBG_ECOM_SOL_GPS1_HDT_USED) != 0;
  ekf_status_message.gps2_vel_used    = (ekf_status & SBG_ECOM_SOL_GPS2_VEL_USED) != 0;
  ekf_status_message.gps2_pos_used    = (ekf_status & SBG_ECOM_SOL_GPS2_POS_USED) != 0;
  ekf_status_message.gps2_course_used = (ekf_status & SBG_ECOM_SOL_GPS2_POS_USED) != 0;
  ekf_status_message.gps2_hdt_used    = (ekf_status & SBG_ECOM_SOL_GPS2_HDT_USED) != 0;
  ekf_status_message.odo_used         = (ekf_status & SBG_ECOM_SOL_ODO_USED) != 0;

  return ekf_status_message;
}

const sbg_driver::SbgGpsPosStatus MessageWrapper::createGpsPosStatusMessage(const SbgLogGpsPos& ref_log_gps_pos) const
{
  sbg_driver::SbgGpsPosStatus gps_pos_status_message;

  gps_pos_status_message.status       = sbgEComLogGpsPosGetStatus(ref_log_gps_pos.status);
  gps_pos_status_message.type         = sbgEComLogGpsPosGetType(ref_log_gps_pos.status);
  gps_pos_status_message.gps_l1_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L1_USED) != 0;
  gps_pos_status_message.gps_l2_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L2_USED) != 0;
  gps_pos_status_message.gps_l5_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GPS_L5_USED) != 0;
  gps_pos_status_message.glo_l1_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GLO_L1_USED) != 0;
  gps_pos_status_message.glo_l2_used  = (ref_log_gps_pos.status & SBG_ECOM_GPS_POS_GLO_L2_USED) != 0;

  return gps_pos_status_message;
}

const sbg_driver::SbgGpsVelStatus MessageWrapper::createGpsVelStatusMessage(const SbgLogGpsVel& ref_log_gps_vel) const
{
  sbg_driver::SbgGpsVelStatus gps_vel_status_message;

  gps_vel_status_message.vel_status = sbgEComLogGpsVelGetStatus(ref_log_gps_vel.status);
  gps_vel_status_message.vel_type   = sbgEComLogGpsVelGetType(ref_log_gps_vel.status);

  return gps_vel_status_message;
}

const sbg_driver::SbgImuStatus MessageWrapper::createImuStatusMessage(const SbgLogImuData& ref_log_imu) const
{
  sbg_driver::SbgImuStatus imu_status_message;

  imu_status_message.imu_com              = (ref_log_imu.status & SBG_ECOM_IMU_COM_OK) != 0;
  imu_status_message.imu_status           = (ref_log_imu.status & SBG_ECOM_IMU_STATUS_BIT) != 0 ;
  imu_status_message.imu_accel_x          = (ref_log_imu.status & SBG_ECOM_IMU_ACCEL_X_BIT) != 0;
  imu_status_message.imu_accel_y          = (ref_log_imu.status & SBG_ECOM_IMU_ACCEL_Y_BIT) != 0;
  imu_status_message.imu_accel_z          = (ref_log_imu.status & SBG_ECOM_IMU_ACCEL_Z_BIT) != 0;
  imu_status_message.imu_gyro_x           = (ref_log_imu.status & SBG_ECOM_IMU_GYRO_X_BIT) != 0;
  imu_status_message.imu_gyro_y           = (ref_log_imu.status & SBG_ECOM_IMU_GYRO_Y_BIT) != 0;
  imu_status_message.imu_gyro_z           = (ref_log_imu.status & SBG_ECOM_IMU_GYRO_Z_BIT) != 0;
  imu_status_message.imu_accels_in_range  = (ref_log_imu.status & SBG_ECOM_IMU_ACCELS_IN_RANGE) != 0;
  imu_status_message.imu_gyros_in_range   = (ref_log_imu.status & SBG_ECOM_IMU_GYROS_IN_RANGE) != 0;

  return imu_status_message;
}

const sbg_driver::SbgMagStatus MessageWrapper::createMagStatusMessage(const SbgLogMag& ref_log_mag) const
{
  sbg_driver::SbgMagStatus mag_status_message;

  mag_status_message.mag_x            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_X_BIT) != 0;
  mag_status_message.mag_y            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_Y_BIT) != 0;
  mag_status_message.mag_z            = (ref_log_mag.status & SBG_ECOM_MAG_MAG_Z_BIT) != 0;
  mag_status_message.accel_x          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_X_BIT) != 0;
  mag_status_message.accel_y          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_Y_BIT) != 0;
  mag_status_message.accel_z          = (ref_log_mag.status & SBG_ECOM_MAG_ACCEL_Z_BIT) != 0;
  mag_status_message.mags_in_range    = (ref_log_mag.status & SBG_ECOM_MAG_MAGS_IN_RANGE) != 0;
  mag_status_message.accels_in_range  = (ref_log_mag.status & SBG_ECOM_MAG_ACCELS_IN_RANGE) != 0;
  mag_status_message.calibration      = (ref_log_mag.status & SBG_ECOM_MAG_CALIBRATION_OK) != 0;

  return mag_status_message;
}

const sbg_driver::SbgShipMotionStatus MessageWrapper::createShipMotionStatusMessage(const SbgLogShipMotionData& ref_log_ship_motion) const
{
  sbg_driver::SbgShipMotionStatus ship_motion_status_message;

  ship_motion_status_message.heave_valid      = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_VALID) != 0;
  ship_motion_status_message.heave_vel_aided  = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_VEL_AIDED) != 0;
  ship_motion_status_message.period_available = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_PERIOD_INCLUDED) != 0;
  ship_motion_status_message.period_valid     = (ref_log_ship_motion.status & SBG_ECOM_HEAVE_PERIOD_VALID) != 0;

  return ship_motion_status_message;
}

const sbg_driver::SbgStatusAiding MessageWrapper::createStatusAidingMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusAiding status_aiding_message;

  status_aiding_message.gps1_pos_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_POS_RECV) != 0;
  status_aiding_message.gps1_vel_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_VEL_RECV) != 0;
  status_aiding_message.gps1_hdt_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_HDT_RECV) != 0;
  status_aiding_message.gps1_utc_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_GPS1_UTC_RECV) != 0;

  status_aiding_message.mag_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_MAG_RECV) != 0;
  status_aiding_message.odo_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_ODO_RECV) != 0;
  status_aiding_message.dvl_recv = (ref_log_status.aidingStatus & SBG_ECOM_AIDING_DVL_RECV) != 0;

  return status_aiding_message;
}

const sbg_driver::SbgStatusCom MessageWrapper::createStatusComMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusCom status_com_message;

  status_com_message.port_a = (ref_log_status.comStatus & SBG_ECOM_PORTA_VALID) != 0;
  status_com_message.port_b = (ref_log_status.comStatus & SBG_ECOM_PORTB_VALID) != 0;
  status_com_message.port_c = (ref_log_status.comStatus & SBG_ECOM_PORTC_VALID) != 0;
  status_com_message.port_d = (ref_log_status.comStatus & SBG_ECOM_PORTD_VALID) != 0;
  status_com_message.port_e = (ref_log_status.comStatus & SBG_ECOM_PORTE_VALID) != 0;

  status_com_message.port_a_rx = (ref_log_status.comStatus & SBG_ECOM_PORTA_RX_OK) != 0;
  status_com_message.port_a_tx = (ref_log_status.comStatus & SBG_ECOM_PORTA_TX_OK) != 0;
  status_com_message.port_b_rx = (ref_log_status.comStatus & SBG_ECOM_PORTB_RX_OK) != 0;
  status_com_message.port_b_tx = (ref_log_status.comStatus & SBG_ECOM_PORTB_TX_OK) != 0;
  status_com_message.port_c_rx = (ref_log_status.comStatus & SBG_ECOM_PORTC_RX_OK) != 0;
  status_com_message.port_c_tx = (ref_log_status.comStatus & SBG_ECOM_PORTC_TX_OK) != 0;
  status_com_message.port_d_rx = (ref_log_status.comStatus & SBG_ECOM_PORTD_RX_OK) != 0;
  status_com_message.port_d_tx = (ref_log_status.comStatus & SBG_ECOM_PORTD_TX_OK) != 0;
  status_com_message.port_e_rx = (ref_log_status.comStatus & SBG_ECOM_PORTE_RX_OK) != 0;
  status_com_message.port_e_tx = (ref_log_status.comStatus & SBG_ECOM_PORTE_TX_OK) != 0;

  status_com_message.can_rx     = (ref_log_status.comStatus & SBG_ECOM_CAN_RX_OK) != 0;
  status_com_message.can_tx     = (ref_log_status.comStatus & SBG_ECOM_CAN_TX_OK) != 0;
  status_com_message.can_status = (ref_log_status.comStatus & SBG_ECOM_CAN_VALID) != 0;

  return status_com_message;
}

const sbg_driver::SbgStatusGeneral MessageWrapper::createStatusGeneralMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatusGeneral status_general_message;

  status_general_message.main_power   = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_MAIN_POWER_OK) != 0;
  status_general_message.imu_power    = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_IMU_POWER_OK) != 0;
  status_general_message.gps_power    = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_GPS_POWER_OK) != 0;
  status_general_message.settings     = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_SETTINGS_OK) != 0;
  status_general_message.temperature  = (ref_log_status.generalStatus & SBG_ECOM_GENERAL_TEMPERATURE_OK) != 0;

  return status_general_message;
}

const sbg_driver::SbgUtcTimeStatus MessageWrapper::createUtcStatusMessage(const SbgLogUtcData& ref_log_utc) const
{
  sbg_driver::SbgUtcTimeStatus utc_status_message;

  utc_status_message.clock_stable     = (ref_log_utc.status & SBG_ECOM_CLOCK_STABLE_INPUT) != 0;
  utc_status_message.clock_utc_sync   = (ref_log_utc.status & SBG_ECOM_CLOCK_UTC_SYNC) != 0;

  utc_status_message.clock_status     = static_cast<uint8_t>(sbgEComLogUtcGetClockStatus(ref_log_utc.status));
  utc_status_message.clock_utc_status = static_cast<uint8_t>(sbgEComLogUtcGetClockUtcStatus(ref_log_utc.status));
  
  return utc_status_message;
}

uint32_t MessageWrapper::getNumberOfDaysInYear(uint16_t year) const
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

uint32_t MessageWrapper::getNumberOfDaysInMonth(uint16_t year, uint8_t month_index) const
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

bool MessageWrapper::isLeapYear(uint16_t year) const
{
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

uint32_t MessageWrapper::convertUtcTimeToEpoch(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  uint32_t days;
  uint32_t seconds;

  //
  // Convert the UTC time to Epoch(Unix) time, which is the elasped seconds since 1 Jan 1970.
  //
  days    = 0;
  seconds = 0;

  for (uint16 yearIndex = 1970; yearIndex < ref_sbg_utc_msg.year; yearIndex++)
  {
    days += getNumberOfDaysInYear(yearIndex); 
  }

  for (uint8_t monthIndex = 1; monthIndex < ref_sbg_utc_msg.month; monthIndex++)
  {
    days += getNumberOfDaysInMonth(ref_sbg_utc_msg.year, monthIndex);
  }

  days += ref_sbg_utc_msg.day - 1;

  seconds = days * 24;
  seconds = (seconds + ref_sbg_utc_msg.hour) * 60;
  seconds = (seconds + ref_sbg_utc_msg.min) * 60;
  seconds = seconds + ref_sbg_utc_msg.sec;

  return seconds;
}

uint32_t MessageWrapper::getUtcTimeOfWeek(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  uint32_t milli_seconds;

  milli_seconds = (ref_sbg_utc_msg.day - 1) * 24;
  milli_seconds = (milli_seconds + ref_sbg_utc_msg.hour) * 60;
  milli_seconds = (milli_seconds + ref_sbg_utc_msg.min) * 60;
  milli_seconds = (milli_seconds + ref_sbg_utc_msg.sec) * 1000;
  milli_seconds = milli_seconds + ref_sbg_utc_msg.nanosec / 1000;

  return milli_seconds;
}

uint32_t MessageWrapper::computeTimeOfWeek(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  //
  // Compute the applied leap second by the device firmware.
  //
  int32     applied_leap_seconds;
  uint32_t  utc_tow;

  applied_leap_seconds  = computeGpsTimeOfWeekDelta(ref_sbg_utc_msg.gps_tow, getUtcTimeOfWeek(ref_sbg_utc_msg));
  utc_tow               = addOffsetGpsTimeOfWeek(ref_sbg_utc_msg.gps_tow, -applied_leap_seconds);

  return addOffsetGpsTimeOfWeek(utc_tow, m_leap_seconds_ * 1000);
}

int32 MessageWrapper::computeGpsTimeOfWeekDelta(uint32_t gps_time_of_week_A, uint32_t gps_time_of_week_B) const
{
  int32 delta_gps_tow;

  //
  // Compute the difference between the time of week, and check for GPS rollover.
  //
  delta_gps_tow = gps_time_of_week_A - gps_time_of_week_B;

  if (delta_gps_tow > SBG_GPS_TIME_OF_WEEK_MS_HALF)
  {
    delta_gps_tow = delta_gps_tow - SBG_GPS_TIME_OF_WEEK_MS_MAX;
  }
  else if (delta_gps_tow < -SBG_GPS_TIME_OF_WEEK_MS_HALF)
  {
    delta_gps_tow = delta_gps_tow + SBG_GPS_TIME_OF_WEEK_MS_MAX;
  }

  return delta_gps_tow;
}

uint32_t MessageWrapper::addOffsetGpsTimeOfWeek(uint32_t gps_time_of_week, int32 offset) const
{
  int32 gps_time_of_week_offseted;

  //
  // Add the offset the Gps time of week and check for Gps rollover.
  //
  gps_time_of_week_offseted = gps_time_of_week + offset;

  if (gps_time_of_week_offseted < 0)
  {
    gps_time_of_week_offseted += SBG_GPS_TIME_OF_WEEK_MS_MAX;
  }
  else if (gps_time_of_week_offseted > SBG_GPS_TIME_OF_WEEK_MS_MAX)
  {
    gps_time_of_week_offseted -= SBG_GPS_TIME_OF_WEEK_MS_MAX;
  }

  return static_cast<uint32_t>(gps_time_of_week_offseted);
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

void MessageWrapper::setRosProcessingTime(const ros::Time& ref_ros_time)
{
  m_ros_processing_time_ = ref_ros_time;
}

void MessageWrapper::setLeapSeconds(int32 leap_seconds)
{
  m_leap_seconds_ = leap_seconds;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

const sbg_driver::SbgEkfEuler MessageWrapper::createSbgEkfEulerMessage(const SbgLogEkfEulerData& ref_log_ekf_euler) const
{
  sbg_driver::SbgEkfEuler ekf_euler_message;

  ekf_euler_message.header.stamp  = createRosTime(ref_log_ekf_euler.timeStamp);
  ekf_euler_message.time_stamp    = ref_log_ekf_euler.timeStamp;

  ekf_euler_message.status    = createEkfStatusMessage(ref_log_ekf_euler.status);
  ekf_euler_message.angle     = createRosVector3(ref_log_ekf_euler.euler);
  ekf_euler_message.accuracy  = createRosVector3(ref_log_ekf_euler.eulerStdDev);

  return ekf_euler_message;
}

const sbg_driver::SbgEkfNav MessageWrapper::createSbgEkfNavMessage(const SbgLogEkfNavData& ref_log_ekf_nav) const
{
  sbg_driver::SbgEkfNav ekf_nav_message;

  ekf_nav_message.header.stamp  = createRosTime(ref_log_ekf_nav.timeStamp);
  ekf_nav_message.time_stamp    = ekf_nav_message.time_stamp;

  ekf_nav_message.status            = createEkfStatusMessage(ref_log_ekf_nav.status);
  ekf_nav_message.velocity          = createRosVector3(ref_log_ekf_nav.velocity);
  ekf_nav_message.velocity_accuracy = createRosVector3(ref_log_ekf_nav.velocityStdDev);
  ekf_nav_message.position          = createRosVector3(ref_log_ekf_nav.position);
  ekf_nav_message.undulation        = ref_log_ekf_nav.undulation;

  return ekf_nav_message;
}

const sbg_driver::SbgEkfQuat MessageWrapper::createSbgEkfQuatMessage(const SbgLogEkfQuatData& ref_log_ekf_quat) const
{
  sbg_driver::SbgEkfQuat ekf_quat_message;

  ekf_quat_message.header.stamp = createRosTime(ref_log_ekf_quat.timeStamp);
  ekf_quat_message.time_stamp   = ref_log_ekf_quat.timeStamp;

  ekf_quat_message.status       = createEkfStatusMessage(ref_log_ekf_quat.status);
  ekf_quat_message.quaternion.x = ref_log_ekf_quat.quaternion[1];
  ekf_quat_message.quaternion.y = ref_log_ekf_quat.quaternion[2];
  ekf_quat_message.quaternion.z = ref_log_ekf_quat.quaternion[3];
  ekf_quat_message.quaternion.w = ref_log_ekf_quat.quaternion[0];
  ekf_quat_message.accuracy     = createRosVector3(ref_log_ekf_quat.eulerStdDev);

  return ekf_quat_message;
}

const sbg_driver::SbgEvent MessageWrapper::createSbgEventMessage(const SbgLogEvent& ref_log_event) const
{
  sbg_driver::SbgEvent event_message;

  event_message.header.stamp  = createRosTime(ref_log_event.timeStamp);
  event_message.time_stamp    = ref_log_event.timeStamp;

  event_message.overflow        = (ref_log_event.status & SBG_ECOM_EVENT_OVERFLOW) != 0;
  event_message.offset_0_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_0_VALID) != 0;
  event_message.offset_1_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_1_VALID) != 0;
  event_message.offset_2_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_2_VALID) != 0;
  event_message.offset_3_valid  = (ref_log_event.status & SBG_ECOM_EVENT_OFFSET_3_VALID) != 0;

  event_message.time_offset_0   = ref_log_event.timeOffset0;
  event_message.time_offset_1   = ref_log_event.timeOffset1;
  event_message.time_offset_2   = ref_log_event.timeOffset2;
  event_message.time_offset_3   = ref_log_event.timeOffset3;

  return event_message;
}

const sbg_driver::SbgGpsHdt MessageWrapper::createSbgGpsHdtMessage(const SbgLogGpsHdt& ref_log_gps_hdt) const
{
  sbg_driver::SbgGpsHdt gps_hdt_message;

  gps_hdt_message.header.stamp  = createRosTime(ref_log_gps_hdt.timeStamp);
  gps_hdt_message.time_stamp    = ref_log_gps_hdt.timeStamp;

  gps_hdt_message.status            = ref_log_gps_hdt.status;
  gps_hdt_message.tow               = ref_log_gps_hdt.timeOfWeek;
  gps_hdt_message.true_heading      = ref_log_gps_hdt.heading;
  gps_hdt_message.true_heading_acc  = ref_log_gps_hdt.headingAccuracy;
  gps_hdt_message.pitch             = ref_log_gps_hdt.pitch;
  gps_hdt_message.pitch_acc         = ref_log_gps_hdt.pitchAccuracy;

  return gps_hdt_message; 
}

const sbg_driver::SbgGpsPos MessageWrapper::createSbgGpsPosMessage(const SbgLogGpsPos& ref_log_gps_pos) const
{
  sbg_driver::SbgGpsPos gps_pos_message;

  gps_pos_message.header.stamp  = createRosTime(ref_log_gps_pos.timeStamp);
  gps_pos_message.time_stamp    = ref_log_gps_pos.timeStamp;

  gps_pos_message.status              = createGpsPosStatusMessage(ref_log_gps_pos);
  gps_pos_message.gps_tow             = ref_log_gps_pos.timeOfWeek;
  gps_pos_message.position.x          = ref_log_gps_pos.latitude;
  gps_pos_message.position.y          = ref_log_gps_pos.longitude;
  gps_pos_message.position.z          = ref_log_gps_pos.altitude;
  gps_pos_message.undulation          = ref_log_gps_pos.undulation;
  gps_pos_message.position_accuracy.x = ref_log_gps_pos.latitudeAccuracy;
  gps_pos_message.position_accuracy.y = ref_log_gps_pos.longitudeAccuracy;
  gps_pos_message.position_accuracy.z = ref_log_gps_pos.altitudeAccuracy;
  gps_pos_message.num_sv_used         = ref_log_gps_pos.numSvUsed;
  gps_pos_message.base_station_id     = ref_log_gps_pos.baseStationId;
  gps_pos_message.diff_age            = ref_log_gps_pos.differentialAge;

  return gps_pos_message;
}

const sbg_driver::SbgGpsRaw MessageWrapper::createSbgGpsRawMessage(const SbgLogGpsRaw& ref_log_gps_raw) const
{
  sbg_driver::SbgGpsRaw gps_raw_message;

  gps_raw_message.data.assign(ref_log_gps_raw.rawBuffer, ref_log_gps_raw.rawBuffer + ref_log_gps_raw.bufferSize);

  return gps_raw_message;
}

const sbg_driver::SbgGpsVel MessageWrapper::createSbgGpsVelMessage(const SbgLogGpsVel& ref_log_gps_vel) const
{
  sbg_driver::SbgGpsVel gps_vel_message;

  gps_vel_message.header.stamp  = createRosTime(ref_log_gps_vel.timeStamp);
  gps_vel_message.time_stamp    = ref_log_gps_vel.timeStamp;

  gps_vel_message.status      = createGpsVelStatusMessage(ref_log_gps_vel);
  gps_vel_message.gps_tow     = ref_log_gps_vel.timeOfWeek;
  gps_vel_message.vel         = createRosVector3(ref_log_gps_vel.velocity);
  gps_vel_message.vel_acc     = createRosVector3(ref_log_gps_vel.velocityAcc);
  gps_vel_message.course      = ref_log_gps_vel.course;
  gps_vel_message.course_acc  = ref_log_gps_vel.courseAcc;

  return gps_vel_message;
}

const sbg_driver::SbgImuData MessageWrapper::createSbgImuDataMessage(const SbgLogImuData& ref_log_imu_data) const
{
  sbg_driver::SbgImuData imu_data_message;

  imu_data_message.header.stamp = createRosTime(ref_log_imu_data.timeStamp);
  imu_data_message.time_stamp   = ref_log_imu_data.timeStamp;

  imu_data_message.imu_status   = createImuStatusMessage(ref_log_imu_data);
  imu_data_message.accel        = createRosVector3(ref_log_imu_data.accelerometers);
  imu_data_message.gyro         = createRosVector3(ref_log_imu_data.gyroscopes);
  imu_data_message.temp         = ref_log_imu_data.temperature;
  imu_data_message.delta_vel    = createRosVector3(ref_log_imu_data.deltaVelocity);
  imu_data_message.delta_angle  = createRosVector3(ref_log_imu_data.deltaAngle);

  return imu_data_message;
}

const sbg_driver::SbgMag MessageWrapper::createSbgMagMessage(const SbgLogMag& ref_log_mag) const
{
  sbg_driver::SbgMag mag_message;

  mag_message.header.stamp  = createRosTime(ref_log_mag.timeStamp);
  mag_message.time_stamp    = ref_log_mag.timeStamp;

  mag_message.mag     = createRosVector3(ref_log_mag.magnetometers);
  mag_message.accel   = createRosVector3(ref_log_mag.accelerometers);
  mag_message.status  = createMagStatusMessage(ref_log_mag);

  return mag_message;
}

const sbg_driver::SbgMagCalib MessageWrapper::createSbgMagCalibMessage(const SbgLogMagCalib& ref_log_mag_calib) const
{
  sbg_driver::SbgMagCalib mag_calib_message;

  // TODO. SbgMagCalib is not implemented.
  mag_calib_message.header.stamp = createRosTime(ref_log_mag_calib.timeStamp);

  return mag_calib_message;
}

const sbg_driver::SbgOdoVel MessageWrapper::createSbgOdoVelMessage(const SbgLogOdometerData& ref_log_odo) const
{
  sbg_driver::SbgOdoVel odo_vel_message;

  odo_vel_message.header.stamp  = createRosTime(ref_log_odo.timeStamp);
  odo_vel_message.time_stamp    = ref_log_odo.timeStamp;

  odo_vel_message.status  = ref_log_odo.status;
  odo_vel_message.vel     = ref_log_odo.velocity;

  return odo_vel_message;
}

const sbg_driver::SbgPressure MessageWrapper::createSbgPressureMessage(const SbgLogPressureData& ref_log_pressure) const
{
  sbg_driver::SbgPressure pressure_message;

  pressure_message.header.stamp = createRosTime(ref_log_pressure.timeStamp);
  pressure_message.time_stamp   = ref_log_pressure.timeStamp;

  pressure_message.valid_pressure = (ref_log_pressure.status & SBG_ECOM_PRESSURE_PRESSURE_VALID) != 0;
  pressure_message.valid_altitude = (ref_log_pressure.status & SBG_ECOM_PRESSURE_HEIGHT_VALID) != 0;
  pressure_message.pressure       = ref_log_pressure.pressure;
  pressure_message.altitude       = ref_log_pressure.height;

  return pressure_message;
}

const sbg_driver::SbgShipMotion MessageWrapper::createSbgShipMotionMessage(const SbgLogShipMotionData& ref_log_ship_motion) const
{
  sbg_driver::SbgShipMotion ship_motion_message;

  ship_motion_message.header.stamp  = createRosTime(ref_log_ship_motion.timeStamp);
  ship_motion_message.time_stamp    = ref_log_ship_motion.timeStamp;

  ship_motion_message.ship_motion   = createRosVector3(ref_log_ship_motion.shipMotion);
  ship_motion_message.acceleration  = createRosVector3(ref_log_ship_motion.shipAccel);
  ship_motion_message.velocity      = createRosVector3(ref_log_ship_motion.shipVel);
  ship_motion_message.status        = createShipMotionStatusMessage(ref_log_ship_motion);

  return ship_motion_message;
}

const sbg_driver::SbgStatus MessageWrapper::createSbgStatusMessage(const SbgLogStatusData& ref_log_status) const
{
  sbg_driver::SbgStatus status_message;

  status_message.header.stamp = createRosTime(ref_log_status.timeStamp);
  status_message.time_stamp   = ref_log_status.timeStamp;

  status_message.status_general = createStatusGeneralMessage(ref_log_status);
  status_message.status_com     = createStatusComMessage(ref_log_status);
  status_message.status_aiding  = createStatusAidingMessage(ref_log_status);

  return status_message;
}

const sbg_driver::SbgUtcTime MessageWrapper::createSbgUtcTimeMessage(const SbgLogUtcData& ref_log_utc) const
{
  sbg_driver::SbgUtcTime utc_time_message;

  utc_time_message.header.stamp = createRosTime(ref_log_utc.timeStamp);
  utc_time_message.time_stamp   = ref_log_utc.timeStamp;

  utc_time_message.clock_status = createUtcStatusMessage(ref_log_utc);
  utc_time_message.year         = ref_log_utc.year;
  utc_time_message.month        = ref_log_utc.month;
  utc_time_message.day          = ref_log_utc.day;
  utc_time_message.hour         = ref_log_utc.hour;
  utc_time_message.min          = ref_log_utc.minute;
  utc_time_message.sec          = ref_log_utc.second;
  utc_time_message.nanosec      = ref_log_utc.nanoSecond;
  utc_time_message.gps_tow      = ref_log_utc.gpsTimeOfWeek;

  return utc_time_message;
}

const sensor_msgs::Imu MessageWrapper::createRosImuMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgEkfQuat& ref_sbg_quat_msg) const
{
  sensor_msgs::Imu imu_ros_message;

  imu_ros_message.header.stamp = createRosTime(ref_sbg_imu_msg.time_stamp);

  imu_ros_message.orientation                       = ref_sbg_quat_msg.quaternion;
  imu_ros_message.angular_velocity                  = ref_sbg_imu_msg.gyro;
  imu_ros_message.angular_velocity_covariance[0]    = -1;
  imu_ros_message.linear_acceleration               = ref_sbg_imu_msg.accel;
  imu_ros_message.linear_acceleration_covariance[0] = -1;

  return imu_ros_message;
}

const sensor_msgs::Temperature MessageWrapper::createRosTemperatureMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg) const
{
  sensor_msgs::Temperature temperature_message;

  temperature_message.header.stamp    = createRosTime(ref_sbg_imu_msg.time_stamp);
  temperature_message.header.frame_id = "Imu temperature";
  temperature_message.temperature     = ref_sbg_imu_msg.temp;
  temperature_message.variance        = 0;

  return temperature_message;
}

const sensor_msgs::MagneticField MessageWrapper::createRosMagneticMessage(const sbg_driver::SbgMag& ref_sbg_mag_msg) const
{
  sensor_msgs::MagneticField magnetic_message;

  magnetic_message.header.stamp   = createRosTime(ref_sbg_mag_msg.time_stamp);
  magnetic_message.magnetic_field = ref_sbg_mag_msg.mag;

  return magnetic_message;
}

const sensor_msgs::FluidPressure MessageWrapper::createRosFluidPressureMessage(const sbg_driver::SbgPressure& ref_sbg_press_msg) const
{
  sensor_msgs::FluidPressure fluid_pressure_message;

  fluid_pressure_message.header.stamp   = createRosTime(ref_sbg_press_msg.time_stamp);
  fluid_pressure_message.fluid_pressure = ref_sbg_press_msg.pressure;
  fluid_pressure_message.variance       = 0;

  return fluid_pressure_message;
}

const geometry_msgs::TwistStamped MessageWrapper::createRosTwistStampedMessage(const sbg_driver::SbgImuData& ref_sbg_imu_msg, const sbg_driver::SbgImuData& ref_p_sbg_imu_msg) const
{
  geometry_msgs::TwistStamped twist_stamped_message;
  double delta_t;

  delta_t = (ref_sbg_imu_msg.time_stamp - ref_p_sbg_imu_msg.time_stamp) * 1e-6;

  twist_stamped_message.header.stamp  = createRosTime(ref_sbg_imu_msg.time_stamp);
  twist_stamped_message.twist.angular = ref_sbg_imu_msg.gyro;

  twist_stamped_message.twist.linear.x = (ref_sbg_imu_msg.accel.x - ref_p_sbg_imu_msg.accel.x) / delta_t;
  twist_stamped_message.twist.linear.y = (ref_sbg_imu_msg.accel.y - ref_p_sbg_imu_msg.accel.y) / delta_t;
  twist_stamped_message.twist.linear.z = (ref_sbg_imu_msg.accel.z - ref_p_sbg_imu_msg.accel.z) / delta_t;

  return twist_stamped_message;
}

const geometry_msgs::PointStamped MessageWrapper::createRosPointStampedMessage(const sbg_driver::SbgEkfNav& ref_sbg_ekf_msg) const
{
  geometry_msgs::PointStamped point_stamped_message;

  point_stamped_message.header.stamp    = createRosTime(ref_sbg_ekf_msg.time_stamp);
  point_stamped_message.header.frame_id = "IMU position in ECEF";

  //
  // Conversion from Geodetic coordinates to ECEF is based on World Geodetic System 1984 (WGS84).
  // Radius are expressed in meters, and latitute/longitude in radian.
  //
  double equatorial_radius;
  double polar_radius;
  double prime_vertical_radius;
  double eccentricity;
  double latitude;
  double longitude;

  equatorial_radius = 6378137.0;
  polar_radius      = 6356752.314245;
  eccentricity      = 1 - pow(polar_radius, 2) / pow(equatorial_radius, 2);
  latitude          = ref_sbg_ekf_msg.position.x * SBG_PI / 180;
  longitude         = ref_sbg_ekf_msg.position.y * SBG_PI / 180;

  prime_vertical_radius = equatorial_radius / sqrt(1 - pow(eccentricity, 2) * pow(sin(latitude), 2));

  point_stamped_message.point.x = (prime_vertical_radius + ref_sbg_ekf_msg.position.z) * cos(latitude) * cos(longitude);
  point_stamped_message.point.y = (prime_vertical_radius + ref_sbg_ekf_msg.position.z) * cos(latitude) * sin(longitude);
  point_stamped_message.point.z = ((pow(polar_radius, 2) / pow(equatorial_radius, 2)) * prime_vertical_radius + ref_sbg_ekf_msg.position.z) * sin(latitude);

  return point_stamped_message;
}

const sensor_msgs::TimeReference MessageWrapper::createRosUtcTimeReferenceMessage(const sbg_driver::SbgUtcTime& ref_sbg_utc_msg) const
{
  sensor_msgs::TimeReference utc_reference_message;

  utc_reference_message.header.stamp    = createRosTime(ref_sbg_utc_msg.time_stamp);
  utc_reference_message.header.frame_id = "UTC time reference";
  utc_reference_message.source          = "UTC time from device converted to Epoch";

  //
  // Check if the leap second is included in the UTC time.
  //
  if (ref_sbg_utc_msg.clock_status.clock_utc_status == SBG_ECOM_UTC_VALID)
  {
    utc_reference_message.time_ref.sec  = convertUtcTimeToEpoch(ref_sbg_utc_msg);
    utc_reference_message.time_ref.nsec = ref_sbg_utc_msg.nanosec;
  }
  else if (ref_sbg_utc_msg.clock_status.clock_utc_status == SBG_ECOM_UTC_NO_LEAP_SEC)
  {
    //
    // The leap second is not valid in the UTC time.
    // Compute the epoch time from Gps time, and with the known leap second.
    //
    uint32 utc_to_epoch_ms;
    utc_to_epoch_ms = convertUtcTimeToEpoch(ref_sbg_utc_msg) * 1000;
    utc_to_epoch_ms = utc_to_epoch_ms - getUtcTimeOfWeek(ref_sbg_utc_msg);
    utc_to_epoch_ms = utc_to_epoch_ms + computeTimeOfWeek(ref_sbg_utc_msg);

    utc_reference_message.time_ref.sec  = floor(utc_to_epoch_ms / 1000);
    utc_reference_message.time_ref.nsec = (utc_to_epoch_ms - floor(utc_to_epoch_ms / 1000)) * 1000 + ref_sbg_utc_msg.nanosec;
  }

  return utc_reference_message;
}

const sensor_msgs::NavSatFix MessageWrapper::createRosNavSatFixMessage(const sbg_driver::SbgGpsPos& ref_sbg_gps_msg) const
{
  sensor_msgs::NavSatFix nav_sat_fix_message;

  nav_sat_fix_message.header.stamp    = createRosTime(ref_sbg_gps_msg.time_stamp);
  nav_sat_fix_message.header.frame_id = "Navigation Satellite fix";

  if (ref_sbg_gps_msg.status.type == SBG_ECOM_POS_NO_SOLUTION)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_NO_FIX;
  }
  else if (ref_sbg_gps_msg.status.type == SBG_ECOM_POS_SBAS)
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_SBAS_FIX;
  }
  else
  {
    nav_sat_fix_message.status.status = nav_sat_fix_message.status.STATUS_FIX;
  }

  if (ref_sbg_gps_msg.status.glo_l1_used || ref_sbg_gps_msg.status.glo_l2_used)
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GLONASS;
  }
  else
  {
    nav_sat_fix_message.status.service = nav_sat_fix_message.status.SERVICE_GPS;
  }

  nav_sat_fix_message.latitude  = ref_sbg_gps_msg.position.x;
  nav_sat_fix_message.longitude = ref_sbg_gps_msg.position.y;
  nav_sat_fix_message.altitude  = ref_sbg_gps_msg.position.z;

  nav_sat_fix_message.position_covariance[0] = pow(ref_sbg_gps_msg.position_accuracy.x, 2);
  nav_sat_fix_message.position_covariance[4] = pow(ref_sbg_gps_msg.position_accuracy.y, 2);
  nav_sat_fix_message.position_covariance[8] = pow(ref_sbg_gps_msg.position_accuracy.z, 2);

  nav_sat_fix_message.position_covariance_type = nav_sat_fix_message.COVARIANCE_TYPE_DIAGONAL_KNOWN;

  return nav_sat_fix_message;
}