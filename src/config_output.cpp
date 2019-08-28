#include "config_output.h"

using sbg::ConfigOutput;

/*!
 * Class to store the output configuration for the device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

ConfigOutput::ConfigOutput(void):
m_rebootNeeded(false),
m_ros_standard_messages_(false)
{

}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

void ConfigOutput::loadOutputConfigFor(ros::NodeHandle& ref_node_handle, const char* ref_key_string, SbgEComOutputMode* p_output_mode)
{
  *p_output_mode = static_cast<SbgEComOutputMode>(ref_node_handle.param<int>(ref_key_string, SBG_ECOM_OUTPUT_MODE_DISABLED));
}

void ConfigOutput::configureCommandOutput(SbgEComHandle* p_com_handle, SbgEComOutputPort output_port, SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_mode)
{
  SbgErrorCode      error_code;
  SbgEComOutputMode current_output_mode;

  //
  // Get the current output mode for the device and the selected log ID.
  // If output modes are different, udpate the device mode with the one loaded from the parameters.
  //
  error_code = sbgEComCmdOutputGetConf(p_com_handle, output_port, sbg_msg_class, sbg_msg_id, &current_output_mode);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the output configuration from the device %s", sbgErrorCodeToString(error_code));
  }

  if (current_output_mode != output_mode)
  {
    error_code = sbgEComCmdOutputSetConf(p_com_handle, output_port, sbg_msg_class, sbg_msg_id, output_mode);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the output configuration log ID %d from the device %s", sbg_msg_id, sbgErrorCodeToString(error_code));
    }
    else
    {
      m_rebootNeeded = true;
    }
  }
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

SbgEComOutputMode ConfigOutput::getOutputMode(SbgEComMsgId sbg_msg_id) const
{
  switch (sbg_msg_id)
  {
  case SBG_ECOM_LOG_STATUS:
    return m_log_status_;

  case SBG_ECOM_LOG_IMU_DATA:
    return m_log_imu_data_;

  case SBG_ECOM_LOG_EKF_EULER:
    return m_log_ekf_euler_;

  case SBG_ECOM_LOG_EKF_QUAT:
    return m_log_ekf_quat_;

  case SBG_ECOM_LOG_EKF_NAV:
    return m_log_ekf_nav_;

  case SBG_ECOM_LOG_SHIP_MOTION:
    return m_log_ship_motion_;

  case SBG_ECOM_LOG_UTC_TIME:
    return m_log_utc_time_;

  case SBG_ECOM_LOG_MAG:
    return m_log_mag_;

  case SBG_ECOM_LOG_MAG_CALIB:
    return m_log_mag_calib_;

  case SBG_ECOM_LOG_GPS1_VEL:
    return m_log_gps1_vel_;

  case SBG_ECOM_LOG_GPS1_POS:
    return m_log_gps1_pos_;

  case SBG_ECOM_LOG_GPS1_HDT:
    return m_log_gps1_hdt_;

  case SBG_ECOM_LOG_GPS1_RAW:
    return m_log_gps1_raw_;

  case SBG_ECOM_LOG_ODO_VEL:
    return m_log_odo_vel_;

  case SBG_ECOM_LOG_EVENT_A:
    return m_log_event_a_;

  case SBG_ECOM_LOG_EVENT_B:
    return m_log_event_b_;

  case SBG_ECOM_LOG_EVENT_C:
    return m_log_event_c_;

  case SBG_ECOM_LOG_EVENT_D:
    return m_log_event_d_;

  case SBG_ECOM_LOG_EVENT_E:
    return m_log_event_e_;

  case SBG_ECOM_LOG_PRESSURE:
    return m_log_pressure_;

  default:
    return SBG_ECOM_OUTPUT_MODE_DISABLED;
  }
}

int ConfigOutput::getRateFrequency(void) const
{
  return m_rate_frequency;
}

bool ConfigOutput::isRebootNeeded(void) const
{
  return m_rebootNeeded;
}

bool ConfigOutput::isRosStandardMessagesDefined(void) const
{
  return m_ros_standard_messages_;
}

int32 ConfigOutput::getLeapSeconds(void) const
{
  return m_leap_seconds;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void ConfigOutput::loadFromRosNodeHandle(ros::NodeHandle& ref_node_handle)
{
  loadOutputConfigFor(ref_node_handle, "output/log_status", &m_log_status_);
  loadOutputConfigFor(ref_node_handle, "output/log_imu_data", &m_log_imu_data_);
  loadOutputConfigFor(ref_node_handle, "output/log_ekf_euler", &m_log_ekf_euler_);
  loadOutputConfigFor(ref_node_handle, "output/log_ekf_quat", &m_log_ekf_quat_);
  loadOutputConfigFor(ref_node_handle, "output/log_ekf_nav", &m_log_ekf_nav_);
  loadOutputConfigFor(ref_node_handle, "output/log_ship_motion", &m_log_ship_motion_);
  loadOutputConfigFor(ref_node_handle, "output/log_utc_time", &m_log_utc_time_);
  loadOutputConfigFor(ref_node_handle, "output/log_mag", &m_log_mag_);
  loadOutputConfigFor(ref_node_handle, "output/log_mag_calib", &m_log_mag_calib_);
  loadOutputConfigFor(ref_node_handle, "output/log_gps1_vel", &m_log_gps1_vel_);
  loadOutputConfigFor(ref_node_handle, "output/log_gps1_pos", &m_log_gps1_pos_);
  loadOutputConfigFor(ref_node_handle, "output/log_gps1_hdt", &m_log_gps1_hdt_);
  loadOutputConfigFor(ref_node_handle, "output/log_gps1_raw", &m_log_gps1_raw_);
  loadOutputConfigFor(ref_node_handle, "output/log_odo_vel", &m_log_odo_vel_);
  loadOutputConfigFor(ref_node_handle, "output/log_event_a", &m_log_event_a_);
  loadOutputConfigFor(ref_node_handle, "output/log_event_b", &m_log_event_b_);
  loadOutputConfigFor(ref_node_handle, "output/log_event_c", &m_log_event_c_);
  loadOutputConfigFor(ref_node_handle, "output/log_event_d", &m_log_event_d_);
  loadOutputConfigFor(ref_node_handle, "output/log_event_e", &m_log_event_e_);
  loadOutputConfigFor(ref_node_handle, "output/log_pressure", &m_log_pressure_);

  m_rate_frequency          = ref_node_handle.param<int>("output/frequency", 0);
  m_ros_standard_messages_  = ref_node_handle.param<bool>("output/ros_standard", false);
  m_leap_seconds            = static_cast<int32>(ref_node_handle.param<int>("output/leap_seconds", 0));
}

void ConfigOutput::configureComHandle(SbgEComHandle* p_com_handle, SbgEComOutputPort output_port)
{
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, m_log_status_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, m_log_imu_data_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, m_log_ekf_euler_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, m_log_ekf_quat_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, m_log_ekf_nav_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, m_log_ship_motion_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, m_log_utc_time_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, m_log_mag_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB, m_log_mag_calib_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, m_log_gps1_vel_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, m_log_gps1_pos_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, m_log_gps1_hdt_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW, m_log_gps1_raw_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL, m_log_odo_vel_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A, m_log_event_a_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B, m_log_event_b_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C, m_log_event_c_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D, m_log_event_d_);
  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A, m_log_event_e_);

  configureCommandOutput(p_com_handle, output_port, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_PRESSURE, m_log_pressure_);
}