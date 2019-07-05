#ifndef SBG_ROS_CONFIG_OUTPUT_H
#define SBG_ROS_CONFIG_OUTPUT_H

#include "ros/ros.h"

extern "C"
{
#include <sbgEComLib.h>
}

namespace sbg
{
/*!
 * Class to store the output configuration for the device.
 */
class ConfigOutput
{
private:

  uint8 m_time_reference_;
  int   m_rate_frequency;
  bool  m_rebootNeeded;
  bool  m_ros_standard_messages_;

  SbgEComOutputMode m_log_status_;
  SbgEComOutputMode m_log_imu_data_;
  SbgEComOutputMode m_log_ekf_euler_;
  SbgEComOutputMode m_log_ekf_quat_;
  SbgEComOutputMode m_log_ekf_nav_;
  SbgEComOutputMode m_log_ship_motion_;
  SbgEComOutputMode m_log_utc_time_;
  SbgEComOutputMode m_log_mag_;
  SbgEComOutputMode m_log_mag_calib_;
  SbgEComOutputMode m_log_gps1_vel_;
  SbgEComOutputMode m_log_gps1_pos_;
  SbgEComOutputMode m_log_gps1_hdt_;
  SbgEComOutputMode m_log_gps1_raw_;
  SbgEComOutputMode m_log_odo_vel_;
  SbgEComOutputMode m_log_event_a_;
  SbgEComOutputMode m_log_event_b_;
  SbgEComOutputMode m_log_event_c_;
  SbgEComOutputMode m_log_event_d_;
  SbgEComOutputMode m_log_event_e_;
  SbgEComOutputMode m_log_pressure_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Load output configuration from ros parameter handle and key.
   * 
   * \param[in]   ref_node_handle   ROS nodeHandle.
   * \param[in]   ref_key_string    Key to load the parameter.
   * \param[out]  p_output_mode     Output mode to load configuration for.
   */
  void loadOutputConfigFor(ros::NodeHandle& ref_node_handle, const char* ref_key_string, SbgEComOutputMode* p_output_mode);

  /*!
   * Configure command output.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \param[in] output_port       Output communication port.
   * \param[in] sbg_msg_class     Class Id of the SBG message.
   * \param[in] sbg_msg_id        Id of the SBG message.
   * \param[in] output_mode       Output mode to configure.
   * \throw                       Unable to get the output mode.
   *                              Unable to set the output mode.
   */
  void configureCommandOutput(SbgEComHandle* p_com_handle, SbgEComOutputPort output_port, SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_mode); 

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  ConfigOutput(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the output configuration.
   * 
   * \param[in] sbg_msg_id        Id of the SBG message.
   * \return                      Output configuration.
   */
  SbgEComOutputMode getOutputMode(SbgEComMsgId sbg_msg_id) const;

  /*!
   * Get the rate frequency.
   * 
   * \return                      Rate frequency parameter.
   */
  int getRateFrequency(void) const;

  /*!
   * Check if the device has to be rebooted.
   * 
   * \return                      True if the device has to be rebooted.
   */
  bool isRebootNeeded(void) const;

  /*!
   * Check if the ROS standard messages have to be published.
   * 
   * \return                      True if the ROS standard publishers have to be configured.
   */
  bool isRosStandardMessagesDefined(void) const;

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Load the configuration from a ros parameter handle.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadFromRosNodeHandle(ros::NodeHandle& ref_node_handle);

  /*!
   * Configure the connected communication SBG handle.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \param[in] output_port       Output communication port.
   * \throw                       Unable to configure the output modes.
   */
  void configureComHandle(SbgEComHandle* p_com_handle, SbgEComOutputPort output_port);
};
}

#endif