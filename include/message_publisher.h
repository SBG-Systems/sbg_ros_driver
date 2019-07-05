#ifndef SBG_ROS_MESSAGE_PUBLISHER_H
#define SBG_ROS_MESSAGE_PUBLISHER_H

#include <config_output.h>
#include <message_wrapper.h>

namespace sbg
{
/*!
 * Class to publish all SBG-ROS messages to the corresponding publishers. 
 */
class MessagePublisher
{
private:

  ros::Publisher m_sbgStatus_pub_;
  ros::Publisher m_sbgUtcTime_pub_;
  ros::Publisher m_sbgImuData_pub_;
  ros::Publisher m_sbgEkfEuler_pub_;
  ros::Publisher m_sbgEkfQuat_pub_;
  ros::Publisher m_sbgEkfNav_pub_;
  ros::Publisher m_sbgShipMotion_pub_;
  ros::Publisher m_sbgMag_pub_;
  ros::Publisher m_sbgMagCalib_pub_;
  ros::Publisher m_sbgGpsVel_pub_;
  ros::Publisher m_sbgGpsPos_pub_;
  ros::Publisher m_sbgGpsHdt_pub_;
  ros::Publisher m_sbgGpsRaw_pub_;
  ros::Publisher m_sbgOdoVel_pub_;
  ros::Publisher m_sbgEventA_pub_;
  ros::Publisher m_sbgEventB_pub_;
  ros::Publisher m_sbgEventC_pub_;
  ros::Publisher m_sbgEventD_pub_;
  ros::Publisher m_sbgEventE_pub_;
  ros::Publisher m_sbgPressure_pub_;

  ros::Publisher          m_imu_pub_;
  sbg_driver::SbgImuData  m_sbg_imu_message_;
  sbg_driver::SbgEkfQuat  m_sbg_ekf_quat_message_;

  ros::Publisher m_temp_pub_;
  ros::Publisher m_mag_pub_;
  ros::Publisher m_fluid_pub_;
  ros::Publisher m_pos_ecef_pub_;
  ros::Publisher m_velocity_pub_;

  MessageWrapper    m_message_wrapper_;
  SbgEComOutputMode m_output_mode_;
  int               m_max_mesages_;

  //---------------------------------------------------------------------//
  //- Private methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Update the output configuration of the publisher.
   * 
   * \param[in] output_conf         Output configuration.
   */
  void updateOutputConfiguration(SbgEComOutputMode output_conf);

  /*!
   * Initialize the publisher for the specified SBG Id, and the output configuration.
   * 
   * \param[in] p_ros_node_handle       Ros NodeHandle to advertise the publisher.
   * \param[in] sbg_msg_id              Id of the SBG message.
   * \param[in] output_conf             Output configuration.
   * \param[in] ref_output_topic        Output topic for the publisher.
   */
  void initPublisher(ros::NodeHandle *p_ros_node_handle, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_conf, const std::string &ref_output_topic);

  /*!
   * Define standard ROS publishers.
   * 
   * \param[in] p_ros_node_handle       Ros NodeHandle to advertise the publisher.
   */
  void defineRosStandardPublishers(ros::NodeHandle *p_ros_node_handle);

  /*!
   * Publish a received SBG IMU log.
   * 
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishIMUData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Process a ROS IMU standard message.
   */
  void processRosImuMessage(void);

  /*!
   * Publish a received SBG Magnetic log.
   * 
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishMagData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG Fluid pressure log.
   * 
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishFluidPressureData(const SbgBinaryLogData &ref_sbg_log);

  /*!
   * Publish a received SBG EkfNav log.
   * 
   * \param[in] ref_sbg_log             SBG log.
   */
  void publishEkfNavigationData(const SbgBinaryLogData &ref_sbg_log);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  MessagePublisher(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the output frequency for the publisher.
   * 
   * \return                            Output frequency (in Hz).
   */
  int getOutputFrequency(void) const;

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the publishers for the output configuration.
   * 
   * \param[in] p_ros_node_handle       Ros NodeHandle to advertise the publisher.
   * \param[in] ref_output_config       Output configuration for the publishers.
   */
  void initPublishers(ros::NodeHandle *p_ros_node_handle, const ConfigOutput &ref_output_config); 

  /*!
   * Publish the received SbgLog if the corresponding publisher is defined.
   * 
   * \param[in] ref_ros_time            ROS processing time for the messages.
   * \param[in] sbg_msg_class           Class ID of the SBG message.
   * \param[in] sbg_msg_id              Id of the SBG message.
   * \param[in] ref_sbg_log             SBG binary log.
   */
  void publish(const ros::Time& ref_ros_time, SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, const SbgBinaryLogData &ref_sbg_log);
};
}

#endif