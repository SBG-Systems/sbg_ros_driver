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
   * \param[in] sbg_msg_class           Class ID of the SBG message.
   * \param[in] sbg_msg_id              Id of the SBG message.
   * \param[in] ref_sbg_log             SBG binary log.
   */
  void publish(SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, const SbgBinaryLogData &ref_sbg_log) const;
};
}

#endif