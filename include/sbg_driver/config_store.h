/*!
*  \file         config_store.h
*  \author       SBG Systems
*  \date         13/03/2020
*
*  \brief        Class to handle the device configuration.
*
*  Methods to extract configuration values and load it to the ROS node.
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

#ifndef SBG_ROS_CONFIG_STORE_H
#define SBG_ROS_CONFIG_STORE_H

// SbgECom headers
#include <sbgEComLib.h>

// ROS headers
#include <ros/ros.h>

// Project headers
#include "sbg_vector3.h"

namespace sbg
{
  /*!
   * Time reference.
   */
  enum class TimeReference
  {
    ROS = 0,
    INS_UNIX = 1,
  };

/*!
 * Class to handle the device configuration.
 */
class ConfigStore
{
public:

  /*!
   * Structure to define the SBG log output.
   */
  struct SbgLogOutput
  {
    SbgEComClass              message_class;
    SbgEComMsgId              message_id;
    SbgEComOutputMode         output_mode;
  };

private:

  std::string                 uart_port_name_;
  SbgEComOutputPort           output_port_;
  uint32_t                    uart_baud_rate_;
  bool                        serial_communication_;

  sbgIpAddress                sbg_ip_address_;
  uint32_t                    out_port_address_;
  uint32_t                    in_port_address_;
  bool                        upd_communication_;

  bool                        configure_through_ros_;

  SbgEComInitConditionConf    init_condition_conf_;
  SbgEComModelInfo            motion_profile_model_info_;

  SbgEComSensorAlignmentInfo  sensor_alignement_info_;
  SbgVector3<float>           sensor_lever_arm_;

  SbgEComAidingAssignConf     aiding_assignement_conf_;

  SbgEComModelInfo            mag_model_info_;
  SbgEComMagRejectionConf     mag_rejection_conf_;
  SbgEComMagCalibMode         mag_calib_mode_;
  SbgEComMagCalibBandwidth    mag_calib_bandwidth_;

  SbgEComModelInfo            gnss_model_info_;
  SbgEComGnssInstallation     gnss_installation_;
  SbgEComGnssRejectionConf    gnss_rejection_conf_;

  SbgEComOdoConf              odometer_conf_;
  SbgVector3<float>           odometer_level_arm_;
  SbgEComOdoRejectionConf     odometer_rejection_conf_;

  std::vector<SbgLogOutput>   output_modes_;
  bool                        ros_standard_output_;

  TimeReference               time_reference_;

  uint32_t                    rate_frequency_;
  std::string                 frame_id_;
  bool                        use_enu_;

  bool                        odom_enable_;
  bool                        odom_publish_tf_;
  std::string                 odom_frame_id_;
  std::string                 odom_base_frame_id_;
  std::string                 odom_init_frame_id_;

  bool                        rtcm_subscribe_;
  std::string                 rtcm_full_topic_;

  bool                        nmea_publish_;
  std::string                 nmea_full_topic_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Get the ROS integer parameter casted in the T type.
   * This function has the same behavior as the param base function, however it enables an implicit cast, and the use of const NodeHandle.
   *
   * \template  T                 Template type to cast the ROS param to.
   * \param[in] ref_node_handle   ROS NodeHandle.
   * \param[in] param_key         Parameter key.
   * \param[in] default_value     Default value for the parameter.
   * \return                      ROS integer parameter casted.
   */
  template <typename T>
  T getParameter(const ros::NodeHandle& ref_node_handle, std::string param_key, int default_value) const
  {
    if (ref_node_handle.hasParam(param_key))
    {
      int parameter;
      ref_node_handle.param<int>(param_key, parameter, default_value);

      return static_cast<T>(parameter);
    }
    else
    {
      return static_cast<T>(default_value);
    }
  }

  /*!
   * Load driver parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadDriverParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load odometry parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadOdomParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load interface communication parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadCommunicationParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load sensor parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadSensorParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load IMU alignement parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadImuAlignementParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load aiding assignement parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadAidingAssignementParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load magnetometers parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadMagnetometersParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load Gnss parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadGnssParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load odometer parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadOdometerParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load frame parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
   void loadOutputFrameParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load the output configuration.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   * \param[in] ref_key           String key for the output config.
   * \param[in] sbg_msg_class     SBG message class.
   * \param[in] sbg_msg_id        ID of the SBG log.
   */
  void loadOutputConfiguration(const ros::NodeHandle& ref_node_handle, const std::string& ref_key, SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id);

  /*!
   * Load output time reference.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   * \param[in] ref_key           String key for the output config.
   */
  void loadOutputTimeReference(const ros::NodeHandle& ref_node_handle, const std::string& ref_key);

  /*!
   * Load RTCM parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadRtcmParameters(const ros::NodeHandle& ref_node_handle);

  /*!
   * Load NMEA parameters.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadNmeaParameters(const ros::NodeHandle& ref_node_handle);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  ConfigStore();

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Check if the configuration should be done with ROS.
   *
   * \return                      True if the ROS driver has to configure the device.
   */
  bool checkConfigWithRos() const;

  /*!
   * Check if the interface configuration is a serial interface.
   *
   * \return                      True if the interface is serial, False otherwise.
   */
  bool isInterfaceSerial() const;

  /*!
   * Get the UART port name.
   *
   * \return                      UART serial port name.
   */
  const std::string &getUartPortName() const;

  /*!
   * Get the UART baudrate communication.
   *
   * \return                      UART serial baudrate.
   */
  uint32_t getBaudRate() const;

  /*!
   * Get the output port of the device.
   *
   * \return                      SBG device output port.
   */
  SbgEComOutputPort getOutputPort() const;

  /*!
   * Check if the interface configuration is a UDP interface.
   *
   * \return                      True if the interface is UDP, False otherwise.
   */
  bool isInterfaceUdp() const;

  /*!
   * Get the Ip address of the interface.
   *
   * \return                      Ip address.
   */
  sbgIpAddress getIpAddress() const;

  /*!
   * Get the output port.
   *
   * \return                      Output port.
   */
  uint32_t getOutputPortAddress() const;

  /*!
   * Get the input port.
   *
   * \return                      Input port.
   */
  uint32_t getInputPortAddress() const;

  /*!
   * Get the initial conditions configuration.
   *
   * \return                                Initial conditions configuration.
   */
  const SbgEComInitConditionConf &getInitialConditions() const;

  /*!
   * Get the motion profile configuration.
   *
   * \return                                Motion profile configuration.
   */
  const SbgEComModelInfo &getMotionProfile() const;

  /*!
   * Get the sensor alignement configuration.
   *
   * \return                                Sensor alignement configuration.
   */
  const SbgEComSensorAlignmentInfo &getSensorAlignement() const;

  /*!
   * Get the sensor level arms.
   *
   * \return                                Sensor level arms vector (in meters).
   */
  const SbgVector3<float> &getSensorLevelArms() const;

  /*!
   * Get the aiding assignement configuration.
   *
   * \return                                Aiding assignement configuration.
   */
  const SbgEComAidingAssignConf &getAidingAssignement() const;

  /*!
   * Get the magnetometer model configuration.
   *
   * \return                                Magnetometer model configuration.
   */
  const SbgEComModelInfo &getMagnetometerModel() const;

  /*!
   * Get the magnetometer rejection configuration.
   *
   * \return                                Magnetometer rejection configuration.
   */
  const SbgEComMagRejectionConf &getMagnetometerRejection() const;

  /*!
   * Get the magnetometer calibration mode.
   *
   * \return                                Magnetometer calibration mode.
   */
  const SbgEComMagCalibMode &getMagnetometerCalibMode() const;

  /*!
   * Get the magnetometer calibration bandwidth.
   *
   * \return                                Magnetometer calibration bandwidth.
   */
  const SbgEComMagCalibBandwidth &getMagnetometerCalibBandwidth() const;

  /*!
   * Get the Gnss model configuration.
   *
   * \return                                Gnss model configuration.
   */
  const SbgEComModelInfo &getGnssModel() const;

  /*!
   * Get the Gnss installation configuration.
   *
   * \return                                Gnss installation configuration.
   */
  const SbgEComGnssInstallation &getGnssInstallation() const;

  /*!
   * Get the Gnss rejection configuration.
   *
   * \return                                Gnss rejection configuration.
   */
  const SbgEComGnssRejectionConf &getGnssRejection() const;

  /*!
   * Get the odometer configuration.
   *
   * \return                                Odometer configuration.
   */
  const SbgEComOdoConf &getOdometerConf() const;

  /*!
   * Get the odometer level arms.
   *
   * \return                                Odometer level arms vector (in meters).
   */
  const SbgVector3<float> &getOdometerLevelArms() const;

  /*!
   * Get the odometer rejection.
   *
   * \return                                Odometer rejection configuration.
   */
  const SbgEComOdoRejectionConf &getOdometerRejection() const;

  /*!
   * Get all the output modes.
   *
   * \return                      Output mode for this config store.
   */
  const std::vector<SbgLogOutput> &getOutputModes() const;

  /*!
   * Check if the ROS standard outputs are defined.
   *
   * \return                      True if standard ROS messages output are defined.
   */
  bool checkRosStandardMessages() const;

  /*!
   * Get the reading frequency defined in settings.
   * If this frequency is null, the driver will automatically configure the max output frequency according to the outputs.
   *
   * \return                      Rate frequency parameter (in Hz).
   */
  uint32_t getReadingRateFrequency() const;

  /*!
   * Get the frame ID.
   *
   * \return                      Frame ID.
   */
  const std::string &getFrameId() const;

  /*!
   * Get use ENU.
   *
   * \return                      True if the frame convention to use is ENU.
   */
  bool getUseEnu() const;

  /*!
   * Get odom enable.
   *
   * \return                      True if the odometry is enabled.
   */
  bool getOdomEnable() const;

  /*!
   * Get odom publish_tf.
   *
   * \return                      If true publish odometry transforms.
   */
  bool getOdomPublishTf() const;

  /*!
   * Get the odometry frame ID.
   *
   * \return                      Odometry frame ID.
   */
  const std::string &getOdomFrameId() const;

  /*!
   * Get the odometry base frame ID.
   *
   * \return                      Odometry base frame ID.
   */
  const std::string &getOdomBaseFrameId() const;

  /*!
   * Get the odometry init frame ID.
   *
   * \return                      Odometry init frame ID.
   */
  const std::string &getOdomInitFrameId() const;

  /*!
   * Get the time reference.
   *
   * \return                      Time reference.
   */
  TimeReference getTimeReference() const;

  /*!
   * Returns if the node should subscribe to RTCM publisher
   *
   * \return                      True to subscribe to RTCM messages.
   */
  bool shouldSubscribeToRtcm() const;

  /*!
   * Get RTCM full topic.
   *
   * \return                      String with RTCM namespace + topic.
   */
  const std::string &getRtcmFullTopic() const;

  /*!
   * Returns if a specific NMEA GGA message should be published or not.
   *
   * This GGA message is dedicated for NTRIP VRS operations and not for navigation.
   *
   * \return                      True if NMEA is enabled.
   */
  bool shouldPublishNmea() const;

  /*!
   * Get NMEA full topic.
   *
   * \return                      String with NMEA namespace + topic.
   */
  const std::string &getNmeaFullTopic() const;

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Load the configuration from a ros parameter handle.
   *
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadFromRosNodeHandle(const ros::NodeHandle& ref_node_handle);
};
}

#endif // SBG_ROS_CONFIG_STORE_H
