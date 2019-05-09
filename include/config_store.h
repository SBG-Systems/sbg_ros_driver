#ifndef SBG_ROS_CONFIG_STORE_H
#define SBG_ROS_CONFIG_STORE_H

#include "config_output.h"

namespace sbg
{
/*!
 * Class to handle the device configuration.
 */
class ConfigStore
{
private:

  std::string                 m_uart_port_name_;
  SbgEComOutputPort           m_output_port_;
  uint32                      m_uart_baud_rate_;

  SbgEComInitConditionConf    m_init_condition_conf_;
  SbgEComModelInfo            m_motion_profile_model_info_;

  SbgEComSensorAlignmentInfo  m_sensor_alignement_info_;
  float                       m_sensor_lever_arm_[3];

  SbgEComAidingAssignConf     m_aiding_assignement_conf_;

  SbgEComModelInfo            m_mag_model_info_;
  SbgEComMagRejectionConf     m_mag_rejection_conf_;
  SbgEComMagCalibMode         m_mag_calib_mode_;
  SbgEComMagCalibBandwidth    m_mag_calib_bandwidth_;

  SbgEComModelInfo            m_gnss_model_info_;
  SbgEComGnssAlignmentInfo    m_gnss_alignement_info_;
  SbgEComGnssRejectionConf    m_gnss_rejection_conf_;

  SbgEComOdoConf              m_odometer_conf_;
  float                       m_odometer_level_arm_[3];
  SbgEComOdoRejectionConf     m_odometer_rejection_conf_;

  bool                        m_rebootNeeded;

  ConfigOutput                m_config_output_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   * Load interface communication parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadCommunicationParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load sensor parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadSensorParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load IMU alignement parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadImuAlignementParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load aiding assignement parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadAidingAssignementParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load magnetometers parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadMagnetometersParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load Gnss parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadGnssParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Load odometer parameters.
   * 
   * \param[in] ref_node_handle   ROS nodeHandle.
   */
  void loadOdometerParameters(ros::NodeHandle& ref_node_handle);

  /*!
   * Configure the initial condition parameters.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the initial conditions.
   */
  void configureInitCondition(SbgEComHandle* p_com_handle);

  /*!
   * Configure the motion profile.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the motion profile.
   */
  void configureMotionProfile(SbgEComHandle* p_com_handle);

  /*!
   * Configure the IMU alignement.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the IMU alignement.
   */
  void configureImuAlignement(SbgEComHandle* p_com_handle);

  /*!
   * Configure the aiding assignement.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the aiding assignement.
   */
  void configureAidingAssignement(SbgEComHandle* p_com_handle);

  /*!
   * Configure the magnetometers model.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the magnetometers model.
   */
  void configureMagModel(SbgEComHandle* p_com_handle);

  /*!
   * Configure the magnetometers rejection.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the magnetometers rejection.
   */
  void configureMagRejection(SbgEComHandle* p_com_handle);

  /*!
   * Configure the Gnss model.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the Gnss model.
   */
  void configureGnssModel(SbgEComHandle* p_com_handle);

  /*!
   * Configure the Gnss level arm.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the Gnss level arm.
   */
  void configureGnssLevelArm(SbgEComHandle* p_com_handle);

  /*!
   * Configure the Gnss rejection.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the Gnss rejection.
   */
  void configureGnssRejection(SbgEComHandle* p_com_handle);

  /*!
   * Configure the odometer.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the odometer.
   */
  void configureOdometer(SbgEComHandle* p_com_handle);

  /*!
   * Configure the odometer level arm.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the odometer level arm.
   */
  void configureOdometerLevelArm(SbgEComHandle* p_com_handle);

  /*!
   * Configure the odometer rejection.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the odometer rejection.
   */
  void configureOdometerRejection(SbgEComHandle* p_com_handle);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   */
  ConfigStore(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Check if the device has to be rebooted.
   * 
   * \return                      True if the device has to be rebooted.
   */
  bool isRebootNeeded(void) const;

  /*!
   * Get the rate frequency.
   * 
   * \return                      Rate frequency parameter.
   */
  int getRateFrequency(void) const;

  /*!
   * Get the output configuration.
   * 
   * \return                      Output configuration.
   */
  const ConfigOutput &getOutputConfiguration(void) const;

  /*!
   * Get the magnetometers calibration mode.
   * 
   * \return                      Magnetometer calibration mode.
   */
  SbgEComMagCalibMode getMagCalibrationMode(void) const;

  /*!
   * Get the magnetometers calibration bandwidth.
   * 
   * \return                      Magnetometer calibration bandwidth.
   */
  SbgEComMagCalibBandwidth getMagCalibrationBandwidth(void) const;

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
   * Initialize the communication interface.
   * 
   * \param[in] p_sbg_interface   SBG communication interface.
   * \throw     Exception         Unable to initialize the communication interface.
   */
  void initCommunicationInterface(SbgInterface* p_sbg_interface) const;

  /*!
   * Configure the connected communication SBG handle.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to configure the SBG handle.
   */
  void configureComHandle(SbgEComHandle* p_com_handle);
};
}

#endif