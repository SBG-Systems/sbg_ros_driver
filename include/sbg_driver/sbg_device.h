#ifndef SBG_ROS_SBG_DEVICE_H
#define SBG_ROS_SBG_DEVICE_H

// Standard headers
#include <iostream>
#include <map>
#include <string>

// ROS headers
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

// Project headers
#include <config_applier.h>
#include <config_store.h>
#include <message_publisher.h>

namespace sbg
{
/*!
 * Class to handle a connected SBG device.
 */
class SbgDevice
{
private:

  SbgEComHandle           m_com_handle_;
  SbgInterface            m_sbg_interface_;
  ros::NodeHandle*        m_p_node_;
  MessagePublisher        m_message_publisher_;
  ConfigStore             m_config_store_;
  ConfigApplier           m_config_applier_;

  uint32                  m_rate_frequency_;

  bool                    m_mag_calibration_ongoing_;
  bool                    m_mag_calibration_done_;
  SbgEComMagCalibResults  m_magCalibResults;
  ros::ServiceServer      m_calib_service_;
  ros::ServiceServer      m_calib_save_service_;

  ros::Time               m_ros_processing_time_;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  /*!
   *  Callback definition called each time a new log is received.
   * 
   *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
   *  \param[in]  msgClass                Class of the message we have received
   *  \param[in]  msg                     Message ID of the log received.
   *  \param[in]  pLogData                Contains the received log data as an union.
   *  \param[in]  pUserArg                Optional user supplied argument.
   *  \return                             SBG_NO_ERROR if the received log has been used successfully.
   */
  static SbgErrorCode onLogReceivedCallback(SbgEComHandle* pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData* pLogData, void* pUserArg);

  /*!
   * Function to handle the received log.
   * 
   * \param[in]  msgClass               Class of the message we have received
   * \param[in]  msg                    Message ID of the log received.
   * \param[in]  pLogData               Contains the received log data as an union.
   */
  void onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData* pLogData);

  /*!
   * Load the parameters.
   */
  void loadParameters(void);

  /*!
   * Create the connection to the SBG device.
   * 
   * \throw                       Unable to connect to the SBG device.
   */
  void connect(void);

  /*!
   * Read the device informations.
   *
   * \throw                       Unable to read the device information.
   */
  void readDeviceInfo(void);

  /*!
   * Get the SBG version as a string.
   * 
   * \param[in] sbg_version_enc   SBG version encoded.
   * \return                      String version decoded.
   */
  std::string getVersionAsString(uint32 sbg_version_enc) const;

  /*!
   * Initialize the publishers according to the configuration.
   */
  void initPublishers(void);

  /*!
   * Configure the connected SBG device.
   */
  void configure(void);

  /*!
   * Process the magnetometer calibration.
   * 
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration process has been succesfull.
   */
  bool processMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response);

  /*!
   * Save the magnetometer calibration.
   * 
   * \param[in] ref_ros_request   ROS service request.
   * \param[in] ref_ros_response  ROS service response.
   * \return                      Return true if the calibration has been saved.
   */
  bool saveMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response);

  /*!
   * Start the magnetometer calibration process.
   * 
   * \return                      True if the calibration process has started successfully.
   */
  bool startMagCalibration(void);

  /*!
   * End the magnetometer calibration process.
   * 
   * \return                      True if the calibration process has ended successfully.
   */
  bool endMagCalibration(void);

  /*!
   * Upload the magnetometers calibration results to the device.
   * 
   * \return                      True if the magnetometers calibration has been successfully uploaded to the device.
   */
  bool uploadMagCalibrationToDevice(void);

  /*!
   * Display magnetometers calibration status result.
   */
  void displayMagCalibrationStatusResult(void) const;

  /*!
   * Export magnetometers calibration results.
   */
  void exportMagCalibrationResults(void) const;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   * 
   * \param[in] p_node_handle       ROS NodeHandle.
   */
  SbgDevice(ros::NodeHandle* p_node_handle);

  /*!
   * Default destructor.
   */
  ~SbgDevice(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the frequency to update the main rate loop for device handling.
   * 
   * \return                      Device frequency to read the logs (in Hz).
   */
  uint32 getUpdateFrequency(void) const;

  //---------------------------------------------------------------------//
  //- Public  methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the SBG device for receiving data.
   * 
   * \throw                       Unable to initialize the SBG device.
   */
  void initDeviceForReceivingData(void);

  /*!
   * Initialize the device for magnetometers calibration.
   */
  void initDeviceForMagCalibration(void);

  /*!
   * Periodic handle of the connected SBG device.
   */
  void periodicHandle(void);
};
}

static std::map<SbgEComMagCalibQuality, std::string> MAG_CALIB_QUAL= {{SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL, "Quality: optimal"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_GOOD, "Quality: good"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_POOR, "Quality: poor"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_INVALID, "Quality: invalid"}};

static std::map<SbgEComMagCalibConfidence, std::string> MAG_CALIB_CONF = {{SBG_ECOM_MAG_CALIB_TRUST_HIGH, "Confidence: high"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_MEDIUM, "Confidence: medium"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_LOW, "Confidence: low"}};

static std::map<SbgEComMagCalibMode, std::string> MAG_CALIB_MODE = {{SBG_ECOM_MAG_CALIB_MODE_2D, "Mode 2D"},
                                                                            {SBG_ECOM_MAG_CALIB_MODE_3D, "Mode 3D"}};

static std::map<SbgEComMagCalibBandwidth, std::string> MAG_CALIB_BW = {{SBG_ECOM_MAG_CALIB_HIGH_BW, "High Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_MEDIUM_BW, "Medium Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_LOW_BW, "Low Bandwidth"}};

#endif