#ifndef SBG_ROS_ELLIPSE_H
#define SBG_ROS_ELLIPSE_H

#include <iostream>
#include <map>
#include <string>

#include <message_publisher.h>
#include <config_store.h>

namespace sbg
{
/*!
 * Class to handle a connected SBG Ellipse device.
 */
class Ellipse
{
private:

  SbgEComHandle           m_com_handle_;
  SbgInterface            m_sbg_interface_;
  ros::NodeHandle*        m_p_node_;
  MessagePublisher        m_message_publisher_;
  ConfigStore             m_config_store_;

  int                     m_rate_frequency_;
  SbgEComMagCalibResults  m_magCalibResults;

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
  static SbgErrorCode onLogReceivedCallback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

  /*!
   * Function to handle the received log.
   * 
   * \param[in]  msgClass               Class of the message we have received
   * \param[in]  msg                    Message ID of the log received.
   * \param[in]  pLogData               Contains the received log data as an union.
   */
  void onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData);

  /*!
   * Load the parameters.
   */
  void loadParameters(void);

  /*!
   * Read the device informations.
   * 
   * \param[in] p_com_handle      SBG communication handle.
   * \throw                       Unable to read the device information.
   */
  void readDeviceInfo(SbgEComHandle* p_com_handle);

  /*!
   * Get the SBG version decoded to string.
   * 
   * \param[in] sbg_version_enc   SBG version encoded.
   * \return                      String version decoded.
   */
  std::string getSbgVersionDecoded(uint32 sbg_version_enc) const;

  /*!
   * Initialize the publishers according to the configuration.
   */
  void initPublishers(void);

  /*!
   * Configure the connected Ellipse.
   */
  void configureEllipse(void);

  /*!
   * Save the configuration of the connected Ellipse.
   * 
   * \throw                       Unable to save the configuration.
   */
  void saveEllipseConfiguration(void);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   * 
   * \param[in] p_node_handle       ROS NodeHandle.
   */
  Ellipse(ros::NodeHandle* p_node_handle);

  /*!
   * Default destructor.
   */
  ~Ellipse(void);

  //---------------------------------------------------------------------//
  //- Parameters                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Get the device frequency.
   * 
   * \return                      Device frequency to read the logs.
   */
  int getDeviceRateFrequency(void) const;

  //---------------------------------------------------------------------//
  //- Public  methods                                                   -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the Ellipse for receiving data.
   * 
   * \throw                       Unable to initialize the Ellipse.
   */
  void initEllipseForReceivingData(void);

  /*!
   * Create the connection to the Ellipse device.
   * 
   * \throw                       Unable to connect to the Ellipse.
   */
  void connect(void);

  /*!
   * Periodic handle of the connected Ellipse.
   */
  void periodicHandle(void);

  // TODO. Improve magnetometer calibration.
  bool start_mag_calibration();
  bool end_mag_calibration();
  bool save_mag_calibration();
};
}

static std::map<SbgEComMagCalibQuality, std::string> MAG_CALIB_QUAL= {{SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL, "Quality: optimal"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_GOOD, "Quality: good"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_POOR, "Quality: poor"}};

static std::map<SbgEComMagCalibConfidence, std::string> MAG_CALIB_CONF = {{SBG_ECOM_MAG_CALIB_TRUST_HIGH, "Confidence: high"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_MEDIUM, "Confidence: medium"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_LOW, "Confidence: low"}};

static std::map<SbgEComMagCalibMode, std::string> MAG_CALIB_MODE = {{SBG_ECOM_MAG_CALIB_MODE_2D, "Mode 2D"},
                                                                            {SBG_ECOM_MAG_CALIB_MODE_3D, "Mode 3D"}};

static std::map<SbgEComMagCalibBandwidth, std::string> MAG_CALIB_BW = {{SBG_ECOM_MAG_CALIB_HIGH_BW, "High Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_MEDIUM_BW, "Medium Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_LOW_BW, "Low Bandwidth"}};

#endif