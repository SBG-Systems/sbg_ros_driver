#include <iomanip>
#include <fstream>
#include <ctime>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include "ellipse.h"

using namespace std;
using sbg::Ellipse;

// From ros_com/recorder
std::string timeToStr(ros::WallTime ros_t)
{
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

/*!
 * Class to handle a connected SBG Ellipse device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

Ellipse::Ellipse(ros::NodeHandle* p_node_handle):
m_p_node_(p_node_handle)
{
  loadParameters();
}

Ellipse::~Ellipse(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComClose(&m_com_handle_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to close the SBG communication handle - %s.", sbgErrorCodeToString(error_code));
  }

  error_code = sbgInterfaceSerialDestroy(&m_sbg_interface_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to close the serial interface - %s.", sbgErrorCodeToString(error_code));
  }
}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

SbgErrorCode Ellipse::onLogReceivedCallback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  Ellipse *p_ellipse;
  p_ellipse = (Ellipse*)(pUserArg);

  p_ellipse->onLogReceived(msgClass, msg, pLogData);

  return SBG_NO_ERROR;
}

void Ellipse::onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData)
{
  //
  // Publish the received SBG log.
  //
  m_message_publisher_.publish(msgClass, msg, *pLogData);
}

void Ellipse::loadParameters(void)
{
  //
  // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
  //
  ros::NodeHandle n_private("~");
  m_config_store_.loadFromRosNodeHandle(n_private);
}

void Ellipse::readDeviceInfo(SbgEComHandle* p_com_handle)
{
  SbgEComDeviceInfo device_info;
  SbgErrorCode      error_code; 
  
  error_code = sbgEComCmdGetInfo(p_com_handle, &device_info);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("Unable to get the device Info : " + std::string(sbgErrorCodeToString(error_code)));
  }

  ROS_INFO("SBG DRIVER - productCode = %s", device_info.productCode);
  ROS_INFO("SBG DRIVER - serialNumber = %u", device_info.serialNumber);

  ROS_INFO("SBG DRIVER - calibationRev = %s", getSbgVersionDecoded(device_info.calibationRev).c_str());
  ROS_INFO("SBG DRIVER - calibrationYear = %u", device_info.calibrationYear);
  ROS_INFO("SBG DRIVER - calibrationMonth = %u", device_info.calibrationMonth);
  ROS_INFO("SBG DRIVER - calibrationDay = %u", device_info.calibrationDay);

  ROS_INFO("SBG DRIVER - hardwareRev = %s", getSbgVersionDecoded(device_info.hardwareRev).c_str());
  ROS_INFO("SBG DRIVER - firmwareRev = %s", getSbgVersionDecoded(device_info.firmwareRev).c_str()); 
}

std::string Ellipse::getSbgVersionDecoded(uint32 sbg_version_enc) const
{
  char version[32];
  sbgVersionToStringEncoded(sbg_version_enc, version, 32);

  return std::string(version);
}

void Ellipse::initPublishers(void)
{
  m_message_publisher_.initPublishers(m_p_node_, m_config_store_.getOutputConfiguration());

  //
  // Check if the rate frequency has to be defined according to the defined publishers.
  //
  if(m_config_store_.getRateFrequency() == 0)
  {
    m_rate_frequency_ = m_message_publisher_.getOutputFrequency();
  }
  else
  {
    m_rate_frequency_ = m_config_store_.getRateFrequency();
  }
}

void Ellipse::configureEllipse(void)
{
  m_config_store_.configureComHandle(&m_com_handle_);

  if (m_config_store_.isRebootNeeded())
  {
    saveEllipseConfiguration();
  }
}

void Ellipse::saveEllipseConfiguration(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComCmdSettingsAction(&m_com_handle_, SBG_ECOM_SAVE_SETTINGS);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("Unable to save the settings on the Ellipse - " + std::string(sbgErrorCodeToString(error_code)));
  }

  ROS_INFO("SBG_DRIVER - Settings saved and device rebooted.");
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

int Ellipse::getDeviceRateFrequency(void) const
{
  return m_rate_frequency_;
}

//---------------------------------------------------------------------//
//- Public  methods                                                   -//
//---------------------------------------------------------------------//

void Ellipse::initEllipseForReceivingData(void)
{
  SbgErrorCode error_code;

  initPublishers();
  configureEllipse();

  error_code = sbgEComSetReceiveLogCallback(&m_com_handle_, onLogReceivedCallback, this);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to set the callback function - " + std::string(sbgErrorCodeToString(error_code)));
  }
}

void Ellipse::connect(void)
{
  SbgErrorCode error_code;

  m_config_store_.initCommunicationInterface(&m_sbg_interface_);
  error_code = sbgEComInit(&m_com_handle_, &m_sbg_interface_);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to initialize the SbgECom protocol - " + std::string(sbgErrorCodeToString(error_code)));
  }

  readDeviceInfo(&m_com_handle_);
}

void Ellipse::periodicHandle(void)
{
  sbgEComHandle(&m_com_handle_);
}

bool Ellipse::start_mag_calibration()
{
  SbgErrorCode errorCode;
  SbgEComMagCalibMode         mag_calib_mode;
  SbgEComMagCalibBandwidth    mag_calib_bandwidth;

  mag_calib_mode = m_config_store_.getMagCalibrationMode();
  mag_calib_bandwidth = m_config_store_.getMagCalibrationBandwidth();
  
  errorCode = sbgEComCmdMagStartCalib(&m_com_handle_, mag_calib_mode, mag_calib_bandwidth);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagStartCalib Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }else{
    ROS_INFO("SBG DRIVER - MAG CALIBRATION Start calibration");
    ROS_INFO("SBG DRIVER - MAG CALIBRATION mode : %s", MAG_CALIB_MODE[mag_calib_mode].c_str());
    ROS_INFO("SBG DRIVER - MAG CALIBRATION bandwidth : %s", MAG_CALIB_BW[mag_calib_bandwidth].c_str());
    return true;
  }
}

bool Ellipse::end_mag_calibration(){
  SbgErrorCode errorCode = sbgEComCmdMagComputeCalib(&m_com_handle_, &m_magCalibResults);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagStartCalib Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }

  ROS_INFO("SBG DRIVER - MAG CALIBRATION - %s", MAG_CALIB_QUAL[m_magCalibResults.quality].c_str());
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - %s", MAG_CALIB_CONF[m_magCalibResults.confidence].c_str());

  SbgEComMagCalibMode         mag_calib_mode;
  SbgEComMagCalibBandwidth    mag_calib_bandwidth;

  mag_calib_mode = m_config_store_.getMagCalibrationMode();
  mag_calib_bandwidth = m_config_store_.getMagCalibrationBandwidth();

  /// ************* WARNING IF ISSUES WITH COMPUTATIONS ************* //

  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - Not enough valid points. Maybe you are moving too fast");
  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - Unable to find a calibration solution. Maybe there are too much non static distortions");
  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - The magnetic calibration has troubles to correct the magnetometers and inertial frame alignment");
  if(mag_calib_mode == SBG_ECOM_MAG_CALIB_MODE_2D){
    if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
      ROS_WARN("SBG DRIVER - MAG CALIBRATION - Too much roll motion for a 2D magnetic calibration");
    if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
      ROS_WARN("SBG DRIVER - MAG CALIBRATION - Too much pitch motion for a 2D magnetic calibration");
  }
  else{
    if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
      ROS_WARN("SBG DRIVER - MAG CALIBRATION - Not enough roll motion for a 3D magnetic calibration");
    if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
      ROS_WARN("SBG DRIVER - MAG CALIBRATION - Not enough pitch motion for a 3D magnetic calibration.");
  }
  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Z_MOTION_ISSUE)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - Not enough yaw motion to compute a valid magnetic calibration");

  /// ************* Results ************* //

  ROS_INFO("SBG DRIVER - MAG CALIBRATION - Used Points: %u", m_magCalibResults.numPoints);
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - Max Points: %u", m_magCalibResults.maxNumPoints);
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - Mean, Std, Max");
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - [Before] %.2f %.2f %.2f", m_magCalibResults.beforeMeanError, m_magCalibResults.beforeStdError, m_magCalibResults.beforeMaxError);
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - [After] %.2f %.2f %.2f", m_magCalibResults.afterMeanError, m_magCalibResults.afterStdError, m_magCalibResults.afterMaxError);
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - Accuracy (deg) %0.2f %0.2f %0.2f", sbgRadToDegF(m_magCalibResults.meanAccuracy), sbgRadToDegF(m_magCalibResults.stdAccuracy), sbgRadToDegF(m_magCalibResults.maxAccuracy));

  /// ************* Save matrix to a file ************* //

  ostringstream oss;
  oss << "mag_calib";
  oss << timeToStr(ros::WallTime::now());
  oss << ".txt";

  ofstream save_file;
  save_file.open(oss.str());
  save_file << "Parameters" << endl;
  save_file << "* CALIB_MODE = " << MAG_CALIB_MODE[mag_calib_mode] << endl;
  save_file << "* CALIB_BW = " << MAG_CALIB_BW[mag_calib_bandwidth] << endl;
  save_file << "============================" << endl;
  save_file << "Results" << endl;
  save_file << MAG_CALIB_QUAL[m_magCalibResults.quality] << endl;
  save_file << MAG_CALIB_CONF[m_magCalibResults.confidence] << endl;
  save_file << "Infos" << endl;
  save_file << "* Used Points : " << m_magCalibResults.numPoints << "/" << m_magCalibResults.maxNumPoints << endl;
  save_file << "* Mean, Std, Max" << endl;
  save_file << "  [Before] " << m_magCalibResults.beforeMeanError << " " << m_magCalibResults.beforeStdError << " " << m_magCalibResults.beforeMaxError << endl;
  save_file << "  [After] " << m_magCalibResults.afterMeanError << " " << m_magCalibResults.afterStdError << " " << m_magCalibResults.afterMaxError << endl;
  save_file << "  [Accuracy] " << sbgRadToDegF(m_magCalibResults.meanAccuracy) << " " << sbgRadToDegF(m_magCalibResults.stdAccuracy) << " " << sbgRadToDegF(m_magCalibResults.maxAccuracy);
  save_file << "* Offset" << endl;
  save_file << m_magCalibResults.offset[0] << "\t" << m_magCalibResults.offset[1] << "\t" << m_magCalibResults.offset[2] << endl;
  save_file << "* Matrix" << endl;
  save_file << m_magCalibResults.matrix[0] << "\t" << m_magCalibResults.matrix[1] << "\t" << m_magCalibResults.matrix[2] << endl;
  save_file << m_magCalibResults.matrix[3] << "\t" << m_magCalibResults.matrix[4] << "\t" << m_magCalibResults.matrix[5] << endl;
  save_file << m_magCalibResults.matrix[6] << "\t" << m_magCalibResults.matrix[7] << "\t" << m_magCalibResults.matrix[8] << endl;
  save_file.close();
  const std::string tmp = oss.str();
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - Saving data to %s", tmp.c_str());

  return true;
}

bool Ellipse::save_mag_calibration(){
  SbgErrorCode errorCode = sbgEComCmdMagSetCalibData(&m_com_handle_, m_magCalibResults.offset, m_magCalibResults.matrix);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagSetCalibData Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }
  else{
    ROS_INFO("SBG DRIVER - MAG CALIBRATION - Saving data to the device");
    saveEllipseConfiguration();
    return true;
  }
}