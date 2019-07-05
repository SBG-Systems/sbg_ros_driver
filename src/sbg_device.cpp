#include <iomanip>
#include <fstream>
#include <ctime>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include "sbg_device.h"

using namespace std;
using sbg::SbgDevice;

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
 * Class to handle a connected SBG device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

SbgDevice::SbgDevice(ros::NodeHandle* p_node_handle):
m_p_node_(p_node_handle),
m_mag_calibration_ongoing_(false),
m_mag_calibration_done_(false)
{
  loadParameters();
  connect();
}

SbgDevice::~SbgDevice(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComClose(&m_com_handle_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to close the SBG communication handle - %s.", sbgErrorCodeToString(error_code));
  }

  m_config_store_.closeCommunicationInterface(&m_sbg_interface_);
}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

SbgErrorCode SbgDevice::onLogReceivedCallback(SbgEComHandle* pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData* pLogData, void* pUserArg)
{
  SbgDevice *p_sbg_device;
  p_sbg_device = (SbgDevice*)(pUserArg);

  p_sbg_device->onLogReceived(msgClass, msg, pLogData);

  return SBG_NO_ERROR;
}

void SbgDevice::onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData* pLogData)
{
  //
  // Publish the received SBG log.
  //
  m_message_publisher_.publish(m_ros_processing_time_, msgClass, msg, *pLogData);
}

void SbgDevice::loadParameters(void)
{
  //
  // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
  //
  ros::NodeHandle n_private("~");
  m_config_store_.loadFromRosNodeHandle(n_private);
}

void SbgDevice::connect(void)
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

void SbgDevice::readDeviceInfo(SbgEComHandle* p_com_handle)
{
  SbgEComDeviceInfo device_info;
  SbgErrorCode      error_code; 
  
  error_code = sbgEComCmdGetInfo(p_com_handle, &device_info);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to get the device Info : %s", sbgErrorCodeToString(error_code));
  }

  ROS_INFO("SBG DRIVER - productCode = %s", device_info.productCode);
  ROS_INFO("SBG DRIVER - serialNumber = %u", device_info.serialNumber);

  ROS_INFO("SBG DRIVER - calibationRev = %s", getSbgVersionDecoded(device_info.calibationRev).c_str());
  ROS_INFO("SBG DRIVER - calibrationDate = %u / %u / %u", device_info.calibrationDay, device_info.calibrationMonth, device_info.calibrationYear);

  ROS_INFO("SBG DRIVER - hardwareRev = %s", getSbgVersionDecoded(device_info.hardwareRev).c_str());
  ROS_INFO("SBG DRIVER - firmwareRev = %s", getSbgVersionDecoded(device_info.firmwareRev).c_str()); 
}

std::string SbgDevice::getSbgVersionDecoded(uint32 sbg_version_enc) const
{
  char version[32];
  sbgVersionToStringEncoded(sbg_version_enc, version, 32);

  return std::string(version);
}

void SbgDevice::initPublishers(void)
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

void SbgDevice::configure(void)
{
  m_config_store_.configureComHandle(&m_com_handle_);

  if (m_config_store_.isRebootNeeded())
  {
    saveDeviceConfiguration();
  }
}

void SbgDevice::saveDeviceConfiguration(void)
{
  SbgErrorCode error_code;

  error_code = sbgEComCmdSettingsAction(&m_com_handle_, SBG_ECOM_SAVE_SETTINGS);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to save the settings on the SBG device - %s", sbgErrorCodeToString(error_code));
  }
  else
  {
    ROS_INFO("SBG_DRIVER - Settings saved and device rebooted.");
  }
}

bool SbgDevice::processMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response)
{
  if (m_mag_calibration_ongoing_)
  {
    if (endMagCalibration())
    {
      ref_ros_response.success = true;
      ref_ros_response.message = "Magnetometer calibration is finished. See the output console to get calibration informations.";
    }
    else
    {
      ref_ros_response.success = false;
      ref_ros_response.message = "Unable to end the calibration.";
    }

    m_mag_calibration_ongoing_  = false;
    m_mag_calibration_done_     = true;
  }
  else
  {
    if (startMagCalibration())
    {
      ref_ros_response.success = true;
      ref_ros_response.message = "Magnetometer calibration process started.";
    }
    else
    {
      ref_ros_response.success = false;
      ref_ros_response.message = "Unable to start magnetometers calibration.";
    }

    m_mag_calibration_ongoing_ = true;
  }

  return ref_ros_response.success;
}

bool SbgDevice::saveMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response)
{
  if (m_mag_calibration_ongoing_)
  {
    ref_ros_response.success = false;
    ref_ros_response.message = "Magnetometer calibration process is still ongoing, finish it before trying to save it.";
  }
  else if (m_mag_calibration_done_)
  {
    if (uploadMagCalibrationToDevice())
    {
      ref_ros_response.success = true;
      ref_ros_response.message = "Magnetometer calibration has been uploaded to the device.";
    }
    else
    {
      ref_ros_response.success = false;
      ref_ros_response.message = "Magnetometer calibration has not been uploaded to the device.";
    }
  }
  else
  {
    ref_ros_response.success = false;
    ref_ros_response.message = "No magnetometer calibration has been done.";
  }

  return ref_ros_response.success;
}

bool SbgDevice::startMagCalibration(void)
{
  SbgErrorCode              error_code;
  SbgEComMagCalibMode       mag_calib_mode;
  SbgEComMagCalibBandwidth  mag_calib_bandwidth;

  mag_calib_mode      = m_config_store_.getMagCalibrationMode();
  mag_calib_bandwidth = m_config_store_.getMagCalibrationBandwidth();
  
  error_code = sbgEComCmdMagStartCalib(&m_com_handle_, mag_calib_mode, mag_calib_bandwidth);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Unable to start the magnetometer calibration : %s", sbgErrorCodeToString(error_code));
    return false;
  }
  else
  {
    ROS_INFO("SBG DRIVER [Mag Calib] - Start calibration");
    ROS_INFO("SBG DRIVER [Mag Calib] - Mode : %s", MAG_CALIB_MODE[mag_calib_mode].c_str());
    ROS_INFO("SBG DRIVER [Mag Calib] - Bandwidth : %s", MAG_CALIB_BW[mag_calib_bandwidth].c_str());
    return true;
  }
}

bool SbgDevice::endMagCalibration(void)
{
  SbgErrorCode error_code;
  
  error_code = sbgEComCmdMagComputeCalib(&m_com_handle_, &m_magCalibResults);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Unable to compute the magnetometer calibration results : %s", sbgErrorCodeToString(error_code));
    return false;
  }
  else
  {
    displayMagCalibrationStatusResult();
    exportMagCalibrationResults();

    return true;
  }
}

bool SbgDevice::uploadMagCalibrationToDevice(void)
{
  SbgErrorCode error_code;

  if (m_magCalibResults.quality != SBG_ECOM_MAG_CALIB_QUAL_INVALID)
  {
    error_code = sbgEComCmdMagSetCalibData(&m_com_handle_, m_magCalibResults.offset, m_magCalibResults.matrix);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Unable to set the magnetometers calibration data to the device : %s", sbgErrorCodeToString(error_code));
      return false;
    }
    else
    {
      ROS_INFO("SBG DRIVER [Mag Calib] - Saving data to the device");
      saveDeviceConfiguration();
      return true;
    }
  }
  else
  {
    ROS_ERROR("SBG DRIVER [Mag Calib] - The calibration was invalid, it can't be uploaded on the device.");
    return false;
  }
}

void SbgDevice::displayMagCalibrationStatusResult(void) const
{
  ROS_INFO("SBG DRIVER [Mag Calib] - Quality of the calibration %s", MAG_CALIB_QUAL[m_magCalibResults.quality].c_str());
  ROS_INFO("SBG DRIVER [Mag Calib] - Calibration results confidence %s", MAG_CALIB_CONF[m_magCalibResults.confidence].c_str());

  SbgEComMagCalibMode         mag_calib_mode;
  SbgEComMagCalibBandwidth    mag_calib_bandwidth;

  mag_calib_mode      = m_config_store_.getMagCalibrationMode();
  mag_calib_bandwidth = m_config_store_.getMagCalibrationBandwidth();

  //
  // Check the magnetometers calibration status and display the warnings.
  //
  if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Not enough valid points. Maybe you are moving too fast");
  }
  if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Unable to find a calibration solution. Maybe there are too much non static distortions");
  }   
  if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - The magnetic calibration has troubles to correct the magnetometers and inertial frame alignment");
  }
  if (mag_calib_mode == SBG_ECOM_MAG_CALIB_MODE_2D)
  {
    if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Too much roll motion for a 2D magnetic calibration");
    }
    if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Too much pitch motion for a 2D magnetic calibration");
    }
  }
  else
  {
    if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Not enough roll motion for a 3D magnetic calibration");
    }
    if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Not enough pitch motion for a 3D magnetic calibration.");
    }
  }
  if (m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_Z_MOTION_ISSUE)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Not enough yaw motion to compute a valid magnetic calibration");
  }
}

void SbgDevice::exportMagCalibrationResults(void) const
{
  SbgEComMagCalibMode       mag_calib_mode;
  SbgEComMagCalibBandwidth  mag_calib_bandwidth;
  ostringstream             mag_results_stream;
  string                    output_filename;

  mag_calib_mode      = m_config_store_.getMagCalibrationMode();
  mag_calib_bandwidth = m_config_store_.getMagCalibrationBandwidth();
  
  mag_results_stream << "SBG DRIVER [Mag Calib]" << endl;
  mag_results_stream << "======= Parameters =======" << endl;
  mag_results_stream << "* CALIB_MODE = " << MAG_CALIB_MODE[mag_calib_mode] << endl;
  mag_results_stream << "* CALIB_BW = " << MAG_CALIB_BW[mag_calib_bandwidth] << endl;

  mag_results_stream << "======= Results =======" << endl;
  mag_results_stream << MAG_CALIB_QUAL[m_magCalibResults.quality] << endl;
  mag_results_stream << MAG_CALIB_CONF[m_magCalibResults.confidence] << endl;
  mag_results_stream << "======= Infos =======" << endl;
  mag_results_stream << "* Used points : " << m_magCalibResults.numPoints << "/" << m_magCalibResults.maxNumPoints << endl;
  mag_results_stream << "* Mean, Std, Max" << endl;
  mag_results_stream << "[Before]\t" << m_magCalibResults.beforeMeanError << "\t" <<  m_magCalibResults.beforeStdError << "\t" << m_magCalibResults.beforeMaxError << endl;
  mag_results_stream << "[After]\t" << m_magCalibResults.afterMeanError << "\t" << m_magCalibResults.afterStdError << "\t" << m_magCalibResults.afterMaxError << endl;
  mag_results_stream << "[Accuracy]\t" << sbgRadToDegF(m_magCalibResults.meanAccuracy) << "\t" << sbgRadToDegF(m_magCalibResults.stdAccuracy) << "\t" << sbgRadToDegF(m_magCalibResults.maxAccuracy) << endl;
  mag_results_stream << "* Offset\t" << m_magCalibResults.offset[0] << "\t" << m_magCalibResults.offset[1] << "\t" << m_magCalibResults.offset[2] << endl;
  
  mag_results_stream << "* Matrix" << endl;
  mag_results_stream << m_magCalibResults.matrix[0] << "\t" << m_magCalibResults.matrix[1] << "\t" << m_magCalibResults.matrix[2] << endl;
  mag_results_stream << m_magCalibResults.matrix[3] << "\t" << m_magCalibResults.matrix[4] << "\t" << m_magCalibResults.matrix[5] << endl;
  mag_results_stream << m_magCalibResults.matrix[6] << "\t" << m_magCalibResults.matrix[7] << "\t" << m_magCalibResults.matrix[8] << endl;

  output_filename = "mag_calib_" + timeToStr(ros::WallTime::now()) + ".txt";
  ofstream output_file(output_filename);
  output_file << mag_results_stream.str();
  output_file.close();

  ROS_INFO("%s", mag_results_stream.str().c_str());
  ROS_INFO("SBG DRIVER [Mag Calib] - Magnetometers calibration results saved to file %s", output_filename.c_str());
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

int SbgDevice::getDeviceRateFrequency(void) const
{
  return m_rate_frequency_;
}

//---------------------------------------------------------------------//
//- Public  methods                                                   -//
//---------------------------------------------------------------------//

void SbgDevice::initDeviceForReceivingData(void)
{
  SbgErrorCode error_code;
  initPublishers();
  configure();

  error_code = sbgEComSetReceiveLogCallback(&m_com_handle_, onLogReceivedCallback, this);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to set the callback function - " + std::string(sbgErrorCodeToString(error_code)));
  }
}

void SbgDevice::initDeviceForMagCalibration(void)
{
  m_calib_service_      = m_p_node_->advertiseService("mag_calibration", &SbgDevice::processMagCalibration, this);
  m_calib_save_service_ = m_p_node_->advertiseService("mag_calibration_save", &SbgDevice::saveMagCalibration, this);

  ROS_INFO("SBG DRIVER [Init] - SBG device is initialized for magnetometers calibration.");
}

void SbgDevice::periodicHandle(void)
{
  m_ros_processing_time_ = ros::Time::now();

  sbgEComHandle(&m_com_handle_);
}