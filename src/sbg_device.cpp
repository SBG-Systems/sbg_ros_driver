// File header
#include "sbg_device.h"

// Standard headers
#include <iomanip>
#include <fstream>
#include <ctime>

// Boost headers
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

// SbgECom headers
#include <version/sbgVersion.h>

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

//
// Static magnetometers maps definition.
//
std::map<SbgEComMagCalibQuality, std::string> SbgDevice::g_mag_calib_quality_ = { {SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL, "Quality: optimal"},
                                                                                  {SBG_ECOM_MAG_CALIB_QUAL_GOOD, "Quality: good"},
                                                                                  {SBG_ECOM_MAG_CALIB_QUAL_POOR, "Quality: poor"},
                                                                                  {SBG_ECOM_MAG_CALIB_QUAL_INVALID, "Quality: invalid"}};

std::map<SbgEComMagCalibConfidence, std::string> SbgDevice::g_mag_calib_confidence_ = { {SBG_ECOM_MAG_CALIB_TRUST_HIGH, "Confidence: high"},
                                                                                        {SBG_ECOM_MAG_CALIB_TRUST_MEDIUM, "Confidence: medium"},
                                                                                        {SBG_ECOM_MAG_CALIB_TRUST_LOW, "Confidence: low"}};

std::map<SbgEComMagCalibMode, std::string> SbgDevice::g_mag_calib_mode_ = { {SBG_ECOM_MAG_CALIB_MODE_2D, "Mode 2D"},
                                                                            {SBG_ECOM_MAG_CALIB_MODE_3D, "Mode 3D"}};

std::map<SbgEComMagCalibBandwidth, std::string> SbgDevice::g_mag_calib_bandwidth_ = {{SBG_ECOM_MAG_CALIB_HIGH_BW,   "High Bandwidth"},
                                                                                     {SBG_ECOM_MAG_CALIB_MEDIUM_BW, "Medium Bandwidth"},
                                                                                     {SBG_ECOM_MAG_CALIB_LOW_BW,    "Low Bandwidth"}};

/*!
 * Class to handle a connected SBG device.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

SbgDevice::SbgDevice(ros::NodeHandle& ref_node_handle):
ref_node_(ref_node_handle),
mag_calibration_ongoing_(false),
mag_calibration_done_(false)
{
  loadParameters();
  connect();
}

SbgDevice::~SbgDevice()
{
  SbgErrorCode error_code;

  error_code = sbgEComClose(&com_handle_);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("Unable to close the SBG communication handle - %s.", sbgErrorCodeToString(error_code));
  }

  if (config_store_.isInterfaceSerial())
  {
    error_code = sbgInterfaceSerialDestroy(&sbg_interface_);
  }
  else if (config_store_.isInterfaceUdp())
  {
    error_code = sbgInterfaceUdpDestroy(&sbg_interface_);
  }

  if (error_code != SBG_NO_ERROR)
  {
    ROS_ERROR("SBG DRIVER - Unable to close the communication interface.");
  }
}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

SbgErrorCode SbgDevice::onLogReceivedCallback(SbgEComHandle* p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData* p_log_data, void* p_user_arg)
{
  assert(p_user_arg);

  SBG_UNUSED_PARAMETER(p_handle);

  SbgDevice *p_sbg_device;
  p_sbg_device = (SbgDevice*)(p_user_arg);

  p_sbg_device->onLogReceived(msg_class, msg, *p_log_data);

  return SBG_NO_ERROR;
}

void SbgDevice::onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData& ref_sbg_data)
{
  //
  // Publish the received SBG log.
  //
  message_publisher_.publish(msg_class, msg, ref_sbg_data);
}

void SbgDevice::loadParameters()
{
  //
  // Get the ROS private nodeHandle, where the parameters are loaded from the launch file.
  //
  ros::NodeHandle n_private("~");
  config_store_.loadFromRosNodeHandle(n_private);
}

void SbgDevice::connect()
{
  SbgErrorCode error_code;
  error_code = SBG_NO_ERROR;

  //
  // Initialize the communication interface from the config store, then initialize the sbgECom protocol to communicate with the device.
  //
  if (config_store_.isInterfaceSerial())
  {
    ROS_INFO("SBG_DRIVER - serial interface %s at %d bps", config_store_.getUartPortName().c_str(), config_store_.getBaudRate());
    error_code = sbgInterfaceSerialCreate(&sbg_interface_, config_store_.getUartPortName().c_str(), config_store_.getBaudRate());
  }
  else if (config_store_.isInterfaceUdp())
  {
    char ip[16];
    sbgNetworkIpToString(config_store_.getIpAddress(), ip, sizeof(ip));
    ROS_INFO("SBG_DRIVER - UDP interface %s %d->%d", ip, config_store_.getInputPortAddress(), config_store_.getOutputPortAddress());
    error_code = sbgInterfaceUdpCreate(&sbg_interface_, config_store_.getIpAddress(), config_store_.getInputPortAddress(), config_store_.getOutputPortAddress());
  }
  else
  {
    throw ros::Exception("Invalid interface type for the SBG device.");
  }

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to initialize the interface - " + std::string(sbgErrorCodeToString(error_code)));
  }

  error_code = sbgEComInit(&com_handle_, &sbg_interface_);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to initialize the SbgECom protocol - " + std::string(sbgErrorCodeToString(error_code)));
  }

  readDeviceInfo();
}

void SbgDevice::readDeviceInfo(void)
{
  SbgEComDeviceInfo device_info;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdGetInfo(&com_handle_, &device_info);

  if (error_code == SBG_NO_ERROR)
  {
    ROS_INFO("SBG_DRIVER - productCode = %s", device_info.productCode);
    ROS_INFO("SBG_DRIVER - serialNumber = %u", device_info.serialNumber);

    ROS_INFO("SBG_DRIVER - calibationRev = %s", getVersionAsString(device_info.calibationRev).c_str());
    ROS_INFO("SBG_DRIVER - calibrationDate = %u / %u / %u", device_info.calibrationDay, device_info.calibrationMonth, device_info.calibrationYear);

    ROS_INFO("SBG_DRIVER - hardwareRev = %s", getVersionAsString(device_info.hardwareRev).c_str());
    ROS_INFO("SBG_DRIVER - firmwareRev = %s", getVersionAsString(device_info.firmwareRev).c_str());
  }
  else
  {
    ROS_ERROR("Unable to get the device Info : %s", sbgErrorCodeToString(error_code));
  }
}

std::string SbgDevice::getVersionAsString(uint32 sbg_version_enc) const
{
  char version[32];
  sbgVersionToStringEncoded(sbg_version_enc, version, 32);

  return std::string(version);
}

void SbgDevice::initPublishers()
{
  message_publisher_.initPublishers(ref_node_, config_store_);

  rate_frequency_ = config_store_.getReadingRateFrequency();
}

void SbgDevice::initSubscribers()
{
  if (config_store_.shouldSubscribeToRtcm())
  {
    auto rtcm_cb = [&](const rtcm_msgs::Message::ConstPtr& msg) -> void {
        this->writeRtcmMessageToDevice(msg);
    };

    rtcm_sub_ = ref_node_.subscribe<rtcm_msgs::Message>(config_store_.getRtcmFullTopic(), 10, rtcm_cb);
  }
}

void SbgDevice::configure()
{
  if (config_store_.checkConfigWithRos())
  {
    ConfigApplier configApplier(com_handle_);
    configApplier.applyConfiguration(config_store_);
  }
}

bool SbgDevice::processMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response)
{
  SBG_UNUSED_PARAMETER(ref_ros_request);

  if (mag_calibration_ongoing_)
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

    mag_calibration_ongoing_  = false;
    mag_calibration_done_     = true;
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

    mag_calibration_ongoing_ = true;
  }

  return ref_ros_response.success;
}

bool SbgDevice::saveMagCalibration(std_srvs::Trigger::Request& ref_ros_request, std_srvs::Trigger::Response& ref_ros_response)
{
  SBG_UNUSED_PARAMETER(ref_ros_request);

  if (mag_calibration_ongoing_)
  {
    ref_ros_response.success = false;
    ref_ros_response.message = "Magnetometer calibration process is still ongoing, finish it before trying to save it.";
  }
  else if (mag_calibration_done_)
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

bool SbgDevice::startMagCalibration()
{
  SbgErrorCode              error_code;
  SbgEComMagCalibMode       mag_calib_mode;
  SbgEComMagCalibBandwidth  mag_calib_bandwidth;

  mag_calib_mode      = config_store_.getMagnetometerCalibMode();
  mag_calib_bandwidth = config_store_.getMagnetometerCalibBandwidth();

  error_code = sbgEComCmdMagStartCalib(&com_handle_, mag_calib_mode, mag_calib_bandwidth);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Unable to start the magnetometer calibration : %s", sbgErrorCodeToString(error_code));
    return false;
  }
  else
  {
    ROS_INFO("SBG DRIVER [Mag Calib] - Start calibration");
    ROS_INFO("SBG DRIVER [Mag Calib] - Mode : %s", g_mag_calib_mode_[mag_calib_mode].c_str());
    ROS_INFO("SBG DRIVER [Mag Calib] - Bandwidth : %s", g_mag_calib_bandwidth_[mag_calib_bandwidth].c_str());
    return true;
  }
}

bool SbgDevice::endMagCalibration()
{
  SbgErrorCode error_code;

  error_code = sbgEComCmdMagComputeCalib(&com_handle_, &mag_calib_results_);

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

bool SbgDevice::uploadMagCalibrationToDevice()
{
  SbgErrorCode error_code;

  if (mag_calib_results_.quality != SBG_ECOM_MAG_CALIB_QUAL_INVALID)
  {
    error_code = sbgEComCmdMagSetCalibData(&com_handle_, mag_calib_results_.offset, mag_calib_results_.matrix);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Unable to set the magnetometers calibration data to the device : %s", sbgErrorCodeToString(error_code));
      return false;
    }
    else
    {
      ROS_INFO("SBG DRIVER [Mag Calib] - Saving data to the device");
      ConfigApplier configApplier(com_handle_);
      configApplier.saveConfiguration();
      return true;
    }
  }
  else
  {
    ROS_ERROR("SBG DRIVER [Mag Calib] - The calibration was invalid, it can't be uploaded on the device.");
    return false;
  }
}

void SbgDevice::displayMagCalibrationStatusResult() const
{
  ROS_INFO("SBG DRIVER [Mag Calib] - Quality of the calibration %s", g_mag_calib_quality_[mag_calib_results_.quality].c_str());
  ROS_INFO("SBG DRIVER [Mag Calib] - Calibration results confidence %s", g_mag_calib_confidence_[mag_calib_results_.confidence].c_str());

  SbgEComMagCalibMode mag_calib_mode;

  mag_calib_mode = config_store_.getMagnetometerCalibMode();

  //
  // Check the magnetometers calibration status and display the warnings.
  //
  if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Not enough valid points. Maybe you are moving too fast");
  }
  if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - Unable to find a calibration solution. Maybe there are too much non static distortions");
  }
  if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE)
  {
    ROS_WARN("SBG DRIVER [Mag Calib] - The magnetic calibration has troubles to correct the magnetometers and inertial frame alignment");
  }
  if (mag_calib_mode == SBG_ECOM_MAG_CALIB_MODE_2D)
  {
    if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Too much roll motion for a 2D magnetic calibration");
    }
    if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Too much pitch motion for a 2D magnetic calibration");
    }
  }
  else
  {
    if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_X_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Not enough roll motion for a 3D magnetic calibration");
    }
    if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_Y_MOTION_ISSUE)
    {
      ROS_WARN("SBG DRIVER [Mag Calib] - Not enough pitch motion for a 3D magnetic calibration.");
    }
  }
  if (mag_calib_results_.advancedStatus & SBG_ECOM_MAG_CALIB_Z_MOTION_ISSUE)
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

  mag_calib_mode      = config_store_.getMagnetometerCalibMode();
  mag_calib_bandwidth = config_store_.getMagnetometerCalibBandwidth();

  mag_results_stream << "SBG DRIVER [Mag Calib]" << endl;
  mag_results_stream << "======= Parameters =======" << endl;
  mag_results_stream << "* CALIB_MODE = " << g_mag_calib_mode_[mag_calib_mode] << endl;
  mag_results_stream << "* CALIB_BW = " << g_mag_calib_bandwidth_[mag_calib_bandwidth] << endl;

  mag_results_stream << "======= Results =======" << endl;
  mag_results_stream << g_mag_calib_quality_[mag_calib_results_.quality] << endl;
  mag_results_stream << g_mag_calib_confidence_[mag_calib_results_.confidence] << endl;
  mag_results_stream << "======= Infos =======" << endl;
  mag_results_stream << "* Used points : " << mag_calib_results_.numPoints << "/" << mag_calib_results_.maxNumPoints << endl;
  mag_results_stream << "* Mean, Std, Max" << endl;
  mag_results_stream << "[Before]\t" << mag_calib_results_.beforeMeanError << "\t" << mag_calib_results_.beforeStdError << "\t" << mag_calib_results_.beforeMaxError << endl;
  mag_results_stream << "[After]\t" << mag_calib_results_.afterMeanError << "\t" << mag_calib_results_.afterStdError << "\t" << mag_calib_results_.afterMaxError << endl;
  mag_results_stream << "[Accuracy]\t" << sbgRadToDegF(mag_calib_results_.meanAccuracy) << "\t" << sbgRadToDegF(mag_calib_results_.stdAccuracy) << "\t" << sbgRadToDegF(mag_calib_results_.maxAccuracy) << endl;
  mag_results_stream << "* Offset\t" << mag_calib_results_.offset[0] << "\t" << mag_calib_results_.offset[1] << "\t" << mag_calib_results_.offset[2] << endl;

  mag_results_stream << "* Matrix" << endl;
  mag_results_stream << mag_calib_results_.matrix[0] << "\t" << mag_calib_results_.matrix[1] << "\t" << mag_calib_results_.matrix[2] << endl;
  mag_results_stream << mag_calib_results_.matrix[3] << "\t" << mag_calib_results_.matrix[4] << "\t" << mag_calib_results_.matrix[5] << endl;
  mag_results_stream << mag_calib_results_.matrix[6] << "\t" << mag_calib_results_.matrix[7] << "\t" << mag_calib_results_.matrix[8] << endl;

  output_filename = "mag_calib_" + timeToStr(ros::WallTime::now()) + ".txt";
  ofstream output_file(output_filename);
  output_file << mag_results_stream.str();
  output_file.close();

  ROS_INFO("%s", mag_results_stream.str().c_str());
  ROS_INFO("SBG DRIVER [Mag Calib] - Magnetometers calibration results saved to file %s", output_filename.c_str());
}

void SbgDevice::writeRtcmMessageToDevice(const rtcm_msgs::Message::ConstPtr& msg)
{
  auto rtcm_data = msg->message;
  auto error_code = sbgInterfaceWrite(&sbg_interface_, rtcm_data.data(), rtcm_data.size());

  if (error_code != SBG_NO_ERROR)
  {
    char error_str[256];

    sbgEComErrorToString(error_code, error_str);
    SBG_LOG_ERROR(SBG_ERROR, "Failed to sent RTCM data to device: %s", error_str);
  }
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

uint32_t SbgDevice::getUpdateFrequency() const
{
  return rate_frequency_;
}

//---------------------------------------------------------------------//
//- Public  methods                                                   -//
//---------------------------------------------------------------------//

void SbgDevice::initDeviceForReceivingData()
{
  SbgErrorCode error_code;
  initPublishers();
  configure();

  error_code = sbgEComSetReceiveLogCallback(&com_handle_, onLogReceivedCallback, this);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG_DRIVER - [Init] Unable to set the callback function - " + std::string(sbgErrorCodeToString(error_code)));
  }

  initSubscribers();
}

void SbgDevice::initDeviceForMagCalibration()
{
  calib_service_      = ref_node_.advertiseService("sbg/mag_calibration", &SbgDevice::processMagCalibration, this);
  calib_save_service_ = ref_node_.advertiseService("sbg/mag_calibration_save", &SbgDevice::saveMagCalibration, this);

  ROS_INFO("SBG DRIVER [Init] - SBG device is initialized for magnetometers calibration.");
}

void SbgDevice::periodicHandle()
{
  sbgEComHandle(&com_handle_);
}
