// File header
#include "config_store.h"

using sbg::ConfigStore;

/*!
 * Class to handle the device configuration.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

ConfigStore::ConfigStore():
serial_communication_(false),
upd_communication_(false),
configure_through_ros_(false),
ros_standard_output_(false),
rtcm_subscribe_(false),
nmea_publish_(false)
{

}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

void ConfigStore::loadDriverParameters(const ros::NodeHandle& ref_node_handle)
{
  rate_frequency_ = getParameter<uint32_t>(ref_node_handle, "driver/frequency", 400);
}

void ConfigStore::loadOdomParameters(const ros::NodeHandle& ref_node_handle)
{
  ref_node_handle.param<bool>       ("odometry/enable"   , odom_enable_          , false);
  ref_node_handle.param<bool>       ("odometry/publishTf", odom_publish_tf_      , false);
  ref_node_handle.param<std::string>("odometry/odomFrameId", odom_frame_id_      , "odom");
  ref_node_handle.param<std::string>("odometry/baseFrameId", odom_base_frame_id_ , "base_link");
  ref_node_handle.param<std::string>("odometry/initFrameId", odom_init_frame_id_ , "map");
}

void ConfigStore::loadCommunicationParameters(const ros::NodeHandle& ref_node_handle)
{
  ref_node_handle.param<bool>("confWithRos", configure_through_ros_, false);

  if (ref_node_handle.hasParam("uartConf"))
  {
    serial_communication_ = true;
    ref_node_handle.param<std::string>("uartConf/portName", uart_port_name_, "/dev/ttyUSB0");

    uart_baud_rate_ = getParameter<uint32_t>(ref_node_handle, "uartConf/baudRate", 0);
    output_port_    = getParameter<SbgEComOutputPort>(ref_node_handle, "uartConf/portID", SBG_ECOM_OUTPUT_PORT_A);
  }
  else if (ref_node_handle.hasParam("ipConf"))
  {
    std::string ip_address;
    ref_node_handle.param<std::string>("ipConf/ipAddress", ip_address, "0.0.0.0");

    upd_communication_  = true;
    sbg_ip_address_     = sbgNetworkIpFromString(ip_address.c_str());
    out_port_address_   = getParameter<uint32_t>(ref_node_handle, "ipConf/out_port", 0);
    in_port_address_    = getParameter<uint32_t>(ref_node_handle, "ipConf/in_port", 0);
  }
  else
  {
    throw ros::Exception("SBG DRIVER - Invalid communication interface parameters.");
  }
}

void ConfigStore::loadSensorParameters(const ros::NodeHandle& ref_node_handle)
{
  ref_node_handle.param<double>("sensorParameters/initLat", init_condition_conf_.latitude, 48.419727);
  ref_node_handle.param<double>("sensorParameters/initLong", init_condition_conf_.longitude, -4.472119);
  ref_node_handle.param<double>("sensorParameters/initAlt", init_condition_conf_.altitude, 100);

  init_condition_conf_.year     = getParameter<uint16_t>(ref_node_handle, "sensorParameters/year", 2018);
  init_condition_conf_.month    = getParameter<uint8_t>(ref_node_handle, "sensorParameters/year", 03);
  init_condition_conf_.day      = getParameter<uint8_t>(ref_node_handle, "sensorParameters/year", 10);

  motion_profile_model_info_.id = getParameter<uint32>(ref_node_handle, "sensorParameters/motionProfile", SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE);
}

void ConfigStore::loadImuAlignementParameters(const ros::NodeHandle& ref_node_handle)
{
  sensor_alignement_info_.axisDirectionX  = getParameter<SbgEComAxisDirection>(ref_node_handle, "imuAlignementLeverArm/axisDirectionX", SBG_ECOM_ALIGNMENT_FORWARD);
  sensor_alignement_info_.axisDirectionY  = getParameter<SbgEComAxisDirection>(ref_node_handle, "imuAlignementLeverArm/axisDirectionY", SBG_ECOM_ALIGNMENT_FORWARD);

  ref_node_handle.param<float>("imuAlignementLeverArm/misRoll", sensor_alignement_info_.misRoll, 0.0f);
  ref_node_handle.param<float>("imuAlignementLeverArm/misPitch", sensor_alignement_info_.misPitch, 0.0f);
  ref_node_handle.param<float>("imuAlignementLeverArm/misYaw", sensor_alignement_info_.misYaw, 0.0f);

  float sensor_level_arm[3];
  ref_node_handle.param<float>("imuAlignementLeverArm/leverArmX", sensor_level_arm[0], 0.0f);
  ref_node_handle.param<float>("imuAlignementLeverArm/leverArmY", sensor_level_arm[1], 0.0f);
  ref_node_handle.param<float>("imuAlignementLeverArm/leverArmZ", sensor_level_arm[2], 0.0f);

  sensor_lever_arm_ = SbgVector3<float>(sensor_level_arm, 3);
}

void ConfigStore::loadAidingAssignementParameters(const ros::NodeHandle& ref_node_handle)
{
  aiding_assignement_conf_.gps1Port         = getParameter<SbgEComModulePortAssignment>(ref_node_handle, "aidingAssignment/gnss1ModulePortAssignment", SBG_ECOM_MODULE_PORT_B);
  aiding_assignement_conf_.gps1Sync         = getParameter<SbgEComModuleSyncAssignment>(ref_node_handle, "aidingAssignment/gnss1ModuleSyncAssignment", SBG_ECOM_MODULE_SYNC_DISABLED);
  aiding_assignement_conf_.rtcmPort         = getParameter<SbgEComModulePortAssignment>(ref_node_handle, "aidingAssignment/rtcmPortAssignment", SBG_ECOM_MODULE_DISABLED);
  aiding_assignement_conf_.odometerPinsConf = getParameter<SbgEComOdometerPinAssignment>(ref_node_handle, "aidingAssignment/odometerPinAssignment", SBG_ECOM_MODULE_ODO_DISABLED);
}

void ConfigStore::loadMagnetometersParameters(const ros::NodeHandle& ref_node_handle)
{
  mag_model_info_.id                = getParameter<uint32_t>(ref_node_handle, "magnetometer/magnetometerModel", SBG_ECOM_MAG_MODEL_NORMAL);
  mag_rejection_conf_.magneticField = getParameter<SbgEComRejectionMode>(ref_node_handle, "magnetometer/magnetometerRejectMode", SBG_ECOM_AUTOMATIC_MODE);

  mag_calib_mode_       = getParameter<SbgEComMagCalibMode>(ref_node_handle, "magnetometer/calibration/mode", SBG_ECOM_MAG_CALIB_MODE_2D);
  mag_calib_bandwidth_  = getParameter<SbgEComMagCalibBandwidth>(ref_node_handle, "magnetometer/calibration/bandwidth", SBG_ECOM_MAG_CALIB_HIGH_BW);
}

void ConfigStore::loadGnssParameters(const ros::NodeHandle& ref_node_handle)
{
  gnss_model_info_.id = getParameter<uint32_t>(ref_node_handle, "gnss/gnss_model_id", SBG_ECOM_GNSS_MODEL_NMEA);

  ref_node_handle.param<float>("gnss/primaryLeverArmX", gnss_installation_.leverArmPrimary[0], 0.0f);
  ref_node_handle.param<float>("gnss/primaryLeverArmY", gnss_installation_.leverArmPrimary[1], 0.0f);
  ref_node_handle.param<float>("gnss/primaryLeverArmZ", gnss_installation_.leverArmPrimary[2], 0.0f);
  ref_node_handle.param<bool>("gnss/primaryLeverPrecise", gnss_installation_.leverArmPrimaryPrecise, true);
  ref_node_handle.param<float>("gnss/secondaryLeverArmX", gnss_installation_.leverArmSecondary[0], 0.0f);
  ref_node_handle.param<float>("gnss/secondaryLeverArmY", gnss_installation_.leverArmSecondary[1], 0.0f);
  ref_node_handle.param<float>("gnss/secondaryLeverArmZ", gnss_installation_.leverArmSecondary[2], 0.0f);
  gnss_installation_.leverArmSecondaryMode = getParameter<SbgEComGnssInstallationMode>(ref_node_handle, "gnss/secondaryLeverMode", SBG_ECOM_GNSS_INSTALLATION_MODE_SINGLE);

  gnss_rejection_conf_.position = getParameter<SbgEComRejectionMode>(ref_node_handle, "gnss/posRejectMode", SBG_ECOM_AUTOMATIC_MODE);
  gnss_rejection_conf_.velocity = getParameter<SbgEComRejectionMode>(ref_node_handle, "gnss/velRejectMode", SBG_ECOM_AUTOMATIC_MODE);
  gnss_rejection_conf_.hdt      = getParameter<SbgEComRejectionMode>(ref_node_handle, "gnss/hdtRejectMode", SBG_ECOM_AUTOMATIC_MODE);
}

void ConfigStore::loadOdometerParameters(const ros::NodeHandle& ref_node_handle)
{
  ref_node_handle.param<float>("odom/gain", odometer_conf_.gain, 4800.0f);
  ref_node_handle.param<bool>("odom/direction", odometer_conf_.reverseMode, false);

  float array_odometer_level_arm[3];
  ref_node_handle.param<float>("odom/leverArmX", array_odometer_level_arm[0], 0.0f);
  ref_node_handle.param<float>("odom/leverArmY", array_odometer_level_arm[1], 0.0f);
  ref_node_handle.param<float>("odom/leverArmZ", array_odometer_level_arm[2], 0.0f);

  odometer_level_arm_ = SbgVector3<float>(array_odometer_level_arm, 3);

  odometer_conf_.gainError          = getParameter<uint8_t>(ref_node_handle, "odom/gain_error", 0.1);
  odometer_rejection_conf_.velocity = getParameter<SbgEComRejectionMode>(ref_node_handle, "odom/rejectMode", SBG_ECOM_AUTOMATIC_MODE);
}

void ConfigStore::loadOutputConfiguration(const ros::NodeHandle& ref_node_handle, const std::string& ref_key, SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id)
{
  SbgLogOutput log_output;

  log_output.message_class  = sbg_msg_class;
  log_output.message_id     = sbg_msg_id;
  log_output.output_mode    = getParameter<SbgEComOutputMode>(ref_node_handle, ref_key, SBG_ECOM_OUTPUT_MODE_DISABLED);

  output_modes_.push_back(log_output);
}

void ConfigStore::loadOutputFrameParameters(const ros::NodeHandle& ref_node_handle)
{
  ref_node_handle.param<bool>("output/use_enu", use_enu_, false);

  if (use_enu_)
  {
    ref_node_handle.param<std::string>("output/frame_id", frame_id_, "imu_link");
  }
  else
  {
    ref_node_handle.param<std::string>("output/frame_id", frame_id_, "imu_link_ned");
  }
}

void ConfigStore::loadOutputTimeReference(const ros::NodeHandle& ref_node_handle, const std::string& ref_key)
{
  std::string time_reference;

  ref_node_handle.param<std::string>(ref_key, time_reference, "ros");

  if (time_reference == "ros")
  {
    time_reference_ = TimeReference::ROS;
  }
  else if (time_reference == "ins_unix")
  {
    time_reference_ = TimeReference::INS_UNIX;
  }
  else
  {
    throw std::invalid_argument("unknown time reference: " + time_reference);
  }
}

void ConfigStore::loadRtcmParameters(const ros::NodeHandle& ref_node_handle)
{
  std::string     topic_name;
  std::string     rtcm_namespace;

  ref_node_handle.param<bool>("rtcm/subscribe",          rtcm_subscribe_,        false);
  ref_node_handle.param<std::string>("rtcm/topic_name",  topic_name,             "rtcm");
  ref_node_handle.param<std::string>("rtcm/namespace",   rtcm_namespace,         "ntrip_client");

  rtcm_full_topic_ = rtcm_namespace + "/" + topic_name;
}

void ConfigStore::loadNmeaParameters(const ros::NodeHandle& ref_node_handle)
{
  std::string     topic_name;
  std::string     nmea_namespace;

  ref_node_handle.param<bool>("nmea/publish",            nmea_publish_,          false);
  ref_node_handle.param<std::string>("nmea/topic_name",  topic_name,             "nmea");
  ref_node_handle.param<std::string>("nmea/namespace",   nmea_namespace,         "ntrip_client");

  nmea_full_topic_ = nmea_namespace + "/" + topic_name;
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

bool ConfigStore::checkConfigWithRos() const
{
  return configure_through_ros_;
}

bool ConfigStore::isInterfaceSerial() const
{
  return serial_communication_;
}

const std::string &ConfigStore::getUartPortName() const
{
  return uart_port_name_;
}

uint32_t ConfigStore::getBaudRate() const
{
  return uart_baud_rate_;
}

SbgEComOutputPort ConfigStore::getOutputPort() const
{
  return output_port_;
}

bool ConfigStore::isInterfaceUdp() const
{
  return upd_communication_;
}

sbgIpAddress ConfigStore::getIpAddress() const
{
  return sbg_ip_address_;
}

uint32_t ConfigStore::getOutputPortAddress() const
{
  return out_port_address_;
}

uint32_t ConfigStore::getInputPortAddress() const
{
  return in_port_address_;
}

const SbgEComInitConditionConf &ConfigStore::getInitialConditions() const
{
  return init_condition_conf_;
}

const SbgEComModelInfo &ConfigStore::getMotionProfile() const
{
  return motion_profile_model_info_;
}

const SbgEComSensorAlignmentInfo &ConfigStore::getSensorAlignement() const
{
  return sensor_alignement_info_;
}

const sbg::SbgVector3<float> &ConfigStore::getSensorLevelArms() const
{
  return sensor_lever_arm_;
}

const SbgEComAidingAssignConf &ConfigStore::getAidingAssignement() const
{
  return aiding_assignement_conf_;
}

const SbgEComModelInfo &ConfigStore::getMagnetometerModel() const
{
  return mag_model_info_;
}

const SbgEComMagRejectionConf &ConfigStore::getMagnetometerRejection() const
{
  return mag_rejection_conf_;
}

const SbgEComMagCalibMode &ConfigStore::getMagnetometerCalibMode() const
{
  return mag_calib_mode_;
}

const SbgEComMagCalibBandwidth &ConfigStore::getMagnetometerCalibBandwidth() const
{
  return mag_calib_bandwidth_;
}

const SbgEComModelInfo &ConfigStore::getGnssModel() const
{
  return gnss_model_info_;
}

const SbgEComGnssInstallation &ConfigStore::getGnssInstallation() const
{
  return gnss_installation_;
}

const SbgEComGnssRejectionConf &ConfigStore::getGnssRejection() const
{
  return gnss_rejection_conf_;
}

const SbgEComOdoConf &ConfigStore::getOdometerConf() const
{
  return odometer_conf_;
}

const sbg::SbgVector3<float> &ConfigStore::getOdometerLevelArms() const
{
  return odometer_level_arm_;
}

const SbgEComOdoRejectionConf &ConfigStore::getOdometerRejection() const
{
  return odometer_rejection_conf_;
}

const std::vector<ConfigStore::SbgLogOutput> &ConfigStore::getOutputModes() const
{
  return output_modes_;
}

bool ConfigStore::checkRosStandardMessages() const
{
  return ros_standard_output_;
}

uint32_t ConfigStore::getReadingRateFrequency() const
{
  return rate_frequency_;
}

const std::string &ConfigStore::getFrameId() const
{
  return frame_id_;
}

bool ConfigStore::getUseEnu() const
{
  return use_enu_;
}

sbg::TimeReference ConfigStore::getTimeReference() const
{
  return time_reference_;
}

bool ConfigStore::getOdomEnable() const
{
  return odom_enable_;
}

bool ConfigStore::getOdomPublishTf() const
{
  return odom_publish_tf_;
}

const std::string &ConfigStore::getOdomFrameId() const
{
  return odom_frame_id_;
}

const std::string &ConfigStore::getOdomBaseFrameId() const
{
  return odom_base_frame_id_;
}

const std::string &ConfigStore::getOdomInitFrameId() const
{
  return odom_init_frame_id_;
}

bool ConfigStore::shouldSubscribeToRtcm() const
{
  return rtcm_subscribe_;
}

const std::string &ConfigStore::getRtcmFullTopic() const
{
  return rtcm_full_topic_;
}

bool ConfigStore::shouldPublishNmea() const
{
  return nmea_publish_;
}

const std::string &ConfigStore::getNmeaFullTopic() const
{
  return nmea_full_topic_;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void ConfigStore::loadFromRosNodeHandle(const ros::NodeHandle& ref_node_handle)
{
  loadDriverParameters(ref_node_handle);
  loadOdomParameters(ref_node_handle);
  loadCommunicationParameters(ref_node_handle);
  loadSensorParameters(ref_node_handle);
  loadImuAlignementParameters(ref_node_handle);
  loadAidingAssignementParameters(ref_node_handle);
  loadMagnetometersParameters(ref_node_handle);
  loadGnssParameters(ref_node_handle);
  loadOdometerParameters(ref_node_handle);
  loadOutputFrameParameters(ref_node_handle);
  loadRtcmParameters(ref_node_handle);
  loadNmeaParameters(ref_node_handle);

  loadOutputTimeReference(ref_node_handle, "output/time_reference");

  loadOutputConfiguration(ref_node_handle, "output/log_status", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS);
  loadOutputConfiguration(ref_node_handle, "output/log_imu_data", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA);
  loadOutputConfiguration(ref_node_handle, "output/log_ekf_euler", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER);
  loadOutputConfiguration(ref_node_handle, "output/log_ekf_quat", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT);
  loadOutputConfiguration(ref_node_handle, "output/log_ekf_nav", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV);
  loadOutputConfiguration(ref_node_handle, "output/log_ship_motion", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION);
  loadOutputConfiguration(ref_node_handle, "output/log_utc_time", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME);
  loadOutputConfiguration(ref_node_handle, "output/log_mag", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG);
  loadOutputConfiguration(ref_node_handle, "output/log_mag_calib", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB);
  loadOutputConfiguration(ref_node_handle, "output/log_gps1_vel", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL);
  loadOutputConfiguration(ref_node_handle, "output/log_gps1_pos", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS);
  loadOutputConfiguration(ref_node_handle, "output/log_gps1_hdt", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT);
  loadOutputConfiguration(ref_node_handle, "output/log_gps1_raw", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW);
  loadOutputConfiguration(ref_node_handle, "output/log_odo_vel", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL);
  loadOutputConfiguration(ref_node_handle, "output/log_event_a", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A);
  loadOutputConfiguration(ref_node_handle, "output/log_event_b", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B);
  loadOutputConfiguration(ref_node_handle, "output/log_event_c", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C);
  loadOutputConfiguration(ref_node_handle, "output/log_event_d", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D);
  loadOutputConfiguration(ref_node_handle, "output/log_event_e", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_E);
  loadOutputConfiguration(ref_node_handle, "output/log_air_data", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA);
  loadOutputConfiguration(ref_node_handle, "output/log_imu_short", SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_SHORT);

  ref_node_handle.param<bool>("output/ros_standard", ros_standard_output_, false);
}
