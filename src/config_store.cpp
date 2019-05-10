#include "config_store.h"

using sbg::ConfigOutput;
using sbg::ConfigStore;

/*!
 * Class to handle the device configuration.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

ConfigStore::ConfigStore(void):
m_rebootNeeded(false)
{

}

//---------------------------------------------------------------------//
//- Private  methods                                                  -//
//---------------------------------------------------------------------//

void ConfigStore::loadCommunicationParameters(ros::NodeHandle& ref_node_handle)
{
  m_uart_port_name_   = ref_node_handle.param<std::string>("uartConf/portName", "/dev/ttyUSB0");
  m_uart_baud_rate_   = static_cast<uint32>(ref_node_handle.param<int>("uartConf/baudRate", 115200));
  m_output_port_      = static_cast<SbgEComOutputPort>(ref_node_handle.param<int>("uartConf/portID", SBG_ECOM_OUTPUT_PORT_A));
}

void ConfigStore::loadSensorParameters(ros::NodeHandle& ref_node_handle)
{
  m_init_condition_conf_.latitude   = ref_node_handle.param<double>("sensorParameters/initLat", 48.419727);
  m_init_condition_conf_.longitude  = ref_node_handle.param<double>("sensorParameters/initLong", -4.472119);
  m_init_condition_conf_.altitude   = ref_node_handle.param<double>("sensorParameters/initAlt", 100);
  m_init_condition_conf_.year       = ref_node_handle.param<double>("sensorParameters/year", 2018);
  m_init_condition_conf_.month      = ref_node_handle.param<double>("sensorParameters/month", 03);
  m_init_condition_conf_.day        = ref_node_handle.param<double>("sensorParameters/day", 10);

  m_motion_profile_model_info_.id   = static_cast<uint32>(ref_node_handle.param<int>("sensorParameters/motionProfie", SBG_ECOM_MOTION_PROFILE_GENERAL_PURPOSE));
}

void ConfigStore::loadImuAlignementParameters(ros::NodeHandle& ref_node_handle)
{
  m_sensor_alignement_info_.axisDirectionX  = static_cast<SbgEComAxisDirection>(ref_node_handle.param<int>("imuAlignementLeverArm/axisDirectionX", SBG_ECOM_ALIGNMENT_FORWARD));
  m_sensor_alignement_info_.axisDirectionY  = static_cast<SbgEComAxisDirection>(ref_node_handle.param<int>("imuAlignementLeverArm/axisDirectionY", SBG_ECOM_ALIGNMENT_FORWARD));
  m_sensor_alignement_info_.misRoll         = ref_node_handle.param<float>("imuAlignementLeverArm/misRoll",0);
  m_sensor_alignement_info_.misPitch        = ref_node_handle.param<float>("imuAlignementLeverArm/misPitch",0);
  m_sensor_alignement_info_.misYaw          = ref_node_handle.param<float>("imuAlignementLeverArm/misYaw",0);

  m_sensor_lever_arm_[0] = ref_node_handle.param<float>("imuAlignementLeverArm/leverArmX", 0);
  m_sensor_lever_arm_[1] = ref_node_handle.param<float>("imuAlignementLeverArm/leverArmY", 0);
  m_sensor_lever_arm_[2] = ref_node_handle.param<float>("imuAlignementLeverArm/leverArmZ", 0);
}

void ConfigStore::loadAidingAssignementParameters(ros::NodeHandle& ref_node_handle)
{
  m_aiding_assignement_conf_.gps1Port = static_cast<SbgEComModulePortAssignment>(ref_node_handle.param<int>("aidingAssignment/gnss1ModulePortAssignment", SBG_ECOM_MODULE_PORT_B));
  m_aiding_assignement_conf_.gps1Sync = static_cast<SbgEComModuleSyncAssignment>(ref_node_handle.param<int>("aidingAssignment/gnss1ModuleSyncAssignment", SBG_ECOM_MODULE_SYNC_DISABLED));
  m_aiding_assignement_conf_.rtcmPort = static_cast<SbgEComModulePortAssignment>(ref_node_handle.param<int>("aidingAssignment/rtcmPortAssignment", SBG_ECOM_MODULE_DISABLED));
  m_aiding_assignement_conf_.odometerPinsConf = static_cast<SbgEComOdometerPinAssignment>(ref_node_handle.param<int>("aidingAssignment/odometerPinAssignment", SBG_ECOM_MODULE_ODO_DISABLED));
}

void ConfigStore::loadMagnetometersParameters(ros::NodeHandle& ref_node_handle)
{
  m_mag_model_info_.id                = static_cast<uint32>(ref_node_handle.param<int>("magnetometer/magnetometerModel", SBG_ECOM_MAG_MODEL_NORMAL));
  m_mag_rejection_conf_.magneticField = static_cast<SbgEComRejectionMode>(ref_node_handle.param<int>("magnetometer/magnetometerRejectMode", SBG_ECOM_AUTOMATIC_MODE));

  m_mag_calib_mode_       = static_cast<SbgEComMagCalibMode>(ref_node_handle.param<int>("magnetometer/calibration/mode", SBG_ECOM_MAG_CALIB_MODE_2D));
  m_mag_calib_bandwidth_  = static_cast<SbgEComMagCalibBandwidth>(ref_node_handle.param<int>("magnetometer/calibration/bandwidth", SBG_ECOM_MAG_CALIB_HIGH_BW));
}

void ConfigStore::loadGnssParameters(ros::NodeHandle& ref_node_handle)
{
  m_gnss_model_info_.id = static_cast<uint32>(ref_node_handle.param<int>("gnss/gnss_model_id", SBG_ECOM_GNSS_MODEL_NMEA));

  m_gnss_alignement_info_.leverArmX       = ref_node_handle.param<float>("gnss/leverArmX", 0);
  m_gnss_alignement_info_.leverArmY       = ref_node_handle.param<float>("gnss/leverArmY", 0);
  m_gnss_alignement_info_.leverArmZ       = ref_node_handle.param<float>("gnss/leverArmZ", 0);
  m_gnss_alignement_info_.pitchOffset     = ref_node_handle.param<float>("gnss/pitchOffset", 0);
  m_gnss_alignement_info_.yawOffset       = ref_node_handle.param<float>("gnss/yawOffset", 0);
  m_gnss_alignement_info_.antennaDistance = ref_node_handle.param<float>("gnss/antennaDistance", 0);

  m_gnss_rejection_conf_.position = static_cast<SbgEComRejectionMode>(ref_node_handle.param<int>("gnss/posRejectMode", SBG_ECOM_AUTOMATIC_MODE));
  m_gnss_rejection_conf_.velocity = static_cast<SbgEComRejectionMode>(ref_node_handle.param<int>("gnss/velRejectMode", SBG_ECOM_AUTOMATIC_MODE));
  m_gnss_rejection_conf_.hdt      = static_cast<SbgEComRejectionMode>(ref_node_handle.param<int>("gnss/hdtRejectMode", SBG_ECOM_AUTOMATIC_MODE));
}

void ConfigStore::loadOdometerParameters(ros::NodeHandle& ref_node_handle)
{
  m_odometer_conf_.gain         = ref_node_handle.param<float>("odom/gain", 4800);
  m_odometer_conf_.gainError    = static_cast<uint8>(ref_node_handle.param<int>("odom/gain_error", 0.1));
  m_odometer_conf_.reverseMode  = ref_node_handle.param<bool>("odom/direction", false);

  m_odometer_level_arm_[0] = ref_node_handle.param<float>("odom/leverArmX", 0);
  m_odometer_level_arm_[1] = ref_node_handle.param<float>("odom/leverArmY", 0);
  m_odometer_level_arm_[2] = ref_node_handle.param<float>("odom/leverArmZ", 0);

  m_odometer_rejection_conf_.velocity = static_cast<SbgEComRejectionMode>(ref_node_handle.param<int>("odom/rejectMode", SBG_ECOM_AUTOMATIC_MODE));
}

void ConfigStore::configureInitCondition(SbgEComHandle* p_com_handle)
{
  //
  // Get the initial condition of the device, compare with the loaded parameters.
  // If the conditions are different, update the device configuration with the loaded parameters.
  //
  SbgEComInitConditionConf  init_condition;
  SbgErrorCode              error_code;

  error_code = sbgEComCmdSensorGetInitCondition(p_com_handle, &init_condition);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the device Init conditions %s", sbgErrorCodeToString(error_code));
  }

  if (init_condition.year != m_init_condition_conf_.year
  || init_condition.month != m_init_condition_conf_.month
  || init_condition.day != m_init_condition_conf_.day
  || init_condition.altitude != m_init_condition_conf_.altitude
  || init_condition.latitude != m_init_condition_conf_.latitude
  || init_condition.longitude != m_init_condition_conf_.longitude)
  {
    error_code = sbgEComCmdSensorSetInitCondition(p_com_handle, &m_init_condition_conf_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the device Init conditions %s", sbgErrorCodeToString(error_code)) ;
    }
    else
    {
      ROS_INFO("SBG DRIVER - [Param] Initial conditions updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureMotionProfile(SbgEComHandle* p_com_handle)
{
  //
  // Get the motion profile ID, and compare with the loaded one parameter.
  // If the profiles are different, update the device with the loaded one.
  //
  SbgEComModelInfo  motion_profile;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdSensorGetMotionProfileInfo(p_com_handle, &motion_profile);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the motion profile %s", sbgErrorCodeToString(error_code));
  }

  if (motion_profile.id != m_motion_profile_model_info_.id)
  {
    error_code = sbgEComCmdSensorSetMotionProfileId(p_com_handle, m_motion_profile_model_info_.id);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the motion profile %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG DRIVER - [Param] Motion profile updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureImuAlignement(SbgEComHandle* p_com_handle)
{
  //
  // Get the IMU alignement and level arms, and compare with the parameters.
  // If the alignement are differents, update the device with the loaded parameters.
  //
  SbgErrorCode                error_code;
  SbgEComSensorAlignmentInfo  sensor_alignement;
  float                       leverArm[3];

  error_code = sbgEComCmdSensorGetAlignmentAndLeverArm(p_com_handle, &sensor_alignement, leverArm);
 
  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the IMU alignement %s", sbgErrorCodeToString(error_code));
  }

  if (leverArm[0] != m_sensor_lever_arm_[0]
  || leverArm[1] != m_sensor_lever_arm_[1]
  || leverArm[2] != m_sensor_lever_arm_[2]
  || sensor_alignement.axisDirectionX != m_sensor_alignement_info_.axisDirectionX
  || sensor_alignement.axisDirectionY != m_sensor_alignement_info_.axisDirectionY
  || sensor_alignement.misRoll != m_sensor_alignement_info_.misRoll
  || sensor_alignement.misPitch != m_sensor_alignement_info_.misPitch
  || sensor_alignement.misYaw != m_sensor_alignement_info_.misYaw)
  {
    error_code = sbgEComCmdSensorSetAlignmentAndLeverArm(p_com_handle, &m_sensor_alignement_info_, m_sensor_lever_arm_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the IMU alignement %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Imu alignement updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureAidingAssignement(SbgEComHandle* p_com_handle)
{
  //
  // Get the aiding assignement, and compare with the loaded parameters.
  // If the assignement are differents, udpdate the device with the loaded parameters.
  //
  SbgEComAidingAssignConf aiding_assign;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdSensorGetAidingAssignment(p_com_handle, &aiding_assign);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the aiding assignement %s", sbgErrorCodeToString(error_code));
  }

  if (aiding_assign.gps1Port != m_aiding_assignement_conf_.gps1Port
  || aiding_assign.gps1Sync != m_aiding_assignement_conf_.gps1Sync
  || aiding_assign.odometerPinsConf != m_aiding_assignement_conf_.odometerPinsConf
  || aiding_assign.rtcmPort != m_aiding_assignement_conf_.rtcmPort)
  {
    error_code = sbgEComCmdSensorSetAidingAssignment(p_com_handle, &m_aiding_assignement_conf_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the aiding assignement  %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Aiding assignement updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureMagModel(SbgEComHandle* p_com_handle)
{
  //
  // Get the magnetometer model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgEComModelInfo  model_info;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdMagGetModelInfo(p_com_handle, &model_info);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the magnetometer model %s", sbgErrorCodeToString(error_code));
  }

  if (model_info.id != m_mag_model_info_.id)
  {
    error_code = sbgEComCmdMagSetModelId(p_com_handle, m_mag_model_info_.id);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the magnetometer model %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Magnetometer model updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureMagRejection(SbgEComHandle* p_com_handle)
{
  //
  // Get the magnetometer rejection model, and compare with the loaded parameter.
  // If the model are different, update the device with the loaded parameter.
  //
  SbgEComMagRejectionConf mag_rejection;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdMagGetRejection(p_com_handle, &mag_rejection);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the magnetometer rejection %s", sbgErrorCodeToString(error_code));
  }

  if (mag_rejection.magneticField != m_mag_rejection_conf_.magneticField)
  {
    error_code = sbgEComCmdMagSetRejection(p_com_handle, &m_mag_rejection_conf_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the magnetometer rejection %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Magnetometer rejection updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureGnssModel(SbgEComHandle* p_com_handle)
{
  //
  // Get the Gnss model, and compare with the loaded model.
  // If the models are different, update the device with the loaded model.
  //
  SbgEComModelInfo  model_info;
  SbgErrorCode      error_code;

  error_code = sbgEComCmdGnss1GetModelInfo(p_com_handle, &model_info);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the Gnss model %s", sbgErrorCodeToString(error_code));
  }

  if (model_info.id != m_gnss_model_info_.id)
  {
    error_code = sbgEComCmdGnss1SetModelId(p_com_handle, m_gnss_model_info_.id);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the Gnss model %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Gnss model updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureGnssLevelArm(SbgEComHandle* p_com_handle)
{
  //
  // Get the Gnss level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  SbgEComGnssAlignmentInfo  gnss_alignement;
  SbgErrorCode              error_code;

  error_code = sbgEComCmdGnss1GetLeverArmAlignment(p_com_handle, &gnss_alignement);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the Gnss level Arm %s", sbgErrorCodeToString(error_code));
  }

  if (gnss_alignement.antennaDistance != m_gnss_alignement_info_.antennaDistance
  || gnss_alignement.leverArmX != m_gnss_alignement_info_.leverArmX
  || gnss_alignement.leverArmY != m_gnss_alignement_info_.leverArmY
  || gnss_alignement.leverArmZ != m_gnss_alignement_info_.leverArmZ
  || gnss_alignement.pitchOffset != m_gnss_alignement_info_.pitchOffset
  || gnss_alignement.yawOffset != m_gnss_alignement_info_.yawOffset)
  {
    error_code = sbgEComCmdGnss1SetLeverArmAlignment(p_com_handle, &m_gnss_alignement_info_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the Gnss level Arm %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Gnss level arm updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureGnssRejection(SbgEComHandle* p_com_handle)
{
  //
  // Get the Gnss rejection, and compare with the loaded parameters.
  // If the rejection are different, update the device with the loaded parameters.
  //
  SbgEComGnssRejectionConf  rejection;
  SbgErrorCode              error_code;

  error_code = sbgEComCmdGnss1GetRejection(p_com_handle, &rejection);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the Gnss rejection %s", sbgErrorCodeToString(error_code));
  }

  if (rejection.hdt != m_gnss_rejection_conf_.hdt
  || rejection.position != m_gnss_rejection_conf_.position
  || rejection.velocity != m_gnss_rejection_conf_.velocity)
  {
    error_code = sbgEComCmdGnss1SetRejection(p_com_handle, &rejection);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the Gnss rejection %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Gnss rejection updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureOdometer(SbgEComHandle* p_com_handle)
{
  //
  // Get the odometer configuration, and compare with the loaded parameters.
  // If the conf are different, update the device with the loaded parameters.
  //
  SbgEComOdoConf  odom_conf;
  SbgErrorCode    error_code;

  error_code = sbgEComCmdOdoGetConf(p_com_handle, &odom_conf);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the Odometer configuration %s", sbgErrorCodeToString(error_code));
  }

  if (odom_conf.gain != m_odometer_conf_.gain
  || odom_conf.gainError != m_odometer_conf_.gainError
  || odom_conf.reverseMode != m_odometer_conf_.reverseMode)
  {
    error_code = sbgEComCmdOdoSetConf(p_com_handle, &m_odometer_conf_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the Odometer configuration %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Odometer configuration updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureOdometerLevelArm(SbgEComHandle* p_com_handle)
{
  //
  // Get the odometer level arm, and compare with the loaded parameters.
  // If the level arms are different, update the device with the loaded parameters.
  //
  float         leverArm[3];
  SbgErrorCode  error_code;

  error_code = sbgEComCmdOdoGetLeverArm(p_com_handle, leverArm);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the odometer level arms %s", sbgErrorCodeToString(error_code));
  }

  if (leverArm[0] != m_odometer_level_arm_[0]
  || leverArm[1] != m_odometer_level_arm_[1]
  || leverArm[2] != m_odometer_level_arm_[2])
  {
    error_code = sbgEComCmdOdoSetLeverArm(p_com_handle, m_odometer_level_arm_);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the odometer level arms %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Odometer level arms updated on the device.");
      m_rebootNeeded = true;
    }
  }
}

void ConfigStore::configureOdometerRejection(SbgEComHandle* p_com_handle)
{
  //
  // Get the odometer rejection mode, and compare with the loaded parameter.
  // If the mode are different, update the device with the loaded parameter.
  //
  SbgEComOdoRejectionConf odom_rejection;
  SbgErrorCode            error_code;

  error_code = sbgEComCmdOdoGetRejection(p_com_handle, &odom_rejection);

  if (error_code != SBG_NO_ERROR)
  {
    ROS_WARN("Unable to get the odometer rejection %s", sbgErrorCodeToString(error_code));
  }

  if (odom_rejection.velocity != m_odometer_rejection_conf_.velocity)
  {
    error_code = sbgEComCmdOdoSetRejection(p_com_handle, &odom_rejection);

    if (error_code != SBG_NO_ERROR)
    {
      ROS_WARN("Unable to set the odometer rejection %s", sbgErrorCodeToString(error_code));
    }
    else
    {
      ROS_INFO("SBG_DRIVER - [Param] Odometer rejection updated on the device.");
      m_rebootNeeded = true;
    }
  }   
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

bool ConfigStore::isRebootNeeded(void) const
{
  return (m_rebootNeeded | m_config_output_.isRebootNeeded());
}

int ConfigStore::getRateFrequency(void) const
{
  return m_config_output_.getRateFrequency();
}

const ConfigOutput &ConfigStore::getOutputConfiguration(void) const
{
  return m_config_output_;
}

SbgEComMagCalibMode ConfigStore::getMagCalibrationMode(void) const
{
  return m_mag_calib_mode_;
}

SbgEComMagCalibBandwidth ConfigStore::getMagCalibrationBandwidth(void) const
{
  return m_mag_calib_bandwidth_;
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void ConfigStore::loadFromRosNodeHandle(ros::NodeHandle& ref_node_handle)
{
  loadCommunicationParameters(ref_node_handle);
  loadSensorParameters(ref_node_handle);
  loadImuAlignementParameters(ref_node_handle);
  loadAidingAssignementParameters(ref_node_handle);
  loadMagnetometersParameters(ref_node_handle);
  loadGnssParameters(ref_node_handle);
  loadOdometerParameters(ref_node_handle);

  m_config_output_.loadFromRosNodeHandle(ref_node_handle);
}

void ConfigStore::initCommunicationInterface(SbgInterface* p_sbg_interface) const
{
  SbgErrorCode error_code;

  //
  // Create the serial interface with the parameters.
  //
  error_code = sbgInterfaceSerialCreate(p_sbg_interface, m_uart_port_name_.c_str(), m_uart_baud_rate_);

  if (error_code != SBG_NO_ERROR)
  {
    throw ros::Exception("SBG DRIVER - sbgInterfaceSerialCreate Error : " + std::string(sbgErrorCodeToString(error_code)));
  }
}

void ConfigStore::configureComHandle(SbgEComHandle* p_com_handle)
{
  m_config_output_.configureComHandle(p_com_handle, m_output_port_);

  configureInitCondition(p_com_handle);
  configureMotionProfile(p_com_handle);
  configureImuAlignement(p_com_handle);
  configureAidingAssignement(p_com_handle);
  configureMagModel(p_com_handle);
  configureMagRejection(p_com_handle);
  configureGnssLevelArm(p_com_handle);
  configureGnssModel(p_com_handle);
  configureGnssRejection(p_com_handle);
  configureOdometer(p_com_handle);
  configureOdometerLevelArm(p_com_handle);
  configureOdometerRejection(p_com_handle);
}