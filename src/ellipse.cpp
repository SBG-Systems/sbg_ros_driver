#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/thread/xtime.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include "ellipse.h"

using namespace std;

// From ros_com/recorder
std::string timeToStr(ros::WallTime ros_t){
    (void)ros_t;
    std::stringstream msg;
    const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const f = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    msg.imbue(std::locale(msg.getloc(),f));
    msg << now;
    return msg.str();
}

Ellipse::Ellipse(ros::NodeHandle *n){
  m_node = n;
  load_param();
}

Ellipse::~Ellipse(){
  sbgEComClose(&m_comHandle);
  sbgInterfaceSerialDestroy(&m_sbgInterface);
}

void Ellipse::connect(){
  SbgErrorCode errorCode;

  // Set the parameters of the Interface (port, baud_rate)
  errorCode = sbgInterfaceSerialCreate(&m_sbgInterface, m_uartPortName.c_str(), m_uartBaudRate);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("SBG DRIVER - sbgInterfaceSerialCreate Error : %s", sbgErrorCodeToString(errorCode));}

  // Init the SBG
  errorCode = sbgEComInit(&m_comHandle, &m_sbgInterface);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("SBG DRIVER - sbgEComInit Error : %s", sbgErrorCodeToString(errorCode));}

  // Get Infos
  readDeviceInfo(&m_comHandle);
}

void Ellipse::init_callback()
{
  SbgErrorCode errorCode = sbgEComSetReceiveLogCallback(&m_comHandle, onLogReceivedCallback, this);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("SBG DRIVER - sbgEComSetReceiveLogCallback Error : %s", sbgErrorCodeToString(errorCode));}
}

void Ellipse::configure(){
  bool change = false;
  change |= set_cmd_init_parameters();
  change |= set_cmd_motion_profile();
  change |= set_cmd_imu_lever_arm();
  change |= set_cmd_aiding_assignement();
  change |= set_cmd_mag_model();
  change |= set_cmd_mag_reject_mode();
  change |= set_cmd_gnss_model();
  change |= set_cmd_gnss_lever_arm();
  change |= set_cmd_gnss_reject_mode();
  change |= set_cmd_odom_conf();
  change |= set_cmd_odom_lever_arm();
  change |= set_cmd_odom_reject_mode();
  change |= set_cmd_output();

  if(change){
    // // SAVE AND REBOOT
    save_config();
  }
}

void Ellipse::save_config(){
  ROS_INFO("SBG DRIVER - The configuration of the Ellipse was updated according to the configuration file");
    SbgErrorCode errorCode = sbgEComCmdSettingsAction(&m_comHandle, SBG_ECOM_SAVE_SETTINGS);
    if (errorCode != SBG_NO_ERROR)
      ROS_WARN("SBG DRIVER - sbgEComCmdSettingsAction (Saving) Error : %s", sbgErrorCodeToString(errorCode));
    else
      ROS_INFO("SBG DRIVER - SAVED & REBOOT");
}

void Ellipse::load_param(){
  ros::NodeHandle n_private("~");
  m_uartPortName = n_private.param<std::string>("uartConf/portName", "/dev/ttyUSB0");
  m_uartBaudRate = (uint32) n_private.param<int>("uartConf/baudRate", 115200);
  m_portOutput = (SbgEComOutputPort) n_private.param<int>("uartConf/portID", 0);

  m_initLat = (double) n_private.param<double>("sensorParameters/initLat", 48.419727);
  m_initLong = (double) n_private.param<double>("sensorParameters/initLong", -4.472119);
  m_initAlt = (double) n_private.param<double>("sensorParameters/initAlt", 100);
  m_initYear = (uint16) n_private.param<int>("sensorParameters/year", 2018);
  m_initMonth = (uint8) n_private.param<int>("sensorParameters/month", 03);
  m_initDay = (uint8) n_private.param<int>("sensorParameters/day", 10);
  m_motionProfileId = (uint32) n_private.param<int>("sensorParameters/motionProfie", 1);

  m_imuAxisDirectionX = n_private.param<int>("imuAlignementLeverArm/axisDirectionX",0);
  m_imuAxisDirectionY = n_private.param<int>("imuAlignementLeverArm/axisDirectionY",0);
  m_imuMisRoll = n_private.param<float>("imuAlignementLeverArm/misRoll",0);
  m_imuMisPitch = n_private.param<float>("imuAlignementLeverArm/misPitch",0);
  m_imuMisYaw = n_private.param<float>("imuAlignementLeverArm/misYaw",0);
  m_imuLeverArm[0] = n_private.param<float>("imuAlignementLeverArm/leverArmX",0);
  m_imuLeverArm[1] = n_private.param<float>("imuAlignementLeverArm/leverArmY",0);
  m_imuLeverArm[2] = n_private.param<float>("imuAlignementLeverArm/leverArmZ",0);

  m_gnss1ModulePortAssignment = n_private.param<int>("aidingAssignment/gnss1ModulePortAssignment", 1);
  m_gnss1ModuleSyncAssignment = n_private.param<int>("aidingAssignment/gnss1ModuleSyncAssignment", 0);
  m_rtcmPortAssignment = n_private.param<int>("aidingAssignment/rtcmPortAssignment", 255);
  m_odometerPinAssignment = n_private.param<int>("aidingAssignment/odometerPinAssignment", 0);

  m_magModelId = (uint32) n_private.param<int>("magnetometer/magnetometerModel", 201);
  m_magRejectMode = n_private.param<int>("magnetometer/magnetometerRejectMode", 1);

  m_gnssModelId = (uint32) n_private.param<int>("gnss/gnss_model_id", 102);
  m_gnss1LeverArmX = n_private.param<float>("gnss/leverArmX", 0);
  m_gnss1LeverArmY = n_private.param<float>("gnss/leverArmY", 0);
  m_gnss1LeverArmZ = n_private.param<float>("gnss/leverArmZ", 0);
  m_gnss1PitchOffset = n_private.param<float>("gnss/pitchOffset", 0);
  m_gnss1YawOffset = n_private.param<float>("gnss/yawOffset", 0);
  m_gnss1AntennaDistance = n_private.param<float>("gnss/antennaDistance", 0);
  m_gnss1PosRejectMode = n_private.param<int>("gnss/posRejectMode", 1);
  m_gnss1VelRejectMode = n_private.param<int>("gnss/velRejectMode", 1);
  m_gnss1HdtRejectMode = n_private.param<int>("gnss/hdtRejectMode", 1);

  m_odomGain = n_private.param<float>("odom/gain", 4800);
  m_odomGainError = (uint8) n_private.param<int>("odom/gain_error", 0.1);
  m_odomDirection = n_private.param<bool>("odom/direction", 0);
  m_odomLever[0] = n_private.param<float>("odom/leverArmX", 0);
  m_odomLever[1] = n_private.param<float>("odom/leverArmY", 0);
  m_odomLever[2] = n_private.param<float>("odom/leverArmZ", 0);
  m_odomRejectMode = n_private.param<int>("odom/rejectMode", 1);

  m_timeReference = (uint8)n_private.param<int>("output/timeReference",0);

  m_log_status = n_private.param<int>("output/log_status", 0);
  m_log_imu_data = n_private.param<int>("output/log_imu_data", 0);
  m_log_ekf_euler = n_private.param<int>("output/log_ekf_euler", 0);
  m_log_ekf_quat = n_private.param<int>("output/log_ekf_quat", 0);
  m_log_ekf_nav = n_private.param<int>("output/log_ekf_nav", 0);
  m_log_ship_motion = n_private.param<int>("output/log_ship_motion", 0);
  m_log_utc_time = n_private.param<int>("output/log_utc_time", 0);
  m_log_mag = n_private.param<int>("output/log_mag", 0);
  m_log_mag_calib = n_private.param<int>("output/log_mag_calib", 0);
  m_log_gps1_vel = n_private.param<int>("output/log_gps1_vel", 0);
  m_log_gps1_pos = n_private.param<int>("output/log_gps1_pos", 0);
  m_log_gps1_hdt = n_private.param<int>("output/log_gps1_hdt", 0);
  m_log_gps1_raw = n_private.param<int>("output/log_gps1_raw", 0);
  m_log_odo_vel = n_private.param<int>("output/log_odo_vel", 0);
  m_log_event_a = n_private.param<int>("output/log_event_a", 0);
  m_log_event_b = n_private.param<int>("output/log_event_b", 0);
  m_log_event_c = n_private.param<int>("output/log_event_c", 0);
  m_log_event_d = n_private.param<int>("output/log_event_d", 0);
  m_log_event_e = n_private.param<int>("output/log_event_e", 0);
  m_log_pressure = n_private.param<int>("output/log_pressure", 0);
  m_rate_frequency = n_private.param<int>("output/frequency",0);

  m_magnetic_calibration_mode = n_private.param<int>("magnetometer/calibration/mode",1);
  m_magnetic_calibration_bandwidth = n_private.param<int>("magnetometer/calibration/bandwidth",2);
}

/*!
 *  Callback definition called each time a new log is received.
 * 
 *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
 *  \param[in]  msgClass                Class of the message we have received
 *  \param[in]  msg                   Message ID of the log received.
 *  \param[in]  pLogData                Contains the received log data as an union.
 *  \param[in]  pUserArg                Optional user supplied argument.
 *  \return                       SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode Ellipse::onLogReceivedCallback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg)
{
  Ellipse *p_ellipse;
  p_ellipse = (Ellipse*)(pUserArg);

  p_ellipse->onLogReceived(msgClass, msg, pLogData);
}

/*!
 * Function to handle the received log.
 * 
 * \param[in]  msgClass               Class of the message we have received
 * \param[in]  msg                    Message ID of the log received.
 * \param[in]  pLogData               Contains the received log data as an union.
 */
void Ellipse::onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData)
{
  //
  // Publish the received SBG log.
  //
  m_message_publisher_.publish(msgClass, msg, *pLogData);
}

bool Ellipse::set_cmd_init_parameters(){
  SbgEComInitConditionConf init_condition;
  sbgEComCmdSensorGetInitCondition(&m_comHandle, &init_condition);
  if(init_condition.year != m_initYear
     || init_condition.day != m_initDay
     || init_condition.month != m_initMonth
     || init_condition.latitude != m_initLat
     || init_condition.longitude != m_initLong
     || init_condition.altitude != m_initAlt){
    init_condition.year = m_initYear;
    init_condition.day = m_initDay;
    init_condition.month = m_initMonth;
    init_condition.latitude = m_initLat;
    init_condition.longitude = m_initLong;
    init_condition.altitude = m_initAlt;
    sbgEComCmdSensorSetInitCondition(&m_comHandle, &init_condition);
    ROS_INFO("SBG DRIVER - [Param] New init");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_motion_profile(){
  SbgEComModelInfo motion_profile;
  sbgEComCmdSensorGetMotionProfileInfo(&m_comHandle, &motion_profile);
  if(motion_profile.id != m_motionProfileId){
    sbgEComCmdSensorSetMotionProfileId(&m_comHandle, m_motionProfileId);
    ROS_INFO("SBG DRIVER - [Param] motion profile");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_imu_lever_arm(){
  SbgEComSensorAlignmentInfo pAlignConf;
  float leverArm[3];
  sbgEComCmdSensorGetAlignmentAndLeverArm(&m_comHandle, &pAlignConf, leverArm);

  if(leverArm[0] != m_imuLeverArm[0]
     || leverArm[1] != m_imuLeverArm[1]
     || leverArm[2] != m_imuLeverArm[2]
     || pAlignConf.axisDirectionX != m_imuAxisDirectionX
     || pAlignConf.axisDirectionY != m_imuAxisDirectionY
     || pAlignConf.misRoll != m_imuMisRoll
     || pAlignConf.misPitch != m_imuMisPitch
     || pAlignConf.misYaw != m_imuMisYaw ){
    leverArm[0] = m_imuLeverArm[0];
    leverArm[1] = m_imuLeverArm[1];
    leverArm[2] = m_imuLeverArm[2];
    pAlignConf.axisDirectionX = (SbgEComAxisDirection) m_imuAxisDirectionX;
    pAlignConf.axisDirectionY = (SbgEComAxisDirection) m_imuAxisDirectionY;
    pAlignConf.misRoll = m_imuMisRoll;
    pAlignConf.misPitch = m_imuMisPitch;
    pAlignConf.misYaw = m_imuMisYaw ;
    sbgEComCmdSensorSetAlignmentAndLeverArm(&m_comHandle, &pAlignConf, leverArm);
    ROS_INFO("SBG DRIVER - [Param] imu lever arm");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_aiding_assignement(){
  SbgEComAidingAssignConf aidingAssign;
  sbgEComCmdSensorGetAidingAssignment(&m_comHandle, &aidingAssign);

  if(aidingAssign.gps1Port != m_gnss1ModulePortAssignment
     || aidingAssign.gps1Sync != m_gnss1ModuleSyncAssignment
     || aidingAssign.odometerPinsConf != m_odometerPinAssignment
     || aidingAssign.rtcmPort != m_rtcmPortAssignment){
    aidingAssign.gps1Port = (SbgEComModulePortAssignment) m_gnss1ModulePortAssignment;
    aidingAssign.gps1Sync = (SbgEComModuleSyncAssignment) m_gnss1ModuleSyncAssignment;
    aidingAssign.odometerPinsConf = (SbgEComOdometerPinAssignment) m_odometerPinAssignment;
    aidingAssign.rtcmPort = (SbgEComModulePortAssignment) m_rtcmPortAssignment;
    sbgEComCmdSensorSetAidingAssignment(&m_comHandle, &aidingAssign);
    ROS_INFO("SBG DRIVER - [Param] Aiding assignement");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_mag_model(){
  SbgEComModelInfo model_info;
  sbgEComCmdMagGetModelInfo(&m_comHandle, &model_info);
  if(model_info.id != m_magModelId){
    sbgEComCmdMagSetModelId(&m_comHandle, m_magModelId);
    ROS_INFO("SBG DRIVER - [Param] Mag model");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_mag_reject_mode(){
  SbgEComMagRejectionConf rejection;
  sbgEComCmdMagGetRejection(&m_comHandle, &rejection);
  if(rejection.magneticField != m_magRejectMode){
    rejection.magneticField = (SbgEComRejectionMode) m_magRejectMode;
    sbgEComCmdMagSetRejection(&m_comHandle, &rejection);
    ROS_INFO("SBG DRIVER - [Param] Mag reject mode");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_gnss_model(){
  SbgEComModelInfo model_info;
  sbgEComCmdGnss1GetModelInfo(&m_comHandle, &model_info);
  if(model_info.id != m_gnssModelId){
    sbgEComCmdMagSetModelId(&m_comHandle, m_gnssModelId);
    ROS_INFO("SBG DRIVER - [Param] gnss model");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_gnss_lever_arm(){
  SbgEComGnssAlignmentInfo pAlignInfo;
  sbgEComCmdGnss1GetLeverArmAlignment(&m_comHandle, &pAlignInfo);
  if(pAlignInfo.antennaDistance != m_gnss1AntennaDistance
     || pAlignInfo.leverArmX != m_gnss1LeverArmX
     || pAlignInfo.leverArmY != m_gnss1LeverArmY
     || pAlignInfo.leverArmZ != m_gnss1LeverArmZ
     || pAlignInfo.pitchOffset != m_gnss1PitchOffset
     || pAlignInfo.yawOffset != m_gnss1YawOffset ){
    pAlignInfo.antennaDistance = m_gnss1AntennaDistance;
    pAlignInfo.leverArmX = m_gnss1LeverArmX;
    pAlignInfo.leverArmY = m_gnss1LeverArmY;
    pAlignInfo.leverArmZ = m_gnss1LeverArmZ;
    pAlignInfo.pitchOffset = m_gnss1PitchOffset;
    pAlignInfo.yawOffset = m_gnss1YawOffset ;
    sbgEComCmdGnss1SetLeverArmAlignment(&m_comHandle, &pAlignInfo);
    ROS_INFO("SBG DRIVER - [Param] Gnss lever arm");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_gnss_reject_mode(){
  SbgEComGnssRejectionConf rejection;
  sbgEComCmdGnss1GetRejection(&m_comHandle, &rejection);

  if(rejection.hdt != m_gnss1HdtRejectMode
     || rejection.position != m_gnss1PosRejectMode
     || rejection.velocity != m_gnss1VelRejectMode){

    rejection.hdt = (SbgEComRejectionMode) m_gnss1HdtRejectMode;
    rejection.position = (SbgEComRejectionMode) m_gnss1PosRejectMode;
    rejection.velocity = (SbgEComRejectionMode) m_gnss1VelRejectMode;
    sbgEComCmdGnss1SetRejection(&m_comHandle, &rejection);
    ROS_INFO("SBG DRIVER - [Param] gnss reject mode");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_odom_conf(){
  SbgEComOdoConf odom;
  sbgEComCmdOdoGetConf(&m_comHandle, &odom);

  if(odom.gain != m_odomGain
     || odom.gainError != m_odomGainError
     || odom.reverseMode != m_odomDirection){

    odom.gain = m_odomGain;
    odom.gainError = m_odomGainError;
    odom.reverseMode = m_odomDirection;
    sbgEComCmdOdoSetConf(&m_comHandle, &odom);
    ROS_INFO("SBG DRIVER - [Param] odom conf");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_odom_lever_arm(){
  float leverArm[3];
  sbgEComCmdOdoGetLeverArm(&m_comHandle, leverArm);
  if(leverArm[0] != m_odomLever[0]
     || leverArm[1] != m_odomLever[1]
     || leverArm[2] != m_odomLever[2]
     ){
    leverArm[0] = m_odomLever[0];
    leverArm[1] = m_odomLever[1];
    leverArm[2] = m_odomLever[2];
    sbgEComCmdOdoSetLeverArm(&m_comHandle, leverArm);
    ROS_INFO("SBG DRIVER - [Param] odom lever arm ");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_odom_reject_mode(){
  SbgEComOdoRejectionConf rejection;
  sbgEComCmdOdoGetRejection(&m_comHandle, &rejection);
  if(rejection.velocity != m_odomRejectMode){
    rejection.velocity = (SbgEComRejectionMode) m_odomRejectMode;
    sbgEComCmdOdoSetRejection(&m_comHandle, &rejection);
    ROS_INFO("SBG DRIVER - [Param] odom reject mode");
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_output(){
  bool change = false;

  SbgEComOutputMode pConf;
  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_status;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, pConf);
    ROS_INFO("SBG DRIVER - [Param] log status");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME,&pConf);
  if(pConf != m_log_utc_time){
    pConf = (SbgEComOutputMode) m_log_utc_time;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, pConf);
    ROS_INFO("SBG DRIVER - [Param] log utc");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA,&pConf);
  if(pConf != m_log_imu_data){
    pConf = (SbgEComOutputMode) m_log_imu_data;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, pConf);
    ROS_INFO("SBG DRIVER - [Param] log imu data");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER,&pConf);
  if(pConf != m_log_ekf_euler){
    pConf = (SbgEComOutputMode) m_log_ekf_euler;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, pConf);
    ROS_INFO("SBG DRIVER - [Param] log ekf euler");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT,&pConf);
  if(pConf != m_log_ekf_quat){
    pConf = (SbgEComOutputMode) m_log_ekf_quat;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, pConf);
    ROS_INFO("SBG DRIVER - [Param] log ekf quat");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV,&pConf);
  if(pConf != m_log_ekf_nav){
    pConf = (SbgEComOutputMode) m_log_ekf_nav;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, pConf);
    ROS_INFO("SBG DRIVER - [Param] log ekf nav");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION,&pConf);
  if(pConf != m_log_ship_motion){
    pConf = (SbgEComOutputMode) m_log_ship_motion;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, pConf);
    ROS_INFO("SBG DRIVER - [Param] ship motion");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG,&pConf);
  if(pConf != m_log_mag){
    pConf = (SbgEComOutputMode) m_log_mag;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, pConf);
    ROS_INFO("SBG DRIVER - [Param] log mag");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB,&pConf);
  if(pConf != m_log_mag_calib){
    pConf = (SbgEComOutputMode) m_log_mag_calib;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB, pConf);
    ROS_INFO("SBG DRIVER - [Param] mag calib");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL,&pConf);
  if(pConf != m_log_gps1_vel){
    pConf = (SbgEComOutputMode) m_log_gps1_vel;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, pConf);
    ROS_INFO("SBG DRIVER - [Param] gps1 vel");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS,&pConf);
  if(pConf != m_log_gps1_pos){
    pConf = (SbgEComOutputMode) m_log_gps1_pos;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, pConf);
    ROS_INFO("SBG DRIVER - [Param] gps1 pos");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT,&pConf);
  if(pConf != m_log_gps1_hdt){
    pConf = (SbgEComOutputMode) m_log_gps1_hdt;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, pConf);
    ROS_INFO("SBG DRIVER - [Param] gps1 hdt");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW,&pConf);
  if(pConf != m_log_gps1_raw){
    pConf = (SbgEComOutputMode) m_log_gps1_raw;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW, pConf);
    ROS_INFO("SBG DRIVER - [Param] gps1 raw");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL,&pConf);
  if(pConf != m_log_odo_vel){
    pConf = (SbgEComOutputMode) m_log_odo_vel;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL, pConf);
    ROS_INFO("SBG DRIVER - [Param] odo vel");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A,&pConf);
  if(pConf != m_log_event_a){
    pConf = (SbgEComOutputMode) m_log_event_a;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A, pConf);
    ROS_INFO("SBG DRIVER - [Param] event a");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B,&pConf);
  if(pConf != m_log_event_b){
    pConf = (SbgEComOutputMode) m_log_event_b;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B, pConf);
    ROS_INFO("SBG DRIVER - [Param] event b");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C,&pConf);
  if(pConf != m_log_event_c){
    pConf = (SbgEComOutputMode) m_log_event_c;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C, pConf);
    ROS_INFO("SBG DRIVER - [Param] event c");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D,&pConf);
  if(pConf != m_log_event_d){
    pConf = (SbgEComOutputMode) m_log_event_d;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D, pConf);
    ROS_INFO("SBG DRIVER - [Param] event d");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_E,&pConf);
  if(pConf != m_log_event_e){
    pConf = (SbgEComOutputMode) m_log_event_e;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_E, pConf);
    ROS_INFO("SBG DRIVER - [Param] event e");
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_PRESSURE,&pConf);
  if(pConf != m_log_pressure){
    pConf = (SbgEComOutputMode) m_log_pressure;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_PRESSURE, pConf);
    ROS_INFO("SBG DRIVER - [Param] log pressure");
    change = true;
  }

  return change;
}


bool Ellipse::start_mag_calibration(){
  SbgErrorCode errorCode = sbgEComCmdMagStartCalib(&m_comHandle, (SbgEComMagCalibMode)m_magnetic_calibration_mode, (SbgEComMagCalibBandwidth)m_magnetic_calibration_bandwidth);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagStartCalib Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }else{
    ROS_INFO("SBG DRIVER - MAG CALIBRATION Start calibration");
    ROS_INFO("SBG DRIVER - MAG CALIBRATION mode : %s", MAG_CALIB_MODE[(SbgEComMagCalibMode)m_magnetic_calibration_mode].c_str());
    ROS_INFO("SBG DRIVER - MAG CALIBRATION bandwidth : %s", MAG_CALIB_BW[(SbgEComMagCalibBandwidth)m_magnetic_calibration_bandwidth].c_str());
    return true;
  }
}

bool Ellipse::end_mag_calibration(){
  SbgErrorCode errorCode = sbgEComCmdMagComputeCalib(&m_comHandle, &m_magCalibResults);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagStartCalib Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }

  ROS_INFO("SBG DRIVER - MAG CALIBRATION - %s", MAG_CALIB_QUAL[m_magCalibResults.quality].c_str());
  ROS_INFO("SBG DRIVER - MAG CALIBRATION - %s", MAG_CALIB_CONF[m_magCalibResults.confidence].c_str());

  /// ************* WARNING IF ISSUES WITH COMPUTATIONS ************* //

  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_NOT_ENOUGH_POINTS)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - Not enough valid points. Maybe you are moving too fast");
  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_TOO_MUCH_DISTORTIONS)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - Unable to find a calibration solution. Maybe there are too much non static distortions");
  if(m_magCalibResults.advancedStatus & SBG_ECOM_MAG_CALIB_ALIGNMENT_ISSUE)
    ROS_WARN("SBG DRIVER - MAG CALIBRATION - The magnetic calibration has troubles to correct the magnetometers and inertial frame alignment");
  if(m_magnetic_calibration_mode == SBG_ECOM_MAG_CALIB_MODE_2D){
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
  save_file << "* CALIB_MODE = " << MAG_CALIB_MODE[(SbgEComMagCalibMode)m_magnetic_calibration_mode] << endl;
  save_file << "* CALIB_BW = " << MAG_CALIB_BW[(SbgEComMagCalibBandwidth)m_magnetic_calibration_bandwidth] << endl;
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
  SbgErrorCode errorCode = sbgEComCmdMagSetCalibData(&m_comHandle, m_magCalibResults.offset, m_magCalibResults.matrix);
  if (errorCode != SBG_NO_ERROR){
    ROS_WARN("SBG DRIVER - sbgEComCmdMagSetCalibData Error : %s", sbgErrorCodeToString(errorCode));
    return false;
  }
  else{
    ROS_INFO("SBG DRIVER - MAG CALIBRATION - Saving data to the device");
    return true;
  }
}

/*!
 * Initialize the publishers according to the configuration.
 */
void Ellipse::initPublishers(void)
{
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_STATUS, static_cast<SbgEComOutputMode>(m_log_status), "status");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_UTC_TIME, static_cast<SbgEComOutputMode>(m_log_utc_time), "utc_time");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_IMU_DATA, static_cast<SbgEComOutputMode>(m_log_imu_data), "imu_data");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_MAG, static_cast<SbgEComOutputMode>(m_log_mag), "mag");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_MAG_CALIB, static_cast<SbgEComOutputMode>(m_log_mag_calib), "mag_calib");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EKF_EULER, static_cast<SbgEComOutputMode>(m_log_ekf_euler), "ekf_euler");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EKF_QUAT, static_cast<SbgEComOutputMode>(m_log_ekf_quat), "ekf_quat");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EKF_NAV, static_cast<SbgEComOutputMode>(m_log_ekf_nav), "ekf_nav");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_SHIP_MOTION, static_cast<SbgEComOutputMode>(m_log_ship_motion), "ship_motion");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_GPS1_VEL, static_cast<SbgEComOutputMode>(m_log_gps1_vel), "gps_vel");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_GPS1_POS, static_cast<SbgEComOutputMode>(m_log_gps1_pos), "gps_pos");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_GPS1_HDT, static_cast<SbgEComOutputMode>(m_log_gps1_hdt), "gps_hdt");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_GPS1_RAW, static_cast<SbgEComOutputMode>(m_log_gps1_raw), "gps_raw");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_ODO_VEL, static_cast<SbgEComOutputMode>(m_log_odo_vel), "odo_vel");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EVENT_A, static_cast<SbgEComOutputMode>(m_log_event_a), "eventA");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EVENT_B, static_cast<SbgEComOutputMode>(m_log_event_b), "eventB");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EVENT_C, static_cast<SbgEComOutputMode>(m_log_event_c), "eventC");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EVENT_D, static_cast<SbgEComOutputMode>(m_log_event_d), "eventD");
  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_EVENT_E, static_cast<SbgEComOutputMode>(m_log_event_e), "eventE");

  m_message_publisher_.initPublisher(m_node, SBG_ECOM_LOG_PRESSURE, static_cast<SbgEComOutputMode>(m_log_pressure), "pressure");

  //
  // Check if the rate frequency has to be defined according to the defined publishers.
  //
  if(m_rate_frequency==0)
  {
    m_rate_frequency = m_message_publisher_.getOutputFrequency();
  }
}

/*!
 * Periodic handle of the connected Ellipse.
 */
void Ellipse::periodicHandle(void)
{
  sbgEComHandle(&m_comHandle);
}

/*!
 * Read the device informations.
 * 
 * \param[in] p_com_handle      SBG communication handle.
 */
void Ellipse::readDeviceInfo(SbgEComHandle* p_com_handle)
{
  SbgEComDeviceInfo pInfo;
  SbgErrorCode errorCode = sbgEComCmdGetInfo(p_com_handle, &pInfo);
  if (errorCode != SBG_NO_ERROR)
  {
    ROS_WARN("SBG DRIVER - sbgEComCmdGetInfo Error : %s", sbgErrorCodeToString(errorCode));
  }

  char version[32];

  ROS_INFO("SBG DRIVER - productCode = %s", pInfo.productCode);
  ROS_INFO("SBG DRIVER - serialNumber = %u", pInfo.serialNumber);

  sbgVersionToStringEncoded(pInfo.calibationRev, version, 32);
  ROS_INFO("SBG DRIVER - calibationRev = %s", version);

  ROS_INFO("SBG DRIVER - calibrationYear = %u", pInfo.calibrationYear);
  ROS_INFO("SBG DRIVER - calibrationMonth = %u", pInfo.calibrationMonth);
  ROS_INFO("SBG DRIVER - calibrationDay = %u", pInfo.calibrationDay);

  sbgVersionToStringEncoded(pInfo.hardwareRev, version, 32);
  ROS_INFO("SBG DRIVER - hardwareRev = %s", version);

  sbgVersionToStringEncoded(pInfo.firmwareRev, version, 32);
  ROS_INFO("SBG DRIVER - firmwareRev = %s", version); 
}

/*!
 * Get the device frequency.
 * 
 * \return                      Device frequency to read the logs.
 */
int Ellipse::getDeviceRateFrequency(void) const
{
  return m_rate_frequency;
}