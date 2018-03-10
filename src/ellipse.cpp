#include "ellipse.h"
#include "ellipse_msg.h"

Ellipse::Ellipse(ros::NodeHandle *n){
  m_node = n;
  m_uartPortName = "/dev/ttyUSB0";
  m_uartBaudRate = 115200;
  m_rate_frequency = 25;
}

Ellipse::~Ellipse(){

}

void Ellipse::connect(){
  SbgErrorCode errorCode;

  // Set the parameters of the Interface (port, baud_rate)
  errorCode = sbgInterfaceSerialCreate(&m_sbgInterface, m_uartPortName.c_str(), m_uartBaudRate);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgInterfaceSerialCreate Error : %s", sbgErrorCodeToString(errorCode));}

  // Init the SBG
  errorCode = sbgEComInit(&m_comHandle, &m_sbgInterface);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComInit Error : %s", sbgErrorCodeToString(errorCode));}

  // Get Infos
  read_GetInfo(&m_comHandle);
}

void Ellipse::init_callback(){
  SbgErrorCode errorCode = sbgEComSetReceiveLogCallback(&m_comHandle, onLogReceived, this);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComSetReceiveLogCallback Error : %s", sbgErrorCodeToString(errorCode));}
}

void Ellipse::init_publishers(){
  m_sbgStatus_pub = m_node->advertise<sbg_driver::SbgStatus>("status",10);
  m_sbgUtcTime_pub = m_node->advertise<sbg_driver::SbgUtcTime>("utc_time",10);
  m_sbgImuData_pub = m_node->advertise<sbg_driver::SbgImuData>("imu_data",10);
  m_sbgEkfEuler_pub = m_node->advertise<sbg_driver::SbgEkfEuler>("ekf_euler",10);
  m_sbgEkfQuat_pub = m_node->advertise<sbg_driver::SbgEkfQuat>("ekf_quat",10);
  m_sbgEkfNav_pub = m_node->advertise<sbg_driver::SbgEkfNav>("ekf_nav",10);
  m_sbgShipMotion_pub = m_node->advertise<sbg_driver::SbgShipMotion>("ship_motion",10);
  m_sbgMag_pub = m_node->advertise<sbg_driver::SbgMag>("mag",10);
  m_sbgMagCalib_pub = m_node->advertise<sbg_driver::SbgMagCalib>("mag_calib",10);
  m_sbgGpsVel_pub = m_node->advertise<sbg_driver::SbgGpsVel>("gps_vel",10);
  m_sbgGpsPos_pub = m_node->advertise<sbg_driver::SbgGpsPos>("gps_pos",10);
  m_sbgGpsHdt_pub = m_node->advertise<sbg_driver::SbgGpsHdt>("gps_hdt",10);
  m_sbgGpsRaw_pub = m_node->advertise<sbg_driver::SbgGpsRaw>("gps_raw",10);
  m_sbgOdoVel_pub = m_node->advertise<sbg_driver::SbgOdoVel>("odo_vel",10);
  m_sbgEvent_pub = m_node->advertise<sbg_driver::SbgEvent>("event",10);
  m_sbgPressure_pub = m_node->advertise<sbg_driver::SbgPressure>("pressure",10);
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
    SbgErrorCode errorCode = sbgEComCmdSettingsAction(&m_comHandle, SBG_ECOM_SAVE_SETTINGS);
    if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComCmdSettingsAction (Saving) Error : %s", sbgErrorCodeToString(errorCode));}
  }
}

void Ellipse::publish(){
  SbgErrorCode errorCode = sbgEComHandle(&m_comHandle);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("sbgEComHandle Error : %s", sbgErrorCodeToString(errorCode));}

  if(m_new_sbgStatus){
    m_new_sbgStatus = false;
    m_sbgStatus_pub.publish(m_sbgStatus_msg);
  }

  if(m_new_sbgUtcTime){
    m_new_sbgUtcTime = false;
    m_sbgUtcTime_pub.publish(m_sbgUtcTime_msg);
  }

  if(m_new_sbgImuData){
    m_new_sbgImuData = false;
    m_sbgImuData_pub.publish(m_sbgImuData_msg);
  }

  if(m_new_sbgEkfEuler){
    m_new_sbgEkfEuler = false;
    m_sbgEkfEuler_pub.publish(m_sbgEkfEuler_msg);
  }

  if(m_new_sbgEkfQuat){
    m_new_sbgEkfQuat = false;
    m_sbgEkfQuat_pub.publish(m_sbgEkfQuat_msg);
  }

  if(m_new_sbgEkfNav){
    m_new_sbgEkfNav = false;
    m_sbgEkfNav_pub.publish(m_sbgEkfNav_msg);
  }

  if(m_new_sbgShipMotion){
    m_new_sbgShipMotion = false;
    m_sbgShipMotion_pub.publish(m_sbgShipMotion_msg);
  }

  if(m_new_sbgMag){
    m_new_sbgMag = false;
    m_sbgMag_pub.publish(m_sbgMag_msg);
  }

  if(m_new_sbgMagCalib){
    m_new_sbgMagCalib = false;
    m_sbgMagCalib_pub.publish(m_sbgMagCalib_msg);
  }

  if(m_new_sbgGpsVel){
    m_new_sbgGpsVel = false;
    m_sbgGpsVel_pub.publish(m_sbgGpsVel_msg);
  }

  if(m_new_sbgGpsPos){
    m_new_sbgGpsPos = false;
    m_sbgGpsPos_pub.publish(m_sbgGpsPos_msg);
  }

  if(m_new_sbgGpsHdt){
    m_new_sbgGpsHdt = false;
    m_sbgGpsHdt_pub.publish(m_sbgGpsHdt_msg);
  }

  if(m_new_sbgGpsRaw){
    m_new_sbgGpsRaw = false;
    m_sbgGpsRaw_pub.publish(m_sbgGpsRaw_msg);
  }

  if(m_new_sbgOdoVel){
    m_new_sbgOdoVel = false;
    m_sbgOdoVel_pub.publish(m_sbgOdoVel_msg);
  }

  if(m_new_sbgEvent){
    m_new_sbgEvent = false;
    m_sbgEvent_pub.publish(m_sbgEvent_msg);
  }

  if(m_new_sbgPressure){
    m_new_sbgPressure = false;
    m_sbgPressure_pub.publish(m_sbgPressure_msg);
  }
}


SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg){
  Ellipse *e = (Ellipse*)pUserArg;
  switch (msg){
  case SBG_ECOM_LOG_STATUS:
    read_ecom_log_status(e->m_sbgStatus_msg, pLogData);
    e->m_new_sbgStatus = true;
    break;

  case SBG_ECOM_LOG_UTC_TIME:
    read_ecom_log_utc_time(e->m_sbgUtcTime_msg, pLogData);
    e->m_new_sbgUtcTime = true;
    break;

  case SBG_ECOM_LOG_IMU_DATA:
    read_ecom_log_imu_data(e->m_sbgImuData_msg, pLogData);
    e->m_new_sbgImuData = true;
    break;

  case SBG_ECOM_LOG_EKF_EULER:
    read_ecom_log_ekf_euler(e->m_sbgEkfEuler_msg, pLogData);
    e->m_new_sbgEkfEuler = true;
    break;

  case SBG_ECOM_LOG_EKF_QUAT:
    read_ecom_log_ekf_quat(e->m_sbgEkfQuat_msg, pLogData);
    e->m_new_sbgEkfQuat = true;
    break;

  case SBG_ECOM_LOG_EKF_NAV:
    read_ecom_log_ekf_nav(e->m_sbgEkfNav_msg, pLogData);
    e->m_new_sbgEkfNav = true;
    break;

  case SBG_ECOM_LOG_SHIP_MOTION:
    read_ecom_log_ship_motion(e->m_sbgShipMotion_msg, pLogData);
    e->m_new_sbgShipMotion = true;
    break;

  case SBG_ECOM_LOG_MAG:
    read_ecom_log_mag(e->m_sbgMag_msg, pLogData);
    e->m_new_sbgMag = true;
    break;

  case SBG_ECOM_LOG_MAG_CALIB:
    read_ecom_log_mag_calib(e->m_sbgMagCalib_msg, pLogData);
    e->m_new_sbgMagCalib = true;
    break;

  case SBG_ECOM_LOG_GPS1_VEL:
    read_ecom_log_gps_vel(e->m_sbgGpsVel_msg, pLogData);
    e->m_new_sbgGpsVel = true;
    break;

  case SBG_ECOM_LOG_GPS1_POS:
    read_ecom_log_gps_pos(e->m_sbgGpsPos_msg, pLogData);
    e->m_new_sbgGpsPos = true;
    break;

  case SBG_ECOM_LOG_GPS1_HDT:
    read_ecom_log_gps_hdt(e->m_sbgGpsHdt_msg, pLogData);
    e->m_new_sbgGpsHdt = true;
    break;

  case SBG_ECOM_LOG_GPS1_RAW:
    read_ecom_log_gps_raw(e->m_sbgGpsRaw_msg, pLogData);
    e->m_new_sbgGpsRaw = true;
    break;

  case SBG_ECOM_LOG_ODO_VEL:
    read_ecom_log_odo_vel(e->m_sbgOdoVel_msg, pLogData);
    e->m_new_sbgOdoVel = true;
    break;

  case SBG_ECOM_LOG_EVENT_A:
    read_ecom_log_event(e->m_sbgEvent_msg, pLogData);
    e->m_new_sbgEvent = true;
    break;

  case SBG_ECOM_LOG_PRESSURE:
    read_ecom_log_pressure(e->m_sbgPressure_msg, pLogData);
    e->m_new_sbgPressure = true;
    break;

  default:
    break;
  }
  return SBG_NO_ERROR;
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
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_motion_profile(){
  SbgEComModelInfo motion_profile;
  sbgEComCmdSensorGetMotionProfileInfo(&m_comHandle, &motion_profile);
  if(motion_profile.id != m_motionProfileId){
    sbgEComCmdSensorSetMotionProfileId(&m_comHandle, m_motionProfileId);
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
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_aiding_assignement(){
  SbgEComAidingAssignConf aidingAssign;
  sbgEComCmdSensorGetAidingAssignment(&m_comHandle, &aidingAssign);
  if(aidingAssign.gps1Port != m_gnss1ModulePortAssignment
     || aidingAssign.gps1Sync != m_gnss1ModuleSyncAssignment
     || aidingAssign.odometerPinsConf != m_rtcmPortAssignment
     || aidingAssign.rtcmPort != m_odometerPinAssignment){
    aidingAssign.gps1Port = (SbgEComModulePortAssignment) m_gnss1ModulePortAssignment;
    aidingAssign.gps1Sync = (SbgEComModuleSyncAssignment) m_gnss1ModuleSyncAssignment;
    aidingAssign.odometerPinsConf = (SbgEComOdometerPinAssignment) m_rtcmPortAssignment;
    aidingAssign.rtcmPort = (SbgEComModulePortAssignment) m_odometerPinAssignment;
    sbgEComCmdSensorSetAidingAssignment(&m_comHandle, &aidingAssign);
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_mag_model(){
  SbgEComModelInfo model_info;
  sbgEComCmdMagGetModelInfo(&m_comHandle, &model_info);
  if(model_info.id != m_magModelId){
    sbgEComCmdMagSetModelId(&m_comHandle, m_magModelId);
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
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_gnss_model(){
  SbgEComModelInfo model_info;
  sbgEComCmdGnss1GetModelInfo(&m_comHandle, &model_info);
  if(model_info.id != m_gnssModelId){
    sbgEComCmdMagSetModelId(&m_comHandle, m_gnssModelId);
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
    return true;
  }
  return false;
}

bool Ellipse::set_cmd_gnss_reject_mode(){
  SbgEComGnssRejectionConf rejection;
  sbgEComCmdGnss1GetRejection(&m_comHandle, &rejection);
  
  if(rejection.course != m_gnss1CourseRejectMode
     || rejection.hdt != m_gnss1HdtRejectMode
     || rejection.position != m_gnss1PosRejectMode
     || rejection.velocity != m_gnss1VelRejectMode){
    
    rejection.course = (SbgEComRejectionMode) m_gnss1CourseRejectMode;
    rejection.hdt = (SbgEComRejectionMode) m_gnss1HdtRejectMode;
    rejection.position = (SbgEComRejectionMode) m_gnss1PosRejectMode;
    rejection.velocity = (SbgEComRejectionMode) m_gnss1VelRejectMode;
    sbgEComCmdGnss1SetRejection(&m_comHandle, &rejection);
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
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_utc_time;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_UTC_TIME, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_imu_data;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_mag;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_mag_calib;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG_CALIB, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_ekf_euler;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_ekf_quat;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_ekf_nav;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_ship_motion;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_SHIP_MOTION, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_gps1_vel;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_VEL, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_gps1_pos;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_gps1_hdt;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_HDT, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_gps1_raw;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_RAW, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_odo_vel;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_ODO_VEL, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_event_a;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_A, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_event_b;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_B, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_event_c;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_C, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_event_d;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EVENT_D, pConf);
    change = true;
  }

  sbgEComCmdOutputGetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_PRESSURE,&pConf);
  if(pConf != m_log_status){
    pConf = (SbgEComOutputMode) m_log_pressure;
    sbgEComCmdOutputSetConf(&m_comHandle, m_portOutput, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_PRESSURE, pConf);
    change = true;
  }

  return change;
}















