#include "ellipse.h"
#include "ellipse_msg.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <ctime>

using namespace std;

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
  read_get_info(&m_comHandle);
}

void Ellipse::init_callback(){
  SbgErrorCode errorCode = sbgEComSetReceiveLogCallback(&m_comHandle, onLogReceived, this);
  if (errorCode != SBG_NO_ERROR){ROS_WARN("SBG DRIVER - sbgEComSetReceiveLogCallback Error : %s", sbgErrorCodeToString(errorCode));}
}

void Ellipse::init_publishers(){
  if(m_log_status!=0)
    m_sbgStatus_pub = m_node->advertise<sbg_driver::SbgStatus>("status",10);
  if(m_log_utc_time !=0)
    m_sbgUtcTime_pub = m_node->advertise<sbg_driver::SbgUtcTime>("utc_time",10);
  if(m_log_imu_data !=0)
    m_sbgImuData_pub = m_node->advertise<sbg_driver::SbgImuData>("imu_data",10);
  if(m_log_ekf_euler !=0)
    m_sbgEkfEuler_pub = m_node->advertise<sbg_driver::SbgEkfEuler>("ekf_euler",10);
  if(m_log_ekf_quat !=0)
    m_sbgEkfQuat_pub = m_node->advertise<sbg_driver::SbgEkfQuat>("ekf_quat",10);
  if(m_log_ekf_nav !=0)
    m_sbgEkfNav_pub = m_node->advertise<sbg_driver::SbgEkfNav>("ekf_nav",10);
  if(m_log_ship_motion !=0)
    m_sbgShipMotion_pub = m_node->advertise<sbg_driver::SbgShipMotion>("ship_motion",10);
  if(m_log_mag !=0)
    m_sbgMag_pub = m_node->advertise<sbg_driver::SbgMag>("mag",10);
  if(m_log_mag_calib !=0)
    m_sbgMagCalib_pub = m_node->advertise<sbg_driver::SbgMagCalib>("mag_calib",10);
  if(m_log_gps1_vel !=0)
    m_sbgGpsVel_pub = m_node->advertise<sbg_driver::SbgGpsVel>("gps_vel",10);
  if(m_log_gps1_pos !=0)
    m_sbgGpsPos_pub = m_node->advertise<sbg_driver::SbgGpsPos>("gps_pos",10);
  if(m_log_gps1_hdt !=0)
    m_sbgGpsHdt_pub = m_node->advertise<sbg_driver::SbgGpsHdt>("gps_hdt",10);
  if(m_log_gps1_raw !=0)
    m_sbgGpsRaw_pub = m_node->advertise<sbg_driver::SbgGpsRaw>("gps_raw",10);
  if(m_log_odo_vel !=0)
    m_sbgOdoVel_pub = m_node->advertise<sbg_driver::SbgOdoVel>("odo_vel",10);
  if(m_log_event_a !=0)
    m_sbgEventA_pub = m_node->advertise<sbg_driver::SbgEvent>("eventA",10);
  if(m_log_event_b !=0)
    m_sbgEventB_pub = m_node->advertise<sbg_driver::SbgEvent>("eventB",10);
  if(m_log_event_c !=0)
    m_sbgEventC_pub = m_node->advertise<sbg_driver::SbgEvent>("eventC",10);
  if(m_log_event_d !=0)
    m_sbgEventD_pub = m_node->advertise<sbg_driver::SbgEvent>("eventD",10);
  if(m_log_pressure !=0)
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
  m_log_pressure = n_private.param<int>("output/log_pressure", 0);
  m_rate_frequency = n_private.param<int>("output/frequency",0);

  m_magnetic_calibration_mode = n_private.param<int>("magnetometer/calibration/mode",1);
  m_magnetic_calibration_bandwidth = n_private.param<int>("magnetometer/calibration/bandwidth",2);

  std::vector<int> mod_div;
  mod_div.push_back(m_log_status);
  mod_div.push_back(m_log_imu_data);
  mod_div.push_back(m_log_ekf_euler);
  mod_div.push_back(m_log_ekf_quat);
  mod_div.push_back(m_log_ekf_nav);
  mod_div.push_back(m_log_ship_motion);
  mod_div.push_back(m_log_utc_time);
  mod_div.push_back(m_log_mag);
  mod_div.push_back(m_log_mag_calib);
  mod_div.push_back(m_log_gps1_vel);
  mod_div.push_back(m_log_gps1_pos);
  mod_div.push_back(m_log_gps1_hdt);
  mod_div.push_back(m_log_gps1_raw);
  mod_div.push_back(m_log_odo_vel);
  mod_div.push_back(m_log_event_a);
  mod_div.push_back(m_log_event_b);
  mod_div.push_back(m_log_event_c);
  mod_div.push_back(m_log_event_d);
  mod_div.push_back(m_log_pressure);

  if(m_rate_frequency==0){
    int mod_div_min = 3e5;
    for(int i=0; i<mod_div.size(); i++){
      if(mod_div[i]<mod_div_min && mod_div[i]!=0)
        mod_div_min = mod_div[i];
    }

    if(mod_div_min>=10000) // Case Event
      mod_div_min = 8;

    m_rate_frequency = MODE_DIV_2_FREQ[mod_div_min];
  }
}

void Ellipse::publish(){
  sbgEComHandle(&m_comHandle);
  if(m_new_sbgStatus && m_log_status != 0){
    m_new_sbgStatus = false;
    m_sbgStatus_pub.publish(m_sbgStatus_msg);
  }

  if(m_new_sbgUtcTime && m_log_utc_time != 0){
    m_new_sbgUtcTime = false;
    m_sbgUtcTime_pub.publish(m_sbgUtcTime_msg);
  }

  if(m_new_sbgImuData && m_log_imu_data != 0){
    m_new_sbgImuData = false;
    m_sbgImuData_pub.publish(m_sbgImuData_msg);
  }

  if(m_new_sbgEkfEuler && m_log_ekf_euler != 0){
    m_new_sbgEkfEuler = false;
    m_sbgEkfEuler_pub.publish(m_sbgEkfEuler_msg);
  }

  if(m_new_sbgEkfQuat && m_log_ekf_quat != 0){
    m_new_sbgEkfQuat = false;
    m_sbgEkfQuat_pub.publish(m_sbgEkfQuat_msg);
  }

  if(m_new_sbgEkfNav && m_log_ekf_nav != 0){
    m_new_sbgEkfNav = false;
    m_sbgEkfNav_pub.publish(m_sbgEkfNav_msg);
  }

  if(m_new_sbgShipMotion && m_log_ship_motion != 0){
    m_new_sbgShipMotion = false;
    m_sbgShipMotion_pub.publish(m_sbgShipMotion_msg);
  }

  if(m_new_sbgMag && m_log_mag != 0){
    m_new_sbgMag = false;
    m_sbgMag_pub.publish(m_sbgMag_msg);
  }

  if(m_new_sbgMagCalib && m_log_mag_calib != 0){
    m_new_sbgMagCalib = false;
    m_sbgMagCalib_pub.publish(m_sbgMagCalib_msg);
  }

  if(m_new_sbgGpsVel && m_log_gps1_vel != 0){
    m_new_sbgGpsVel = false;
    m_sbgGpsVel_pub.publish(m_sbgGpsVel_msg);
  }

  if(m_new_sbgGpsPos && m_log_gps1_pos != 0){
    m_new_sbgGpsPos = false;
    m_sbgGpsPos_pub.publish(m_sbgGpsPos_msg);
  }

  if(m_new_sbgGpsHdt && m_log_gps1_hdt != 0){
    m_new_sbgGpsHdt = false;
    m_sbgGpsHdt_pub.publish(m_sbgGpsHdt_msg);
  }

  if(m_new_sbgGpsRaw && m_log_gps1_raw != 0){
    m_new_sbgGpsRaw = false;
    m_sbgGpsRaw_pub.publish(m_sbgGpsRaw_msg);
  }

  if(m_new_sbgOdoVel && m_log_odo_vel != 0){
    m_new_sbgOdoVel = false;
    m_sbgOdoVel_pub.publish(m_sbgOdoVel_msg);
  }

  if(m_new_sbgEventA && m_log_event_a != 0){
    m_new_sbgEventA = false;
    m_sbgEventA_pub.publish(m_sbgEventA_msg);
  }

  if(m_new_sbgEventB && m_log_event_b != 0){
    m_new_sbgEventB = false;
    m_sbgEventB_pub.publish(m_sbgEventB_msg);
  }

  if(m_new_sbgEventC && m_log_event_c != 0){
    m_new_sbgEventC = false;
    m_sbgEventC_pub.publish(m_sbgEventC_msg);
  }

  if(m_new_sbgEventD && m_log_event_d != 0){
    m_new_sbgEventD = false;
    m_sbgEventD_pub.publish(m_sbgEventD_msg);
  }

  if(m_new_sbgPressure && m_log_pressure != 0){
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
    read_ecom_log_event(e->m_sbgEventA_msg, pLogData);
    e->m_new_sbgEventA = true;
    break;

  case SBG_ECOM_LOG_EVENT_B:
    read_ecom_log_event(e->m_sbgEventB_msg, pLogData);
    e->m_new_sbgEventB = true;
    break;

  case SBG_ECOM_LOG_EVENT_C:
    read_ecom_log_event(e->m_sbgEventC_msg, pLogData);
    e->m_new_sbgEventC = true;
    break;

  case SBG_ECOM_LOG_EVENT_D:
    read_ecom_log_event(e->m_sbgEventD_msg, pLogData);
    e->m_new_sbgEventD = true;
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

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  ostringstream oss;
  oss << std::put_time(&tm, "mag_calib_%Y_%m_%d-%Hh%Mmin%Ss.txt");

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
