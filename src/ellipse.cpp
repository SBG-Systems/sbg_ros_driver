#include "ellipse.h"
#include "ellipse_msg.h"

Ellipse::Ellipse(ros::NodeHandle *n){
  m_node = n;
  m_uart_port = "/dev/ttyUSB0";
  m_uart_baud_rate = 115200;
}

Ellipse::~Ellipse(){

}

void Ellipse::connect(){
  SbgErrorCode errorCode;

  // Set the parameters of the Interface (port, baud_rate)
  errorCode = sbgInterfaceSerialCreate(&m_sbgInterface, m_uart_port.c_str(), m_uart_baud_rate);
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

