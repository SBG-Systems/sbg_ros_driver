#include "message_publisher.h"

using sbg::MessagePublisher;

/*!
 * Class to publish all SBG-ROS messages to the corresponding publishers. 
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

MessagePublisher::MessagePublisher(void):
m_max_mesages_(10),
m_output_mode_(SBG_ECOM_OUTPUT_MODE_DISABLED)
{

}

//---------------------------------------------------------------------//
//- Private methods                                                   -//
//---------------------------------------------------------------------//

void MessagePublisher::updateOutputConfiguration(SbgEComOutputMode output_conf)
{
  //
  // Update the sbg output configuration if needed.
  // Always get the minimal output configuration (Highest frequency).
  //
  if (m_output_mode_ == SBG_ECOM_OUTPUT_MODE_DISABLED)
  {
    m_output_mode_ = output_conf;
  }
  else
  {
    m_output_mode_ = sbgMin(m_output_mode_, output_conf);
  }

  //
  // In case of sbg output event configuration, just define the output on a 25Hz frequency.
  //
  if (m_output_mode_ >= SBG_ECOM_OUTPUT_MODE_PPS)
  {
    m_output_mode_ = SBG_ECOM_OUTPUT_MODE_DIV_8;
  }
}

void MessagePublisher::initPublisher(ros::NodeHandle *p_ros_node_handle, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_conf, const std::string &ref_output_topic)
{
  //
  // Check if the publisher has to be initialized.
  //
  if (output_conf != SBG_ECOM_OUTPUT_MODE_DISABLED)
  {
    updateOutputConfiguration(output_conf);

    switch (sbg_msg_id)
    {
      case SBG_ECOM_LOG_STATUS:
        m_sbgStatus_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgStatus>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_UTC_TIME:
        m_sbgUtcTime_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgUtcTime>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_IMU_DATA:
        m_sbgImuData_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgImuData>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_MAG:
        m_sbgMag_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgMag>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_MAG_CALIB:
        m_sbgMagCalib_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgMagCalib>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EKF_EULER:
        m_sbgEkfEuler_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEkfEuler>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EKF_QUAT:
        m_sbgEkfQuat_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEkfQuat>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EKF_NAV:

        m_sbgEkfNav_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEkfNav>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_SHIP_MOTION:

        m_sbgShipMotion_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgShipMotion>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_GPS1_VEL:
      case SBG_ECOM_LOG_GPS2_VEL:

        m_sbgGpsVel_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgGpsVel>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_GPS1_POS:
      case SBG_ECOM_LOG_GPS2_POS:

        m_sbgGpsPos_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgGpsPos>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_GPS1_HDT:
      case SBG_ECOM_LOG_GPS2_HDT:

        m_sbgGpsHdt_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgGpsHdt>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_GPS1_RAW:
      case SBG_ECOM_LOG_GPS2_RAW:

        m_sbgGpsRaw_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgGpsRaw>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_ODO_VEL:

        m_sbgOdoVel_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgOdoVel>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EVENT_A:

        m_sbgEventA_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EVENT_B:

        m_sbgEventB_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EVENT_C:

        m_sbgEventC_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EVENT_D:

        m_sbgEventD_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_EVENT_E:

        m_sbgEventE_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_DVL_BOTTOM_TRACK:
      case SBG_ECOM_LOG_DVL_WATER_TRACK:
      case SBG_ECOM_LOG_SHIP_MOTION_HP:

        break;

      case SBG_ECOM_LOG_PRESSURE:

        m_sbgPressure_pub_ = p_ros_node_handle->advertise<sbg_driver::SbgPressure>(ref_output_topic, m_max_mesages_);
        break;

      case SBG_ECOM_LOG_USBL:
      case SBG_ECOM_LOG_DEBUG_0:
      case SBG_ECOM_LOG_IMU_RAW_DATA:
      case SBG_ECOM_LOG_DEBUG_1:
      case SBG_ECOM_LOG_DEBUG_2:
      case SBG_ECOM_LOG_DEBUG_3:
      case SBG_ECOM_LOG_IMU_SHORT:
      case SBG_ECOM_LOG_ECOM_NUM_MESSAGES:

        break;
    }
  }
}

//---------------------------------------------------------------------//
//- Parameters                                                        -//
//---------------------------------------------------------------------//

int MessagePublisher::getOutputFrequency(void) const
{
  switch (m_output_mode_)
  {
  case SBG_ECOM_OUTPUT_MODE_DISABLED:
    return 0;
  
  case SBG_ECOM_OUTPUT_MODE_MAIN_LOOP:
    return 200;

  case SBG_ECOM_OUTPUT_MODE_DIV_2:
    return 100;

  case SBG_ECOM_OUTPUT_MODE_DIV_4:
    return 50;

  case SBG_ECOM_OUTPUT_MODE_DIV_5:
    return 40;

  case SBG_ECOM_OUTPUT_MODE_DIV_8:
    return 25;

  case SBG_ECOM_OUTPUT_MODE_DIV_10:
    return 20;

  case SBG_ECOM_OUTPUT_MODE_DIV_20:
    return 10;

  case SBG_ECOM_OUTPUT_MODE_DIV_40:
    return 5;

  case SBG_ECOM_OUTPUT_MODE_DIV_200:
    return 1;

  case SBG_ECOM_OUTPUT_MODE_HIGH_FREQ_LOOP:
    return 1000;

  default:
    return 0;
  }
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void MessagePublisher::initPublishers(ros::NodeHandle *p_ros_node_handle, const ConfigOutput &ref_output_config)
{
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_STATUS, ref_output_config.getOutputMode(SBG_ECOM_LOG_STATUS), "status");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_UTC_TIME, ref_output_config.getOutputMode(SBG_ECOM_LOG_UTC_TIME), "utc_time");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_IMU_DATA, ref_output_config.getOutputMode(SBG_ECOM_LOG_IMU_DATA), "imu_data");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_MAG, ref_output_config.getOutputMode(SBG_ECOM_LOG_MAG), "mag");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_MAG_CALIB, ref_output_config.getOutputMode(SBG_ECOM_LOG_MAG_CALIB), "mag_calib");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EKF_EULER, ref_output_config.getOutputMode(SBG_ECOM_LOG_EKF_EULER), "ekf_euler");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EKF_QUAT, ref_output_config.getOutputMode(SBG_ECOM_LOG_EKF_QUAT), "ekf_quat");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EKF_NAV, ref_output_config.getOutputMode(SBG_ECOM_LOG_EKF_NAV), "ekf_nav");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_SHIP_MOTION, ref_output_config.getOutputMode(SBG_ECOM_LOG_SHIP_MOTION), "ship_motion");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_GPS1_VEL, ref_output_config.getOutputMode(SBG_ECOM_LOG_GPS1_VEL), "gps_vel");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_GPS1_POS, ref_output_config.getOutputMode(SBG_ECOM_LOG_GPS1_POS), "gps_pos");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_GPS1_HDT, ref_output_config.getOutputMode(SBG_ECOM_LOG_GPS1_HDT), "gps_hdt");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_GPS1_RAW, ref_output_config.getOutputMode(SBG_ECOM_LOG_GPS1_RAW), "gps_raw");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_ODO_VEL, ref_output_config.getOutputMode(SBG_ECOM_LOG_ODO_VEL), "odo_vel");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EVENT_A, ref_output_config.getOutputMode(SBG_ECOM_LOG_EVENT_A), "eventA");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EVENT_B, ref_output_config.getOutputMode(SBG_ECOM_LOG_EVENT_B), "eventB");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EVENT_C, ref_output_config.getOutputMode(SBG_ECOM_LOG_EVENT_C), "eventC");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EVENT_D, ref_output_config.getOutputMode(SBG_ECOM_LOG_EVENT_D), "eventD");
  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_EVENT_E, ref_output_config.getOutputMode(SBG_ECOM_LOG_EVENT_E), "eventE");

  initPublisher(p_ros_node_handle, SBG_ECOM_LOG_PRESSURE, ref_output_config.getOutputMode(SBG_ECOM_LOG_PRESSURE), "pressure");
}

void MessagePublisher::publish(SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, const SbgBinaryLogData &ref_sbg_log) const
{
  //
  // Publish the message with the corresponding publisher and SBG message ID.
  // For each log, check if the publisher has been initialized.
  //
  if(sbg_msg_class == SBG_ECOM_CLASS_LOG_ECOM_0)
  {
    switch (sbg_msg_id)
    {
    case SBG_ECOM_LOG_STATUS:

      if (m_sbgStatus_pub_)
      {
        m_sbgStatus_pub_.publish(m_message_wrapper_.createSbgStatusMessage(ref_sbg_log.statusData));
      }
      break;

    case SBG_ECOM_LOG_UTC_TIME:

      if (m_sbgUtcTime_pub_)
      {
        m_sbgUtcTime_pub_.publish(m_message_wrapper_.createSbgUtcTimeMessage(ref_sbg_log.utcData));
      }
      break;

    case SBG_ECOM_LOG_IMU_DATA:

      if (m_sbgImuData_pub_)
      {
        m_sbgImuData_pub_.publish(m_message_wrapper_.createSbgImuDataMessage(ref_sbg_log.imuData));
      }
      break;

    case SBG_ECOM_LOG_MAG:

      if (m_sbgMag_pub_)
      {
        m_sbgMag_pub_.publish(m_message_wrapper_.createSbgMagMessage(ref_sbg_log.magData));
      }
      break;

    case SBG_ECOM_LOG_MAG_CALIB:

      if (m_sbgMagCalib_pub_)
      {
        m_sbgMagCalib_pub_.publish(m_message_wrapper_.createSbgMagCalibMessage(ref_sbg_log.magCalibData));
      }
      break;

    case SBG_ECOM_LOG_EKF_EULER:

      if (m_sbgEkfEuler_pub_)
      {
        m_sbgEkfEuler_pub_.publish(m_message_wrapper_.createSbgEkfEulerMessage(ref_sbg_log.ekfEulerData));
      }
      break;

    case SBG_ECOM_LOG_EKF_QUAT:

      if (m_sbgEkfQuat_pub_)
      {
        m_sbgEkfQuat_pub_.publish(m_message_wrapper_.createSbgEkfQuatMessage(ref_sbg_log.ekfQuatData));
      }
      break;

    case SBG_ECOM_LOG_EKF_NAV:

      if (m_sbgEkfNav_pub_)
      {
        m_sbgEkfNav_pub_.publish(m_message_wrapper_.createSbgEkfNavMessage(ref_sbg_log.ekfNavData));
      }
      break;

    case SBG_ECOM_LOG_SHIP_MOTION:

      if (m_sbgShipMotion_pub_)
      {
        m_sbgShipMotion_pub_.publish(m_message_wrapper_.createSbgShipMotionMessage(ref_sbg_log.shipMotionData));
      }
      break;

    case SBG_ECOM_LOG_GPS1_VEL:
    case SBG_ECOM_LOG_GPS2_VEL:

      if (m_sbgGpsVel_pub_)
      {
        m_sbgGpsVel_pub_.publish(m_message_wrapper_.createSbgGpsVelMessage(ref_sbg_log.gpsVelData));
      }
      break;

    case SBG_ECOM_LOG_GPS1_POS:
    case SBG_ECOM_LOG_GPS2_POS:

      if (m_sbgGpsPos_pub_)
      {
        m_sbgGpsPos_pub_.publish(m_message_wrapper_.createSbgGpsPosMessage(ref_sbg_log.gpsPosData));
      }
      break;

    case SBG_ECOM_LOG_GPS1_HDT:
    case SBG_ECOM_LOG_GPS2_HDT:

      if (m_sbgGpsHdt_pub_)
      {
        m_sbgGpsHdt_pub_.publish(m_message_wrapper_.createSbgGpsHdtMessage(ref_sbg_log.gpsHdtData));
      }
      break;

    case SBG_ECOM_LOG_GPS1_RAW:
    case SBG_ECOM_LOG_GPS2_RAW:

      if (m_sbgGpsRaw_pub_)
      {
        m_sbgGpsRaw_pub_.publish(m_message_wrapper_.createSbgGpsRawMessage(ref_sbg_log.gpsRawData));
      }
      break;

    case SBG_ECOM_LOG_ODO_VEL:

      if (m_sbgOdoVel_pub_)
      {
        m_sbgOdoVel_pub_.publish(m_message_wrapper_.createSbgOdoVelMessage(ref_sbg_log.odometerData));
      }
      break;

    case SBG_ECOM_LOG_EVENT_A:

      if (m_sbgEventA_pub_)
      {
        m_sbgEventA_pub_.publish(m_message_wrapper_.createSbgEventMessage(ref_sbg_log.eventMarker));
      }
      break;

    case SBG_ECOM_LOG_EVENT_B:

      if (m_sbgEventB_pub_)
      {
        m_sbgEventB_pub_.publish(m_message_wrapper_.createSbgEventMessage(ref_sbg_log.eventMarker));
      }
      break;

    case SBG_ECOM_LOG_EVENT_C:

      if (m_sbgEventC_pub_)
      {
        m_sbgEventC_pub_.publish(m_message_wrapper_.createSbgEventMessage(ref_sbg_log.eventMarker));
      }
      break;

    case SBG_ECOM_LOG_EVENT_D:

      if (m_sbgEventD_pub_)
      {
        m_sbgEventD_pub_.publish(m_message_wrapper_.createSbgEventMessage(ref_sbg_log.eventMarker));
      }
      break;

    case SBG_ECOM_LOG_EVENT_E:

      if (m_sbgEventE_pub_)
      {
        m_sbgEventE_pub_.publish(m_message_wrapper_.createSbgEventMessage(ref_sbg_log.eventMarker));
      }
      break;

    case SBG_ECOM_LOG_DVL_BOTTOM_TRACK:
    case SBG_ECOM_LOG_DVL_WATER_TRACK:
    case SBG_ECOM_LOG_SHIP_MOTION_HP:
      break;

    case SBG_ECOM_LOG_PRESSURE:

      if (m_sbgPressure_pub_)
      {
        m_sbgPressure_pub_.publish(m_message_wrapper_.createSbgPressureMessage(ref_sbg_log.pressureData));
      }
      break;

    case SBG_ECOM_LOG_USBL:
    case SBG_ECOM_LOG_DEBUG_0:
    case SBG_ECOM_LOG_IMU_RAW_DATA:
    case SBG_ECOM_LOG_DEBUG_1:
    case SBG_ECOM_LOG_DEBUG_2:
    case SBG_ECOM_LOG_DEBUG_3:
    case SBG_ECOM_LOG_IMU_SHORT:
    case SBG_ECOM_LOG_ECOM_NUM_MESSAGES:

      break;
    } 
  }
  else if (sbg_msg_class == SBG_ECOM_CLASS_LOG_ECOM_1)
  {
    switch (sbg_msg_id)
    {
      case SBG_ECOM_LOG_FAST_IMU_DATA:
      break;
    }
  }
}