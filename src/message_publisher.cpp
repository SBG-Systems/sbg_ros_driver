#include "message_publisher.h"

using sbg::MessagePublisher;

/*!
 * Class to publish all SBG-ROS messages to the corresponding publishers.
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

MessagePublisher::MessagePublisher(void):
m_max_messages_(10)
{
}

//---------------------------------------------------------------------//
//- Private methods                                                   -//
//---------------------------------------------------------------------//

std::string MessagePublisher::getOutputTopicName(SbgEComMsgId sbg_message_id) const
{
  std::string sbg_node_name;
  std::string name;

  sbg_node_name = ros::names::resolve("sbg");

  switch (sbg_message_id)
  {
  case SBG_ECOM_LOG_STATUS:
    name = "/status";
    break;
  case SBG_ECOM_LOG_UTC_TIME:
    name = "/utc_time";
    break;
  case SBG_ECOM_LOG_IMU_DATA:
    name = "/imu_data";
    break;
  case SBG_ECOM_LOG_MAG:
    name = "/mag";
    break;
  case SBG_ECOM_LOG_MAG_CALIB:
    name = "/mag_calib";
    break;
  case SBG_ECOM_LOG_EKF_EULER:
    name = "/ekf_euler";
    break;
  case SBG_ECOM_LOG_EKF_QUAT:
    name = "/ekf_quat";
    break;
  case SBG_ECOM_LOG_EKF_NAV:
    name = "/ekf_nav";
    break;
  case SBG_ECOM_LOG_SHIP_MOTION:
    name = "/ship_motion";
    break;
  case SBG_ECOM_LOG_GPS1_VEL:
    name = "/gps_vel";
    break;
  case SBG_ECOM_LOG_GPS1_POS:
    name = "/gps_pos";
    break;
  case SBG_ECOM_LOG_GPS1_HDT:
    name = "/gps_hdt";
    break;
  case SBG_ECOM_LOG_GPS1_RAW:
    name = "/gps_raw";
    break;
  case SBG_ECOM_LOG_ODO_VEL:
    name = "/odo_vel";
    break;
  case SBG_ECOM_LOG_EVENT_A:
    name = "/eventA";
    break;
  case SBG_ECOM_LOG_EVENT_B:
    name = "/eventB";
    break;
  case SBG_ECOM_LOG_EVENT_C:
    name = "/eventC";
    break;
  case SBG_ECOM_LOG_EVENT_D:
    name = "/eventD";
    break;
  case SBG_ECOM_LOG_EVENT_E:
    name = "/eventE";
    break;
  case SBG_ECOM_LOG_AIR_DATA:
    name = "/air_data";
    break;
  case SBG_ECOM_LOG_IMU_SHORT:
    name = "/imu_short";
    break;
  default:
    return "undefined";
  }

  return sbg_node_name + name;
}

void MessagePublisher::initPublisher(ros::NodeHandle& ref_ros_node_handle, SbgEComMsgId sbg_msg_id, SbgEComOutputMode output_conf, const std::string &ref_output_topic)
{
  //
  // Check if the publisher has to be initialized.
  //
  if (output_conf != SBG_ECOM_OUTPUT_MODE_DISABLED)
  {
    switch (sbg_msg_id)
    {
      case SBG_ECOM_LOG_STATUS:
        m_sbgStatus_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgStatus>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_UTC_TIME:
        m_sbgUtcTime_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgUtcTime>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_IMU_DATA:
        m_sbgImuData_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgImuData>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_MAG:
        m_sbgMag_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgMag>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_MAG_CALIB:
        m_sbgMagCalib_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgMagCalib>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EKF_EULER:
        m_sbgEkfEuler_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEkfEuler>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EKF_QUAT:
        m_sbgEkfQuat_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEkfQuat>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EKF_NAV:

        m_sbgEkfNav_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEkfNav>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_SHIP_MOTION:

        m_sbgShipMotion_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgShipMotion>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_GPS1_VEL:
      case SBG_ECOM_LOG_GPS2_VEL:

        m_sbgGpsVel_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgGpsVel>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_GPS1_POS:
      case SBG_ECOM_LOG_GPS2_POS:

        m_sbgGpsPos_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgGpsPos>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_GPS1_HDT:
      case SBG_ECOM_LOG_GPS2_HDT:

        m_sbgGpsHdt_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgGpsHdt>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_GPS1_RAW:
      case SBG_ECOM_LOG_GPS2_RAW:

        m_sbgGpsRaw_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgGpsRaw>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_ODO_VEL:

        m_sbgOdoVel_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgOdoVel>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EVENT_A:

        m_sbgEventA_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EVENT_B:

        m_sbgEventB_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EVENT_C:

        m_sbgEventC_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EVENT_D:

        m_sbgEventD_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_EVENT_E:

        m_sbgEventE_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgEvent>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_IMU_SHORT:

        m_sbgImuShort_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgImuShort>(ref_output_topic, m_max_messages_);
        break;

      case SBG_ECOM_LOG_AIR_DATA:

        m_sbgAirData_pub_ = ref_ros_node_handle.advertise<sbg_driver::SbgAirData>(ref_output_topic, m_max_messages_);
        break;

      default:
        break;
    }
  }
}

void MessagePublisher::defineRosStandardPublishers(ros::NodeHandle& ref_ros_node_handle, bool odom_enable)
{
  std::string imu_node_name;

  imu_node_name = ros::names::resolve("imu");

  if (m_sbgImuData_pub_ && m_sbgEkfQuat_pub_)
  {
    m_imu_pub_ = ref_ros_node_handle.advertise<sensor_msgs::Imu>(imu_node_name + "/data", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Imu and/or Quat output are not configured, the standard IMU can not be defined.");
  }

  if (m_sbgImuData_pub_)
  {
    m_temp_pub_     = ref_ros_node_handle.advertise<sensor_msgs::Temperature>(imu_node_name + "/temp", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Imu data output are not configured, the standard Temperature publisher can not be defined.");
  }

  if (m_sbgMag_pub_)
  {
    m_mag_pub_ = ref_ros_node_handle.advertise<sensor_msgs::MagneticField>(imu_node_name + "/mag", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Mag data output are not configured, the standard Magnetic publisher can not be defined.");
  }

  //
  // We need either Euler or quat angles, and we must have Nav and IMU data to
  // compute Body and angular velocity.
  //
  if ((m_sbgEkfEuler_pub_ || m_sbgEkfQuat_pub_) && m_sbgEkfNav_pub_ && m_sbgImuData_pub_)
  {
    m_velocity_pub_ = ref_ros_node_handle.advertise<geometry_msgs::TwistStamped>(imu_node_name + "/velocity", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Imu, Nav or Angles data outputs are not configured, the standard Velocity publisher can not be defined.");
  }

  if (m_sbgAirData_pub_)
  {
    m_fluid_pub_ = ref_ros_node_handle.advertise<sensor_msgs::FluidPressure>(imu_node_name + "/pres", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG AirData output are not configured, the standard FluidPressure publisher can not be defined.");
  }

  if (m_sbgEkfNav_pub_)
  {
    m_pos_ecef_pub_ = ref_ros_node_handle.advertise<geometry_msgs::PointStamped>(imu_node_name + "/pos_ecef", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Ekf data output are not configured, the standard ECEF position publisher can not be defined.");
  }

  if (m_sbgUtcTime_pub_)
  {
    m_utc_reference_pub_ = ref_ros_node_handle.advertise<sensor_msgs::TimeReference>(imu_node_name + "/utc_ref", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG Utc data output are not configured, the UTC time reference publisher can not be defined.");
  }

  if (m_sbgGpsPos_pub_)
  {
    m_nav_sat_fix_pub_ = ref_ros_node_handle.advertise<sensor_msgs::NavSatFix>(imu_node_name + "/nav_sat_fix", m_max_messages_);
  }
  else
  {
    ROS_WARN("SBG_DRIVER - [Publisher] SBG GPS Pos data output are not configured, the NavSatFix publisher can not be defined.");
  }

  if (odom_enable)
  {
    if (m_sbgImuData_pub_ && m_sbgEkfNav_pub_ && (m_sbgEkfEuler_pub_ || m_sbgEkfQuat_pub_))
    {
      m_odometry_pub_ = ref_ros_node_handle.advertise<nav_msgs::Odometry>(imu_node_name + "/odometry", m_max_messages_);
    }
    else
    {
      ROS_WARN("SBG_DRIVER - [Publisher] SBG IMU, NAV and Quaternion (or Euler) outputs are not configured, the odometry publisher can not be defined.");
    }
  }
}

void MessagePublisher::publishIMUData(const SbgBinaryLogData &ref_sbg_log)
{
  if (m_sbgImuData_pub_)
  {
    m_sbg_imu_message_ = m_message_wrapper_.createSbgImuDataMessage(ref_sbg_log.imuData);
    m_sbgImuData_pub_.publish(m_sbg_imu_message_);
  }
  if (m_temp_pub_)
  {
    m_temp_pub_.publish(m_message_wrapper_.createRosTemperatureMessage(m_sbg_imu_message_));
  }

  processRosImuMessage();
  processRosVelMessage();
  processRosOdoMessage();
}

void MessagePublisher::processRosVelMessage(void)
{
  if (m_velocity_pub_)
  {
    if (m_sbgEkfQuat_pub_)
    {
      m_velocity_pub_.publish(m_message_wrapper_.createRosTwistStampedMessage(m_sbg_ekf_quat_message_, m_sbg_ekf_nav_message_, m_sbg_imu_message_));
    }
    else if (m_sbgEkfEuler_pub_)
    {
      m_velocity_pub_.publish(m_message_wrapper_.createRosTwistStampedMessage(m_sbg_ekf_euler_message_, m_sbg_ekf_nav_message_, m_sbg_imu_message_));
    }
  }
}

void MessagePublisher::processRosImuMessage(void)
{
  if (m_imu_pub_)
  {
    if (m_sbg_imu_message_.time_stamp == m_sbg_ekf_quat_message_.time_stamp)
    {
      m_imu_pub_.publish(m_message_wrapper_.createRosImuMessage(m_sbg_imu_message_, m_sbg_ekf_quat_message_));
    }
  }
}

void MessagePublisher::processRosOdoMessage(void)
{
  if (m_odometry_pub_)
  {
    if (m_sbg_ekf_nav_message_.status.position_valid)
    {
      if (m_sbg_imu_message_.time_stamp == m_sbg_ekf_nav_message_.time_stamp)
      {
        /*
         * Odometry message can be generated from quaternion or euler angles.
         * Quaternion is prefered if they are available.
         */
        if (m_sbgEkfQuat_pub_)
        {
          if (m_sbg_imu_message_.time_stamp == m_sbg_ekf_quat_message_.time_stamp)
          {
            m_odometry_pub_.publish(m_message_wrapper_.createRosOdoMessage(m_sbg_imu_message_, m_sbg_ekf_nav_message_, m_sbg_ekf_quat_message_, m_sbg_ekf_euler_message_));
          }
        }
        else
        {
          if (m_sbg_imu_message_.time_stamp == m_sbg_ekf_euler_message_.time_stamp)
          {
            m_odometry_pub_.publish(m_message_wrapper_.createRosOdoMessage(m_sbg_imu_message_, m_sbg_ekf_nav_message_, m_sbg_ekf_euler_message_));
          }
        }
      }
    }
  }
}

void MessagePublisher::publishMagData(const SbgBinaryLogData &ref_sbg_log)
{
  sbg_driver::SbgMag sbg_mag_message;
  sbg_mag_message = m_message_wrapper_.createSbgMagMessage(ref_sbg_log.magData);

  if (m_sbgMag_pub_)
  {
    m_sbgMag_pub_.publish(sbg_mag_message);
  }
  if (m_mag_pub_)
  {
    m_mag_pub_.publish(m_message_wrapper_.createRosMagneticMessage(sbg_mag_message));
  }
}

void MessagePublisher::publishFluidPressureData(const SbgBinaryLogData &ref_sbg_log)
{
  sbg_driver::SbgAirData sbg_air_data_message;
  sbg_air_data_message = m_message_wrapper_.createSbgAirDataMessage(ref_sbg_log.airData);

  if (m_sbgAirData_pub_)
  {
    m_sbgAirData_pub_.publish(sbg_air_data_message);
  }
  if (m_fluid_pub_)
  {
    m_fluid_pub_.publish(m_message_wrapper_.createRosFluidPressureMessage(sbg_air_data_message));
  }
}

void MessagePublisher::publishEkfNavigationData(const SbgBinaryLogData &ref_sbg_log)
{
  m_sbg_ekf_nav_message_ = m_message_wrapper_.createSbgEkfNavMessage(ref_sbg_log.ekfNavData);

  if (m_sbgEkfNav_pub_)
  {
    m_sbgEkfNav_pub_.publish(m_sbg_ekf_nav_message_);
  }
  if (m_pos_ecef_pub_)
  {
    m_pos_ecef_pub_.publish(m_message_wrapper_.createRosPointStampedMessage(m_sbg_ekf_nav_message_));
  }
  processRosVelMessage();
}

void MessagePublisher::publishUtcData(const SbgBinaryLogData &ref_sbg_log)
{
  sbg_driver::SbgUtcTime sbg_utc_message;

  sbg_utc_message = m_message_wrapper_.createSbgUtcTimeMessage(ref_sbg_log.utcData);

  if (m_sbgUtcTime_pub_)
  {
    m_sbgUtcTime_pub_.publish(sbg_utc_message);
  }
  if (m_utc_reference_pub_)
  {
    if (sbg_utc_message.clock_status.clock_utc_status != SBG_ECOM_UTC_INVALID)
    {
      m_utc_reference_pub_.publish(m_message_wrapper_.createRosUtcTimeReferenceMessage(sbg_utc_message));
    }
  }
}

void MessagePublisher::publishGpsPosData(const SbgBinaryLogData &ref_sbg_log, SbgEComMsgId sbg_msg_id)
{
  sbg_driver::SbgGpsPos sbg_gps_pos_message;

  sbg_gps_pos_message = m_message_wrapper_.createSbgGpsPosMessage(ref_sbg_log.gpsPosData);

  if (m_sbgGpsPos_pub_)
  {
    m_sbgGpsPos_pub_.publish(sbg_gps_pos_message);
  }
  if (m_nav_sat_fix_pub_)
  {
    m_nav_sat_fix_pub_.publish(m_message_wrapper_.createRosNavSatFixMessage(sbg_gps_pos_message));
  }
  if (m_sbgGpsPos_gga_pub_ && sbg_msg_id == SBG_ECOM_LOG_GPS1_POS)
  {
    m_sbgGpsPos_gga_pub_.publish(m_message_wrapper_.createSbgGpsPosMessageGGA(ref_sbg_log.gpsPosData));
  }
}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void MessagePublisher::initPublishers(ros::NodeHandle& ref_ros_node_handle, const ConfigStore &ref_config_store)
{
  //
  // Initialize all the publishers with the defined SBG output from the config store.
  //
  const std::vector<ConfigStore::SbgLogOutput> &ref_output_modes = ref_config_store.getOutputModes();

  m_message_wrapper_.setTimeReference(ref_config_store.getTimeReference());

  m_message_wrapper_.setFrameId(ref_config_store.getFrameId());

  m_message_wrapper_.setUseEnu(ref_config_store.getUseEnu());

  m_message_wrapper_.setOdomEnable(ref_config_store.getOdomEnable());
  m_message_wrapper_.setOdomPublishTf(ref_config_store.getOdomPublishTf());
  m_message_wrapper_.setOdomFrameId(ref_config_store.getOdomFrameId());
  m_message_wrapper_.setOdomBaseFrameId(ref_config_store.getOdomBaseFrameId());
  m_message_wrapper_.setOdomInitFrameId(ref_config_store.getOdomInitFrameId());

  for (const ConfigStore::SbgLogOutput &ref_output : ref_output_modes)
  {
    initPublisher(ref_ros_node_handle, ref_output.message_id, ref_output.output_mode, getOutputTopicName(ref_output.message_id));
  }

  if (ref_config_store.shouldPublishNmea())
  {
    m_sbgGpsPos_gga_pub_ = ref_ros_node_handle.advertise<nmea_msgs::Sentence>(ref_config_store.getNmeaFullTopic(),m_max_messages_);
  }

  if (ref_config_store.checkRosStandardMessages())
  {
    defineRosStandardPublishers(ref_ros_node_handle, ref_config_store.getOdomEnable());
  }
}

void MessagePublisher::publish(SbgEComClass sbg_msg_class, SbgEComMsgId sbg_msg_id, const SbgBinaryLogData &ref_sbg_log)
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

      publishUtcData(ref_sbg_log);
      break;

    case SBG_ECOM_LOG_IMU_DATA:

      publishIMUData(ref_sbg_log);
      break;

    case SBG_ECOM_LOG_MAG:

      publishMagData(ref_sbg_log);
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
        m_sbg_ekf_euler_message_ = m_message_wrapper_.createSbgEkfEulerMessage(ref_sbg_log.ekfEulerData);
        m_sbgEkfEuler_pub_.publish(m_sbg_ekf_euler_message_);
        processRosVelMessage();
        processRosOdoMessage();
      }
      break;

    case SBG_ECOM_LOG_EKF_QUAT:

      if (m_sbgEkfQuat_pub_)
      {
        m_sbg_ekf_quat_message_ = m_message_wrapper_.createSbgEkfQuatMessage(ref_sbg_log.ekfQuatData);
        m_sbgEkfQuat_pub_.publish(m_sbg_ekf_quat_message_);
        processRosImuMessage();
        processRosVelMessage();
      }
      break;

    case SBG_ECOM_LOG_EKF_NAV:

      publishEkfNavigationData(ref_sbg_log);
      processRosOdoMessage();
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

      publishGpsPosData(ref_sbg_log, sbg_msg_id);
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

    case SBG_ECOM_LOG_IMU_SHORT:

      if (m_sbgImuShort_pub_)
      {
        m_sbgImuShort_pub_.publish(m_message_wrapper_.createSbgImuShortMessage(ref_sbg_log.imuShort));
      }
      break;

    case SBG_ECOM_LOG_AIR_DATA:

      publishFluidPressureData(ref_sbg_log);
      break;

    default:
      break;
    }
  }
  else if (sbg_msg_class == SBG_ECOM_CLASS_LOG_ECOM_1)
  {
    switch (sbg_msg_id)
    {
    default:
      break;
    }
  }
}
