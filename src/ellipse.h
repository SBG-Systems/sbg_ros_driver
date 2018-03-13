#ifndef ELLIPSE_HEADER
#define ELLIPSE_HEADER

#include "ros/ros.h"
#include "ellipse_msg.h"
#include <sbgEComLib.h>
#include <sbgEComIds.h>
#include <sbgErrorCodes.h>

#include <iostream>
#include <map>
#include <string>

class Ellipse
{
  public:

    Ellipse(ros::NodeHandle *n);

    ~Ellipse();

    void init_publishers();
    void init_callback();
    void publish();
    void connect();
    void load_param();
    void configure();
    void save_config();

    bool start_mag_calibration();
    bool end_mag_calibration();
    bool save_mag_calibration();

    bool set_cmd_init_parameters();
    bool set_cmd_motion_profile();
    bool set_cmd_imu_lever_arm();
    bool set_cmd_aiding_assignement();
    bool set_cmd_mag_model();
    bool set_cmd_mag_reject_mode();
    bool set_cmd_gnss_model();
    bool set_cmd_gnss_lever_arm();
    bool set_cmd_gnss_reject_mode();
    bool set_cmd_odom_conf();
    bool set_cmd_odom_lever_arm();
    bool set_cmd_odom_reject_mode();
    bool set_cmd_output();

  private:

    ros::NodeHandle *m_node;

    // *************** Publishers *************** //
  public:
    sbg_driver::SbgStatus m_sbgStatus_msg;
    sbg_driver::SbgUtcTime m_sbgUtcTime_msg;
    sbg_driver::SbgImuData m_sbgImuData_msg;
    sbg_driver::SbgEkfEuler m_sbgEkfEuler_msg;
    sbg_driver::SbgEkfQuat m_sbgEkfQuat_msg;
    sbg_driver::SbgEkfNav m_sbgEkfNav_msg;
    sbg_driver::SbgShipMotion m_sbgShipMotion_msg;
    sbg_driver::SbgMag m_sbgMag_msg;
    sbg_driver::SbgMagCalib m_sbgMagCalib_msg;
    sbg_driver::SbgGpsVel m_sbgGpsVel_msg;
    sbg_driver::SbgGpsPos m_sbgGpsPos_msg;
    sbg_driver::SbgGpsHdt m_sbgGpsHdt_msg;
    sbg_driver::SbgGpsRaw m_sbgGpsRaw_msg;
    sbg_driver::SbgOdoVel m_sbgOdoVel_msg;
    sbg_driver::SbgEvent m_sbgEventA_msg;
    sbg_driver::SbgEvent m_sbgEventB_msg;
    sbg_driver::SbgEvent m_sbgEventC_msg;
    sbg_driver::SbgEvent m_sbgEventD_msg;
    sbg_driver::SbgPressure m_sbgPressure_msg;

  public:
    bool m_new_sbgStatus;
    bool m_new_sbgUtcTime;
    bool m_new_sbgImuData;
    bool m_new_sbgEkfEuler;
    bool m_new_sbgEkfQuat;
    bool m_new_sbgEkfNav;
    bool m_new_sbgShipMotion;
    bool m_new_sbgMag;
    bool m_new_sbgMagCalib;
    bool m_new_sbgGpsVel;
    bool m_new_sbgGpsPos;
    bool m_new_sbgGpsHdt;
    bool m_new_sbgGpsRaw;
    bool m_new_sbgOdoVel;
    bool m_new_sbgEventA;
    bool m_new_sbgEventB;
    bool m_new_sbgEventC;
    bool m_new_sbgEventD;
    bool m_new_sbgPressure;

    int m_rate_frequency;

  private:
    ros::Publisher m_sbgStatus_pub;
    ros::Publisher m_sbgUtcTime_pub;
    ros::Publisher m_sbgImuData_pub;
    ros::Publisher m_sbgEkfEuler_pub;
    ros::Publisher m_sbgEkfQuat_pub;
    ros::Publisher m_sbgEkfNav_pub;
    ros::Publisher m_sbgShipMotion_pub;
    ros::Publisher m_sbgMag_pub;
    ros::Publisher m_sbgMagCalib_pub;
    ros::Publisher m_sbgGpsVel_pub;
    ros::Publisher m_sbgGpsPos_pub;
    ros::Publisher m_sbgGpsHdt_pub;
    ros::Publisher m_sbgGpsRaw_pub;
    ros::Publisher m_sbgOdoVel_pub;
    ros::Publisher m_sbgEventA_pub;
    ros::Publisher m_sbgEventB_pub;
    ros::Publisher m_sbgEventC_pub;
    ros::Publisher m_sbgEventD_pub;
    ros::Publisher m_sbgPressure_pub;

    // *************** SBG TOOLS *************** //
    SbgEComHandle       m_comHandle;
    SbgInterface        m_sbgInterface;

    // *************** SBG Params *************** //
    // connection param
    std::string m_uartPortName;
    uint32 m_uartBaudRate;
    SbgEComOutputPort m_portOutput;

    double m_initLat;
    double m_initLong;
    double m_initAlt;
    uint16 m_initYear;
    uint8 m_initMonth;
    uint8 m_initDay;
    uint32 m_motionProfileId;

    int m_imuAxisDirectionX;
    int m_imuAxisDirectionY;
    float m_imuMisRoll;
    float m_imuMisPitch;
    float m_imuMisYaw;
    float m_imuLeverArm[3];

    int m_gnss1ModulePortAssignment;
    int m_gnss1ModuleSyncAssignment;
    int m_rtcmPortAssignment;
    int m_odometerPinAssignment;

    uint32 m_magModelId;
    int m_magRejectMode;
    float m_magOffset[3];
    float m_magMatrix[3][3];

    uint32 m_gnssModelId;

    float m_gnss1LeverArmX;
    float m_gnss1LeverArmY;
    float m_gnss1LeverArmZ;
    float m_gnss1PitchOffset;
    float m_gnss1YawOffset;
    float m_gnss1AntennaDistance;

    int m_gnss1PosRejectMode;
    int m_gnss1VelRejectMode;
    int m_gnss1HdtRejectMode;

    float m_odomGain;
    uint8 m_odomGainError;
    bool m_odomDirection;
    float m_odomLever[3];
    int m_odomRejectMode;

    uint8 m_timeReference;

    int m_log_status;
    int m_log_imu_data;
    int m_log_ekf_euler;
    int m_log_ekf_quat;
    int m_log_ekf_nav;
    int m_log_ship_motion;
    int m_log_utc_time;
    int m_log_mag;
    int m_log_mag_calib;
    int m_log_gps1_vel;
    int m_log_gps1_pos;
    int m_log_gps1_hdt;
    int m_log_gps1_raw;
    int m_log_odo_vel;
    int m_log_event_a;
    int m_log_event_b;
    int m_log_event_c;
    int m_log_event_d;
    int m_log_pressure;

    int m_magnetic_calibration_mode;
    int m_magnetic_calibration_bandwidth;
    SbgEComMagCalibResults	m_magCalibResults;
};

/*!
     *  Callback definition called each time a new log is received.
     *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
     *  \param[in]  msgClass                Class of the message we have received
     *  \param[in]  msg                   Message ID of the log received.
     *  \param[in]  pLogData                Contains the received log data as an union.
     *  \param[in]  pUserArg                Optional user supplied argument.
     *  \return                       SBG_NO_ERROR if the received log has been used successfully.
     */
    SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

    static std::map<int, int> MODE_DIV_2_FREQ = {{0,0},{1,200},{2,100},{4,50},{5,40},{8,25},{10,20},{20,10},{40,5},{200,1}};

    static std::map<SbgEComMagCalibQuality, std::string> MAG_CALIB_QUAL= {{SBG_ECOM_MAG_CALIB_QUAL_OPTIMAL, "Quality: optimal"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_GOOD, "Quality: good"},
                                                                            {SBG_ECOM_MAG_CALIB_QUAL_POOR, "Quality: poor"}};

    static std::map<SbgEComMagCalibConfidence, std::string> MAG_CALIB_CONF = {{SBG_ECOM_MAG_CALIB_TRUST_HIGH, "Confidence: high"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_MEDIUM, "Confidence: medium"},
                                                                            {SBG_ECOM_MAG_CALIB_TRUST_LOW, "Confidence: low"}};

    static std::map<SbgEComMagCalibMode, std::string> MAG_CALIB_MODE = {{SBG_ECOM_MAG_CALIB_MODE_2D, "Mode 2D"},
                                                                            {SBG_ECOM_MAG_CALIB_MODE_3D, "Mode 3D"}};

    static std::map<SbgEComMagCalibBandwidth, std::string> MAG_CALIB_BW = {{SBG_ECOM_MAG_CALIB_HIGH_BW, "High Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_MEDIUM_BW, "Medium Bandwidth"},
                                                                            {SBG_ECOM_MAG_CALIB_LOW_BW, "Low Bandwidth"}};




#endif
