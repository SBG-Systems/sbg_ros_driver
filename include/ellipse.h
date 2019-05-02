#ifndef ELLIPSE_HEADER
#define ELLIPSE_HEADER

#include "ros/ros.h"

extern "C"
{
#include <sbgEComLib.h>
#include <sbgEComIds.h>
#include <sbgErrorCodes.h>
}

#include <iostream>
#include <map>
#include <string>

#include <message_publisher.h>

class Ellipse
{
  private:

    /*!
     *  Callback definition called each time a new log is received.
     * 
     *  \param[in]  pHandle                 Valid handle on the sbgECom instance that has called this callback.
     *  \param[in]  msgClass                Class of the message we have received
     *  \param[in]  msg                     Message ID of the log received.
     *  \param[in]  pLogData                Contains the received log data as an union.
     *  \param[in]  pUserArg                Optional user supplied argument.
     *  \return                             SBG_NO_ERROR if the received log has been used successfully.
     */
    static SbgErrorCode onLogReceivedCallback(SbgEComHandle *pHandle, SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData, void *pUserArg);

    /*!
     * Function to handle the received log.
     * 
     * \param[in]  msgClass               Class of the message we have received
     * \param[in]  msg                    Message ID of the log received.
     * \param[in]  pLogData               Contains the received log data as an union.
     */
    void onLogReceived(SbgEComClass msgClass, SbgEComMsgId msg, const SbgBinaryLogData *pLogData);

  public:

    Ellipse(ros::NodeHandle *n);

    ~Ellipse();

    void init_callback();
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

    /*!
     * Initialize the publishers according to the configuration.
     */
    void initPublishers(void);

    /*!
     * Periodic handle of the connected Ellipse.
     */
    void periodicHandle(void);

    /*!
     * Read the device informations.
     * 
     * \param[in] p_com_handle      SBG communication handle.
     */
    void readDeviceInfo(SbgEComHandle* p_com_handle);

    /*!
     * Get the device frequency.
     * 
     * \return                      Device frequency to read the logs.
     */
    int getDeviceRateFrequency(void) const;

  private:

    ros::NodeHandle *m_node;
    sbg::MessagePublisher m_message_publisher_;

    int m_rate_frequency;

 
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
    int m_log_event_e;
    int m_log_pressure;

    int m_magnetic_calibration_mode;
    int m_magnetic_calibration_bandwidth;
    SbgEComMagCalibResults	m_magCalibResults;
};

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