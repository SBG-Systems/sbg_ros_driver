#ifndef ELLIPSE_HEADER
#define ELLIPSE_HEADER

#include "ros/ros.h"
#include "ellipse_msg.h"
#include <sbgEComLib.h>
#include <sbgEComIds.h>
#include <sbgErrorCodes.h>

class Ellipse
{
  public:

    Ellipse(ros::NodeHandle *n);

    ~Ellipse();

    void init_publishers();

    void init_callback();

    void publish();

    void connect();
    
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
    sbg_driver::SbgEvent m_sbgEvent_msg;
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
    bool m_new_sbgEvent;
    bool m_new_sbgPressure;

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
    ros::Publisher m_sbgEvent_pub;
    ros::Publisher m_sbgPressure_pub;

    // *************** SBG TOOLS *************** //
    SbgEComHandle       m_comHandle;
    SbgInterface        m_sbgInterface;

    // *************** SBG Params *************** //
    // connection param
    std::string m_uart_port;
    int m_uart_baud_rate;
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

#endif