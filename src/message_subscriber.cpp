#include "message_subscriber.h"

// ROS headers
#include <ros/ros.h>

// SBG headers
#include <interfaces/sbgInterface.h>

using sbg::MessageSubscriber;

/*!
 * Class to publish all SBG-ROS messages to the corresponding publishers. 
 */
//---------------------------------------------------------------------//
//- Constructor                                                       -//
//---------------------------------------------------------------------//

MessageSubscriber::MessageSubscriber(SbgInterface *sbg_interface): NodeHandle("sbg_subscriber"),
m_max_messages_(10), m_sbg_interface_(sbg_interface)
{
}

//---------------------------------------------------------------------//
//- Private methods                                                   -//
//---------------------------------------------------------------------//

void MessageSubscriber::readRosRtcmMessage(const mavros_msgs::RTCM::ConstPtr& msg) const
{
    assert(m_sbg_interface_);

    auto rtcm_data = msg->data;
    auto error_code = sbgInterfaceWrite(m_sbg_interface_, rtcm_data.data(), rtcm_data.size());
    if (error_code != SBG_NO_ERROR)
    {
        char error_str[256];

        sbgEComErrorToString(error_code, error_str);
        SBG_LOG_ERROR(SBG_ERROR, "Failed to sent RTCM data to device: %s", error_str);
    }

}

//---------------------------------------------------------------------//
//- Operations                                                        -//
//---------------------------------------------------------------------//

void MessageSubscriber::initTopicSubscriptions(const ConfigStore &ref_config_store)
{
    if (ref_config_store.shouldListenRtcm())
    {
        m_rtcm_sub_ = subscribe("/" + ref_config_store.getRtcmFullTopic(), m_max_messages_, &MessageSubscriber::readRosRtcmMessage, this);
    }
}
