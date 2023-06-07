/*!
*	\file         message_subscriber.h
*	\author       SBG Systems
*	\date         26/05/2023
*
*	\brief        Manage subscription of messages.
*
*	\section CodeCopyright Copyright Notice
*	MIT License
*
*	Copyright (c) 2023 SBG Systems
*
*	Permission is hereby granted, free of charge, to any person obtaining a copy
*	of this software and associated documentation files (the "Software"), to deal
*	in the Software without restriction, including without limitation the rights
*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*	copies of the Software, and to permit persons to whom the Software is
*	furnished to do so, subject to the following conditions:
*
*	The above copyright notice and this permission notice shall be included in all
*	copies or substantial portions of the Software.
*
*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*	SOFTWARE.
*/

#ifndef SBG_ROS_MESSAGE_SUBSCRIBER_H
#define SBG_ROS_MESSAGE_SUBSCRIBER_H

// Project headers
#include <config_store.h>
#include <mavros_msgs/RTCM.h>

namespace sbg
{
/*!
 * Class to subscribe to all corresponding topics.
 */
class MessageSubscriber: public ros::NodeHandle
{
private:

  ros::Subscriber         m_rtcm_sub_;

  uint32_t                m_max_messages_;
  SbgInterface            *m_sbg_interface_;

  //---------------------------------------------------------------------//
  //- Private methods                                                   -//
  //---------------------------------------------------------------------//
  /*!
   * Handler for subscription to RTCM topic.
   *
   * \param[in] msg             ROS RTCM message.
   */
  void readRosRtcmMessage(const mavros_msgs::RTCM::ConstPtr& msg) const;

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//

  /*!
   * Default constructor.
   *
   * \param[in] sbg_interface   SBG Interface handle.
   */
  MessageSubscriber(SbgInterface *sbg_interface);

  //---------------------------------------------------------------------//
  //- Operations                                                        -//
  //---------------------------------------------------------------------//

  /*!
   * Initialize the subscribers with configuration.
   *
   * \param[in] ref_config_store        Store configuration for the subscribers.
   */
  void initTopicSubscriptions(const ConfigStore &ref_config_store);
};
}

#endif // SBG_ROS_MESSAGE_SUBSCRIBER_H
