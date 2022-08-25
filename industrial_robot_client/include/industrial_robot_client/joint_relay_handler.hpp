/*
 *  Copyright (c) 2022 Ivo Dekker ACRO Diepenbeek KULeuven

 Permission is hereby granted, free of charge, to any person
 obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without
 restriction, including without limitation the rights to use,
 copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the
 Software is furnished to do so, subject to the following
 conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "simple_message/message_handler.hpp"
#include "simple_message/messages/joint_message.hpp"


namespace industrial_robot_client
{
namespace joint_relay_handler
{

using industrial::joint_message::JointMessage;
using industrial::simple_message::SimpleMessage;

/**
 * @brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointRelayHandler : public industrial::message_handler::MessageHandler, public rclcpp::Node
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
* @brief Constructor
*/
  JointRelayHandler();
  
 /**
  * @brief Class initializer
  *
  * @param connection simple message connection that will be used to send replies.
  * @param joint_names list of joint-names for msg-publishing.
  *   - Count and order should match data from robot connection.
  *   - Use blank-name to exclude a joint from publishing.
  *
  * @return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, 
           std::vector<std::string> &joint_names);

protected:
  /**
   * @brief Convert joint message into publish message-types
   *
   * @param[in] msg_in Joint message from robot connection
   * @param[out] control_state FollowJointTrajectoryFeedback message for ROS publishing
   * @param[out] sensor_state JointState message for ROS publishing
   *
   * @return true on success, false otherwise
   */
  virtual bool create_messages(industrial::joint_message::JointMessage &msg_in,
                               control_msgs::action::FollowJointTrajectory_FeedbackMessage *control_state,
                               sensor_msgs::msg::JointState *sensor_state);

  /**
   * @brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * @param[in] pos_in joint positions, exactly as passed from robot connection.
   * @param[out] pos_out transformed joint positions (in same order/count as input positions)
   *
   * @return true on success, false otherwise
   */
  virtual bool transform(const std::vector<double>& pos_in, 
                         std::vector<double>* pos_out)
  {
    *pos_out = pos_in;  // by default, no transform is applied
    return true;
  }

  /**
   * @brief Select specific joints for publishing
   *
   * @param[in] all_joint_pos joint positions, in count/order matching robot connection
   * @param[in] all_joint_names joint names, matching all_joint_pos
   * @param[out] pub_joint_pos joint positions selected for publishing
   * @param[out] pub_joint_names joint names selected for publishing
   *
   * @return true on success, false otherwise
   */
  virtual bool select(const std::vector<double>& all_joint_pos, const std::vector<std::string>& all_joint_names,
                      std::vector<double>* pub_joint_pos, std::vector<std::string>* pub_joint_names);

  /**
   * @brief Callback executed upon receiving a joint message
   *
   * @param in incoming message
   *
   * @return true on success, false otherwise
   */
  bool internalCB(industrial::joint_message::JointMessage &in);

  std::vector<std::string> all_joint_names_;

  rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_FeedbackMessage>::SharedPtr pub_joint_control_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_sensor_state_;

private:
 /**
  * @brief Callback executed upon receiving a message
  *
  * @param in incoming message
  *
  * @return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage &in);
 
};//class JointRelayHandler

}//joint_relay_handler
}//industrial_robot_client
