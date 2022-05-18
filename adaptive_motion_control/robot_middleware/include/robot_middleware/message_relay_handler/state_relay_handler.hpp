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

#include "robot_middleware/message_relay_handler/message_relay_handler.hpp"

#include "moveit/robot_state/robot_state.h"
#include "simple_message/messages/joint_feedback_message.hpp"
#include "simple_message/messages/joint_message.hpp"
#include "simple_message/messages/robot_status_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/typed_message.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace robot_middleware
{
namespace message_relay_handler
{

class StateRelayHandler : public MessageRelayHandler
{
public:
  StateRelayHandler(std::shared_ptr<rclcpp::Node> node);

  ~StateRelayHandler();

  void init(const std::shared_ptr<RobotDriver>& driver,
            const robot_middleware::connection_manager::SimpleSocketManagerPtr& in,
            const robot_middleware::connection_manager::SimpleSocketManagerPtr& out) override;

protected:
  bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp) override;

  void handleMessage(industrial::joint_message::JointMessage& joint_msg, rclcpp::Time& timestamp);

  void handleMessage(industrial::joint_feedback_message::JointFeedbackMessage& joint_fbk_msg, rclcpp::Time& timestamp);

  void handleMessage(industrial::robot_status_message::RobotStatusMessage& robot_status_msg, rclcpp::Time& timestamp);

  double state_receive_timeout;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware