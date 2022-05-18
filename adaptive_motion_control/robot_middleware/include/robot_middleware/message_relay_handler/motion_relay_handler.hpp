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

#include "rclcpp/rclcpp.hpp"
#include "simple_message/messages/joint_traj_pt_full_message.hpp"
#include "simple_message/messages/joint_traj_pt_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/typed_message.hpp"

namespace robot_middleware
{
namespace message_relay_handler
{

class MotionRelayHandler : public MessageRelayHandler
{
public:
  MotionRelayHandler(std::shared_ptr<rclcpp::Node> node);

  ~MotionRelayHandler();

protected:
  void onReceiveTimeout() override;

  void onReceiveFail() override;

  bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp) override;

private:
  bool handleMessage(industrial::joint_traj_pt_message::JointTrajPtMessage& jnt_traj_pt_msg);

  bool handleMessage(industrial::joint_traj_pt_full_message::JointTrajPtFullMessage& jnt_traj_pt_full_msg);

  bool handleTrajectoryPoint(int sequence);

  bool is_relaying_;
  bool ignore_trajectory_;
  double motion_receive_timeout;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware