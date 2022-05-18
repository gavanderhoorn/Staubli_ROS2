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

#include "robot_middleware/message_relay_handler/motion_relay_handler.hpp"

using namespace industrial::joint_traj_pt_message;
using namespace industrial::joint_traj_pt_full_message;
using namespace industrial::simple_message;

namespace robot_middleware
{
namespace message_relay_handler
{

MotionRelayHandler::MotionRelayHandler(std::shared_ptr<rclcpp::Node> node)
  : MessageRelayHandler("motion_relay_handler", node), is_relaying_(false), ignore_trajectory_(false)
{
  double motion_receive_timeout_seconds;
  node_->declare_parameter<double>("motion_relay_handler_receive_timeout", 0.1);
  node_->get_parameter<double>("motion_relay_handler_receive_timeout", motion_receive_timeout_seconds);
  motion_receive_timeout = motion_receive_timeout_seconds * 1e3;
  RCLCPP_DEBUG(node_->get_logger(), "Initialized %s 'motion_receive_timeout': %d ms", getName(), motion_receive_timeout);

  receive_timeout_ = motion_receive_timeout;
}

MotionRelayHandler::~MotionRelayHandler()
{
}

void MotionRelayHandler::onReceiveTimeout()
{
  if (is_relaying_)
  {
    driver_->setMotionControlMode(MotionControlMode::IDLE);
    is_relaying_ = false;
  }
}

void MotionRelayHandler::onReceiveFail()
{
  if (is_relaying_)
  {
    driver_->setMotionControlMode(MotionControlMode::IDLE);
    is_relaying_ = false;
  }
}

bool MotionRelayHandler::handleMessage(SimpleMessage& msg, rclcpp::Time& timestamp)
{
  // do not relay message by default
  is_relaying_ = false;

  MessageRelayHandler::handleMessage(msg, timestamp);

  switch (msg.getMessageType())
  {
    case StandardMsgType::JOINT_TRAJ_PT:
    {
      JointTrajPtMessage traj_pt_msg;
      if (traj_pt_msg.init(msg))
        is_relaying_ = handleMessage(traj_pt_msg);
      break;
    }
    case StandardMsgType::JOINT_TRAJ_PT_FULL:
    {
      JointTrajPtFullMessage traj_pt_full_msg;
      if (traj_pt_full_msg.init(msg))
        is_relaying_ = handleMessage(traj_pt_full_msg);
      break;
    }
    default:
      RCLCPP_INFO(node_->get_logger(), "[%s] Relaying unhandled message (msg_type: %d)", getName(), msg.getMessageType());
      is_relaying_ = driver_->setMotionControlMode(MotionControlMode::MOTION_COMMAND_RELAYING);
      break;
  }

  return is_relaying_;
}

bool MotionRelayHandler::handleMessage(JointTrajPtMessage& jnt_traj_pt_msg)
{
  auto& traj_pt = jnt_traj_pt_msg.point_;
  RCLCPP_INFO(node_->get_logger(), "[%s] Handling JointTrajPtMessage (sequence: %d)", getName(), traj_pt.getSequence());

  return handleTrajectoryPoint(traj_pt.getSequence());
}

bool MotionRelayHandler::handleMessage(JointTrajPtFullMessage& jnt_traj_pt_full_msg)
{
  auto& traj_pt = jnt_traj_pt_full_msg.point_;
  RCLCPP_INFO(node_->get_logger(), "[%s] Handling JointTrajPtFullMessage (sequence: %d)", getName(), traj_pt.getSequence());

  return handleTrajectoryPoint(traj_pt.getSequence());
}

bool MotionRelayHandler::handleTrajectoryPoint(int sequence)
{
  // ignore the rest of the trajectory if the first point can't be relayed
  if (sequence == 0)
    ignore_trajectory_ = !driver_->setMotionControlMode(MotionControlMode::JOINT_TRAJECTORY_STREAMING);

  return !ignore_trajectory_;
}

}  // namespace message_relay_handler
}  // namespace robot_middleware