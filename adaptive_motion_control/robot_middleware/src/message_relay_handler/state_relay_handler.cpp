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

#include "robot_middleware/message_relay_handler/state_relay_handler.hpp"

#include <algorithm>
#include <array>

using namespace industrial::joint_data;
using namespace industrial::joint_feedback_message;
using namespace industrial::joint_message;
using namespace industrial::robot_status_message;
using namespace industrial::simple_message;
using namespace robot_middleware::connection_manager;

namespace robot_middleware
{
namespace message_relay_handler
{

StateRelayHandler::StateRelayHandler(std::shared_ptr<rclcpp::Node> node) : MessageRelayHandler("state_relay_handler", node)
{
  double state_receive_timeout_seconds;
  node_->declare_parameter<double>("state_relay_handler_receive_timeout", 1.0);
  node_->get_parameter<double>("state_relay_handler_receive_timeout", state_receive_timeout_seconds);
  state_receive_timeout = state_receive_timeout_seconds * 1e3;
  RCLCPP_DEBUG(node_->get_logger(), "Initialized %s 'state_receive_timeout': %d ms", getName(), state_receive_timeout);

  receive_timeout_ = state_receive_timeout;
}

StateRelayHandler::~StateRelayHandler()
{
}

void StateRelayHandler::init(const std::shared_ptr<RobotDriver>& driver, const SimpleSocketManagerPtr& in,
                             const SimpleSocketManagerPtr& out)
{
  MessageRelayHandler::init(driver, in, out);
}

bool StateRelayHandler::handleMessage(SimpleMessage& msg, rclcpp::Time& timestamp)
{
  switch (msg.getMessageType())
  {
    case StandardMsgType::JOINT_POSITION:
    {
      JointMessage jnt_msg;
      if (jnt_msg.init(msg))
        handleMessage(jnt_msg, timestamp);
      break;
    }
    case StandardMsgType::JOINT_FEEDBACK:
    {
      JointFeedbackMessage jnt_fbk_msg;
      if (jnt_fbk_msg.init(msg))
        handleMessage(jnt_fbk_msg, timestamp);
      break;
    }
    case StandardMsgType::STATUS:
    {
      RobotStatusMessage status_msg;
      if (status_msg.init(msg))
        handleMessage(status_msg, timestamp);
      break;
    }
    default:
      RCLCPP_DEBUG(node_->get_logger(), "[%s] Received unknown message type: %d", getName(), msg.getMessageType());
      break;
  }

  // relay state messages by default
  return true;
}

void StateRelayHandler::handleMessage(JointMessage& joint_msg, rclcpp::Time& timestamp)
{
  const JointData& jnt_data = joint_msg.getJoints();
  std::array<double, RobotDriver::MAX_NUM_JOINTS> pos;
  int max_num_joints = std::min((int)RobotDriver::MAX_NUM_JOINTS, jnt_data.getMaxNumJoints());
  for (int i = 0; i < max_num_joints; ++i)
    pos[i] = jnt_data.getJoint(i);
  driver_->setJointPositions(pos, timestamp);
}

void StateRelayHandler::handleMessage(JointFeedbackMessage& jnt_fbk_msg, rclcpp::Time& timestamp)
{
  JointData jnt_pos_data, jnt_vel_data;
  std::array<double, RobotDriver::MAX_NUM_JOINTS> pos;
  std::array<double, RobotDriver::MAX_NUM_JOINTS> vel;
  bool has_position = jnt_fbk_msg.getPositions(jnt_pos_data);
  bool has_velocity = jnt_fbk_msg.getVelocities(jnt_vel_data);
  int max_num_joints = std::min((int)RobotDriver::MAX_NUM_JOINTS, jnt_pos_data.getMaxNumJoints());

  // at least position is required!
  if (has_position)
  {
    for (int i = 0; i < max_num_joints; ++i)
      pos[i] = jnt_pos_data.getJoint(i);

    if (has_velocity)
    {
      for (int i = 0; i < max_num_joints; ++i)
        vel[i] = jnt_vel_data.getJoint(i);

      driver_->setJointPositionsAndVelocities(pos, vel, timestamp);
    }
    else
    {
      driver_->setJointPositions(pos, timestamp);
    }

    // #ifndef NDEBUG
    //     std::string jnt_pos_str = std::to_string(pos[0]);
    //     for (int i = 1; i < pos.size(); ++i)
    //       jnt_pos_str += ", " + std::to_string(pos[i]);
    //     ROS_DEBUG_STREAM_NAMED(LOGNAME, "[" << name_ << "] Received joint feedback\n"
    //                                         << "  position: [" << jnt_pos_str << "]\n");
    // #endif
  }
}

void StateRelayHandler::handleMessage(RobotStatusMessage& robot_status_msg, rclcpp::Time& timestamp)
{
  driver_->setRobotStatus(robot_status_msg.status_, timestamp);
}

}  // namespace message_relay_handler
}  // namespace robot_middleware