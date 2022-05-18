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

#include "industrial_robot_client/robot_status_relay_handler.hpp"

using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace industrial::robot_status;
using namespace industrial::robot_status_message;

namespace industrial_robot_client
{
namespace robot_status_relay_handler
{

RobotStatusRelayHandler::RobotStatusRelayHandler() : Node("robot_status_relay_handler")
{
}

bool RobotStatusRelayHandler::init(industrial::smpl_msg_connection::SmplMsgConnection *connection)
{
  pub_robot_status_ = this->create_publisher<industrial_msgs::msg::RobotStatus>("robot_status", 1);
  return init((int)industrial::simple_message::StandardMsgTypes::STATUS, connection);
}

bool RobotStatusRelayHandler::internalCB(industrial::simple_message::SimpleMessage &in)
{
  industrial::robot_status_message::RobotStatusMessage status_msg;

  if (!status_msg.init(in))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize status message");
    return false;
  }

  return internalCB(status_msg);
}

bool RobotStatusRelayHandler::internalCB(industrial::robot_status_message::RobotStatusMessage & in)
{
  industrial_msgs::msg::RobotStatus status;
  bool rtn = true;

  status.drives_powered.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getDrivesPowered());
  status.e_stopped.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getEStopped());
  status.error_code = in.status_.getErrorCode();
  status.in_error.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInError());
  status.in_motion.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getInMotion());
  status.mode.val = industrial::robot_status::RobotModes::toROSMsgEnum(in.status_.getMode());
  status.motion_possible.val = industrial::robot_status::TriStates::toROSMsgEnum(in.status_.getMotionPossible());
  
  pub_robot_status_->publish(status);

  // Reply back to the controller if the sender requested it.
  if (industrial::simple_message::CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    industrial::simple_message::SimpleMessage reply;
    in.toReply(reply, rtn ? industrial::simple_message::ReplyTypes::SUCCESS : industrial::simple_message::ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}

}
}