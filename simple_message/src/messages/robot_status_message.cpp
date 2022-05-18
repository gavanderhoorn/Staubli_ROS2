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
#ifndef FLATHEADERS
#include "simple_message/messages/robot_status_message.hpp"
#include "simple_message/robot_status.hpp"
#include "simple_message/byte_array.hpp"
#else
#include "robot_status_message.hpp"
#include "robot_status.hpp"
#include "byte_array.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::robot_status;

namespace industrial
{
namespace robot_status_message
{

RobotStatusMessage::RobotStatusMessage(void)
{
  this->init();
}

RobotStatusMessage::~RobotStatusMessage(void)
{

}

bool RobotStatusMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->status_))
  {
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("robot_status_message"), "Failed to unload robot status data");
  }
  return rtn;
}

void RobotStatusMessage::init(industrial::robot_status::RobotStatus & status)
{
  this->init();
  this->status_.copyFrom(status);
}

void RobotStatusMessage::init()
{
  this->setMessageType(StandardMsgTypes::STATUS);
  this->status_.init();
}

bool RobotStatusMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  //RCLCPP_INFO(rclcpp::get_logger("robot_status_message"), "Executing robot status message load");
  if (buffer->load(this->status_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("robot_status_message"), "Failed to load robot status data");
  }
  return rtn;
}

bool RobotStatusMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  //RCLCPP_INFO(rclcpp::get_logger("robot_status_message"), "Executing robot status message unload");

  if (buffer->unload(this->status_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("robot_status_message"), "Failed to unload robot status data");
  }
  return rtn;
}

}
}

