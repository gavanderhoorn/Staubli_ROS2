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

#include "set_drive_power_message.hpp"

#include "simple_message/byte_array.hpp"
#include "rclcpp/rclcpp.hpp"

SetDrivePowerMessage::SetDrivePowerMessage()
{
  this->init();
}

SetDrivePowerMessage::~SetDrivePowerMessage()
{
}

bool SetDrivePowerMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->drive_power_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("set_drive_power_message"), "Failed to unload SetDrivePower data");
  }
  return rtn;
}

void SetDrivePowerMessage::init()
{
  this->setMessageType(1610);  // TODO: make enum for Stäubli specific standard port numbers
}

void SetDrivePowerMessage::init(industrial::shared_types::shared_bool drive_power)
{
  this->init();
  this->drive_power_ = drive_power;
}

bool SetDrivePowerMessage::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "Executing SetDrivePower load");

  if (buffer->load(this->drive_power_))
  {

    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower successfully loaded");
    rtn = true;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower not loaded");
    rtn = false;
  }

  return rtn;
}

bool SetDrivePowerMessage::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;

  RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "Executing SetDrivePower unload");
  if (buffer->unload(this->drive_power_))
  {

    rtn = true;
    RCLCPP_INFO(rclcpp::get_logger("set_drive_power_message"), "SetDrivePower successfully unloaded");
  }

  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("set_drive_power_message"), "Failed to unload SetDrivePower");
    rtn = false;
  }

  return rtn;
}