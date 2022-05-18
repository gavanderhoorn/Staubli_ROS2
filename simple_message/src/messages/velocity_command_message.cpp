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

#include "simple_message/messages/velocity_command_message.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace industrial::byte_array;
using namespace industrial::simple_message;

namespace industrial
{
namespace velocity_command_message
{

VelocityCommandMessage::VelocityCommandMessage()
{
  this->init();
}

VelocityCommandMessage::~VelocityCommandMessage()
{
}

void VelocityCommandMessage::init()
{
  this->setMessageType(1641);
  this->data_.init();
}

void VelocityCommandMessage::init(const industrial::velocity_command::VelocityCommand& data)
{
  this->data_ = data;
}

bool VelocityCommandMessage::init(SimpleMessage& msg)
{
  this->init();
  ByteArray data = msg.getData();
  if (!data.unload(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_command_message"), "Failed to unload velocity command message data");
    return false;
  }
  return true;
}

bool VelocityCommandMessage::load(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_command_message"), "Executing velocity command message load");

  if (!buffer->load(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_command_message"), "Failed to load velocity command data");
    return false;
  }

  return true;
}

bool VelocityCommandMessage::unload(ByteArray* buffer)
{
  //RCLCPP_INFO(rclcpp::get_logger("velocity_command_message"), "Executing velocity command message unload");

  if (!buffer->unload(this->data_))
  {
    //RCLCPP_ERROR(rclcpp::get_logger("velocity_command_message"), "Failed to unload velocity command data");
    return false;
  }

  return true;
}

}  // namespace industrial
}  // namespace velocity_command_message