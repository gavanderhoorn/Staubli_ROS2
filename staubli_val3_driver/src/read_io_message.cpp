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

#include "read_io_message.hpp"

#include "simple_message/byte_array.hpp"
#include "simple_message/simple_message.hpp"

#include "rclcpp/rclcpp.hpp"

ReadIOMessage::ReadIOMessage()
{
  this->init();
}

ReadIOMessage::~ReadIOMessage()
{
}

bool ReadIOMessage::init(industrial::simple_message::SimpleMessage& msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to unload read IO data");
  }
  return rtn;
}

void ReadIOMessage::init()
{
  this->setMessageType(1620);  // TODO: make enum for Stäubli specific standard port numbers
}

bool ReadIOMessage::load(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;
  RCLCPP_INFO(rclcpp::get_logger("read_io_message"), "Executing read IO message load");
  if (buffer->load(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to read IO states data");
  }
  return rtn;
}

bool ReadIOMessage::unload(industrial::byte_array::ByteArray* buffer)
{
  bool rtn = false;
  RCLCPP_INFO(rclcpp::get_logger("read_io_message"), "Executing read IO message unload");

  if (buffer->unload(this->states_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("read_io_message"), "Failed to unload read IO data");
  }
  return rtn;
}