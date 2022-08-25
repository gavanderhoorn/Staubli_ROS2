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

#include "staubli_val3_driver/write_single_io_message.hpp"

#include "simple_message/byte_array.hpp"
#include "rclcpp/rclcpp.hpp"

WriteSingleIOMessage::WriteSingleIOMessage()
{
  this->init();
}

WriteSingleIOMessage::~WriteSingleIOMessage()
{
}

bool WriteSingleIOMessage::init(industrial::simple_message::SimpleMessage &msg)
{
  bool rtn = false;
  industrial::byte_array::ByteArray data = msg.getData();
  this->init();
  this->setCommType(msg.getCommType());

  if (data.unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io_message"), "Failed to unload write single IO data");
  }
  return rtn;
}

void WriteSingleIOMessage::init(WriteSingleIO& writeSingleIO)
{
  this->init();
  this->writeSingleIO_.copyFrom(writeSingleIO);
}

void WriteSingleIOMessage::init()
{
  this->setMessageType(1621);  // TODO: make enum for Stäubli specific standard port numbers
  this->writeSingleIO_.init();
}

bool WriteSingleIOMessage::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn;
  RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Executing write single IO message load");

  if (buffer->load(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_ERROR(rclcpp::get_logger("write_single_io_message"), "Failed to load write single IO data");
  }
  return rtn;
}

bool WriteSingleIOMessage::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn;
  RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Executing write single IO message unload");

  if (buffer->unload(this->writeSingleIO_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    RCLCPP_INFO(rclcpp::get_logger("write_single_io_message"), "Failed to unload write single IO data");
  }
  return rtn;
}