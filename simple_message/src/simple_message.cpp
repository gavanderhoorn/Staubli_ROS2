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
#include "simple_message/simple_message.hpp"
#else
#include "simple_message.hpp"
#endif

#ifdef SIMPLE_MESSAGE_MOTOPLUS
#include "motoPlus.h"
#endif

#include "rclcpp/rclcpp.hpp"


using namespace industrial::byte_array;

namespace industrial
{

namespace simple_message
{

SimpleMessage::SimpleMessage(void)
{
}

SimpleMessage::~SimpleMessage(void)
{
}



bool SimpleMessage::init(int msgType, int commType, int replyCode)
{
  ByteArray data;
  data.init();
  return this->init(msgType, commType, replyCode, data);
}

bool SimpleMessage::init(int msgType, int commType, int replyCode, ByteArray & data )
{
  //RCLCPP_INFO(rclcpp::get_logger("simple_message"), "SimpleMessage::init(type: %d, comm: %d, reply: %d, data[%d]...)", msgType, commType, replyCode, data.getBufferSize());
  this->setMessageType(msgType);
  this->setCommType(commType);
  this->setReplyCode(replyCode);
  this->data_.copyFrom(data);

  return this->validateMessage();
}

bool SimpleMessage::init(ByteArray & msg)
{
  int dataSize = 0;
  bool rtn = false;

  if (msg.getBufferSize() >= this->getHeaderSize())
  {
    // Check to see if the message is larger than the standard header
    // If so, copy out the data portion.
    if (msg.getBufferSize() > this->getHeaderSize())
    {
      dataSize = msg.getBufferSize() - this->getHeaderSize();
      //RCLCPP_INFO(rclcpp::get_logger("simple_message"), "Unloading data portion of message: %d bytes", dataSize);
      msg.unload(this->data_, dataSize);
    }
    //RCLCPP_INFO(rclcpp::get_logger("simple_message"), "Unloading header data");
    msg.unload(this->reply_code_);
    msg.unload(this->comm_type_);
    msg.unload(this->message_type_);
    //RCLCPP_INFO(rclcpp::get_logger("simple_message"), "SimpleMessage::init(type: %d, comm: %d, reply: %d, data[%d]...)", this->message_type_, this->comm_type_, this->reply_code_, this->data_.getBufferSize());
    rtn = this->validateMessage();
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("simple_message"), "Failed to init message, buffer size too small: %u", msg.getBufferSize());
    rtn = false;
  }
  return rtn;
}

void SimpleMessage::toByteArray(ByteArray & msg)
{
  msg.init();

  msg.load(this->getMessageType());
  msg.load(this->getCommType());
  msg.load(this->getReplyCode());
  if (this->data_.getBufferSize() > 0 )
  {
    msg.load(this->data_);
  }

}


void SimpleMessage::setData( ByteArray & data)
{
  this->data_.copyFrom(data);
}


bool SimpleMessage::validateMessage()
{

  if ( StandardMsgTypes::INVALID == this->getMessageType())
  {
    //RCLCPP_WARN(rclcpp::get_logger("simple_message"), "Invalid message type: %u", this->getMessageType());
    return false;
  }

  if ( CommTypes::INVALID == this->getCommType())
  {
    //RCLCPP_WARN(rclcpp::get_logger("simple_message"), "Invalid comms. type: %u", this->getCommType());
    return false;
  }

  if (
      (CommTypes::SERVICE_REPLY ==  this->getCommType() &&
          ReplyTypes::INVALID == this->getReplyCode()) ||
          ((CommTypes::SERVICE_REPLY !=  this->getCommType() &&
              ReplyTypes::INVALID != this->getReplyCode()))
  )
  {
    //RCLCPP_WARN(rclcpp::get_logger("simple_message"), "Invalid reply. Comm type: %u, Reply type: %u", this->getCommType(), this->getReplyCode());
    return false;
  }

  return true;
}



	
} // namespace simple_message
} // namespace industrial
