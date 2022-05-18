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
#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/byte_array.hpp"
#else
#include "smpl_msg_connection.hpp"
#include "byte_array.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

#ifdef SIMPLE_MESSAGE_MOTOPLUS
#include "motoPlus.h"
#endif

using namespace industrial::simple_message;
using namespace industrial::shared_types;
using namespace industrial::byte_array;

namespace industrial
{

namespace smpl_msg_connection
{


bool SmplMsgConnection::sendMsg(SimpleMessage & message)
{
  bool rtn;
  ByteArray sendBuffer;
  ByteArray msgData;

  if (message.validateMessage())
  {
    message.toByteArray(msgData);
    sendBuffer.load((int)msgData.getBufferSize());
    sendBuffer.load(msgData);
    rtn = this->sendBytes(sendBuffer);
  }
  else
  {
    rtn = false;
    //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Message validation failed, message not sent");
  }

return rtn;
}


bool SmplMsgConnection::receiveMsg(SimpleMessage & message)
{
  return receiveMsg(message, -1);
}


bool SmplMsgConnection::receiveMsg(SimpleMessage & message, shared_int timeout_ms)
{
  ByteArray lengthBuffer;
  ByteArray msgBuffer;
  int length;

  bool rtn = false;


  rtn = this->receiveBytes(lengthBuffer, message.getLengthSize(), timeout_ms);

  if (rtn)
  {
    rtn = lengthBuffer.unload(length);
    //RCLCPP_INFO(rclcpp::get_logger("smpl_msg_connection"), "Message length: %d", length);

    if (rtn)
    {
      rtn = this->receiveBytes(msgBuffer, length, timeout_ms);

      if (rtn)
      {
        rtn = message.init(msgBuffer);
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to initialize message");
        rtn = false;
      }

    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to receive message");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Failed to receive message length");
    rtn = false;
  }

  return rtn;
}


bool SmplMsgConnection::sendAndReceiveMsg(SimpleMessage & send, SimpleMessage & recv, bool verbose)
{
  return sendAndReceiveMsg(send, recv, -1, verbose);
}

bool SmplMsgConnection::sendAndReceiveMsg(SimpleMessage & send, SimpleMessage & recv,
                                          shared_int timeout_ms, bool verbose)
{
  bool rtn = false;
  rtn = this->sendMsg(send);
  if (rtn)
  {
    if(verbose) {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Sent message");
    }
    rtn = this->receiveMsg(recv, timeout_ms);
    if(verbose) {
      //RCLCPP_ERROR(rclcpp::get_logger("smpl_msg_connection"), "Got message");
    }
  }
  else
  {
    rtn = false;
  }

  return rtn;
}


}//smpl_msg_connection
}//industrial
