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
#include "simple_message/ping_message.hpp"
#include "simple_message/byte_array.hpp"
#else
#include "ping_message.hpp"
#include "byte_array.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::simple_message;
using namespace industrial::byte_array;

namespace industrial
{
namespace ping_message
{


PingMessage::PingMessage(void)
{
  this->init();
}

PingMessage::~PingMessage(void)
{

}


bool PingMessage::init(SimpleMessage & msg)
{
  bool rtn = false;

  if (this->getMessageType() == msg.getMessageType())
  {
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("ping_manager"), "Failed to initialize message, wrong type: %d, expected %d", msg.getMessageType(), this->getMessageType());
    rtn = false;
  }

  return rtn;
}


void PingMessage::init()
{
  this->setMessageType(StandardMsgTypes::PING);
}


}// ping_message
}// industrial




