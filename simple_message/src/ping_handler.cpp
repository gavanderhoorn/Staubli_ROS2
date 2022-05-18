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
#include "simple_message/ping_handler.hpp"
#include "simple_message/ping_message.hpp"
#else
#include "ping_handler.hpp"
#include "ping_message.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::ping_message;
using namespace industrial::simple_message;

namespace industrial
{
namespace ping_handler
{


bool PingHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  return this->init(StandardMsgTypes::PING, connection);
}

bool PingHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  bool rtn = false;
  PingMessage ping;
  SimpleMessage msg;

  if (ping.init(in))
  {
    if (ping.toReply(msg, ReplyTypes::SUCCESS))
    {
      if(this->getConnection()->sendMsg(msg))
      {
        //RCLCPP_INFO(rclcpp::get_logger("ping_handler"), "Ping return sent");
        rtn = true;
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("ping_handler"), "Failed to send ping return");
        rtn = false;
      }
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("ping_handler"), "Failed to generate ping reply message");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("ping_handler"), "Failed to initialize ping message");
    rtn = false;
  }


  return rtn;
}



}//namespace ping_handler
}//namespace industrial



