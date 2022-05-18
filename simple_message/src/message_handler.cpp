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
#include "simple_message/message_handler.hpp"
#else
#include "message_handler.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace message_handler
{

using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;

MessageHandler::MessageHandler(void)
{
  this->setConnection(NULL);
  this->setMsgType(StandardMsgTypes::INVALID);
}


MessageHandler::~MessageHandler(void)
{
}


bool MessageHandler::init(int msg_type, SmplMsgConnection* connection)
{
  bool rtn = false;
  
  if (StandardMsgTypes::INVALID != msg_type)
  {
    if (NULL != connection)
    {
      this->setConnection(connection);
      this->setMsgType(msg_type);
      rtn = true;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Message connection is NULL");
      rtn = false;
    }
    }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Message handler type: %d, not valid", msg_type);
    rtn = false;
    }
    
  return rtn;
}

    
    
bool MessageHandler::callback(SimpleMessage & in)
{
  bool rtn = false;
  
  if (validateMsg(in))
  {
    this->internalCB(in);
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_handler"), "Invalid message passed to callback");
    rtn = true;
  }
  
  return rtn;
}


bool MessageHandler::validateMsg(SimpleMessage & in)
{
  bool rtn = false;
  
  if (in.validateMessage())
  {
    if (in.getMessageType() == this->getMsgType())
    {
      rtn = true;
    }
    else
    {
      //RCLCPP_WARN(rclcpp::get_logger("message_handler"), "Message type: %d, doesn't match handler type: %d", in.getMessageType(), this->getMsgType());
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("message_handler"), "Passed in message invalid");
  }

  return rtn;
  
}

} // namespace message_handler
} // namespace industrial
