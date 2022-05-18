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
#include "simple_message/simple_comms_fault_handler.hpp"
#else
#include "simple_comms_fault_handler.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace simple_comms_fault_handler
{

SimpleCommsFaultHandler::SimpleCommsFaultHandler()
{
  this->connection_ = NULL;
}


SimpleCommsFaultHandler::~SimpleCommsFaultHandler()
{
}

bool SimpleCommsFaultHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  bool rtn = false;

  if (NULL != connection)
  {
    this->setConnection(connection);
    //RCLCPP_INFO(rclcpp::get_logger("simple_comms_fault_handler"), "Default communications fault handler successfully initialized");
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("simple_comms_fault_handler"), "Failed to initialize default communications fault handler");
  }
  return rtn;
}

void SimpleCommsFaultHandler::connectionFailCB()
{

  if (!(this->getConnection()->isConnected()))
  {
    //RCLCPP_INFO(rclcpp::get_logger("simple_comms_fault_handler"), "Connection failed, attempting reconnect");
    this->getConnection()->makeConnect();
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("simple_comms_fault_handler"), "Connection fail callback called while still connected (Possible bug)");
  }
}



}//namespace default_comms_fault_handler
}//namespace industrial




