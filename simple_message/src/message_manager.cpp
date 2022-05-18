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
#include "simple_message/message_manager.hpp"
#include "simple_message/simple_message.hpp"
#else
#include "message_manager.hpp"
#include "simple_message.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
#else
#include "unistd.h"
#endif

#include <thread>
#include <chrono>

using namespace industrial::smpl_msg_connection;
using namespace industrial::message_handler;
using namespace industrial::simple_message;
using namespace industrial::comms_fault_handler;
using namespace industrial::simple_comms_fault_handler;

namespace industrial
{
namespace message_manager
{

/**
 * \brief Constructor
 */
MessageManager::MessageManager()
{
  this->num_handlers_ = 0;
  for (unsigned int i = 0; i < this->getMaxNumHandlers(); i++)
  {
    this->handlers_[i] = NULL;
  }
  this->comms_hndlr_ = NULL;
}

MessageManager::~MessageManager()
{

}

bool MessageManager::init(SmplMsgConnection* connection)
{
  bool rtn = false;

  //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Initializing message manager with default comms fault handler");


  if (NULL != connection)
  {
    this->getDefaultCommsFaultHandler().init(connection);
    this->init(connection, (CommsFaultHandler*)(&this->getDefaultCommsFaultHandler()));
    rtn = true;
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL connection passed into manager init");
    rtn = false;
  }

  return rtn;
}

bool MessageManager::init(SmplMsgConnection* connection, CommsFaultHandler* fault_handler)
{
  bool rtn = false;

    //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Initializing message manager");

    if (NULL != connection && NULL != fault_handler)
    {
      this->setConnection(connection);
      this->getPingHandler().init(connection);
      this->setCommsFaultHandler(fault_handler);

      if (this->add(&this->getPingHandler()))
      {
        rtn = true;
      }
      else
      {
        rtn = false;
        //RCLCPP_WARN(rclcpp::get_logger("message_manager"), "Failed to add ping handler, manager won't respond to pings");
      }

    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL connection or NULL fault handler passed into manager init");
      rtn = false;
    }

    return rtn;
  }



void MessageManager::spinOnce()
{
  SimpleMessage msg;
  MessageHandler* handler = NULL;

  if(!this->getConnection()->isConnected())
  {
    this->getCommsFaultHandler()->connectionFailCB();
  }

  if (this->getConnection()->receiveMsg(msg))
  {
    //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Message received");
    handler = this->getHandler(msg.getMessageType());

    if (NULL != handler)
    {
      //RCLCPP_DEBUG(rclcpp::get_logger("message_manager"), "Executing handler callback for message type: %d", handler->getMsgType());
      handler->callback(msg);
    }
    else
    {
      if (CommTypes::SERVICE_REQUEST == msg.getCommType())
      {
        simple_message::SimpleMessage fail;
        fail.init(msg.getMessageType(), CommTypes::SERVICE_REPLY, ReplyTypes::FAILURE);
        this->getConnection()->sendMsg(fail);
        //RCLCPP_WARN(rclcpp::get_logger("message_manager"), "Unhandled message type encounters, sending failure reply");
      }
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Message callback for message type: %d, not executed", msg.getMessageType());
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Failed to receive incoming message");
    this->getCommsFaultHandler()->sendFailCB();
  }
}

int ms_per_clock;
void mySleep(int sec)
{
#ifdef SIMPLE_MESSAGE_MOTOPLUS
  if (ms_per_clock <= 0)
    ms_per_clock = mpGetRtc();

  mpTaskDelay(sec * 1000 / ms_per_clock);
#else
  std::this_thread::sleep_for(std::chrono::seconds(sec));
#endif
}

void MessageManager::spin()
{
  //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Entering message manager spin loop");
// remove ROS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_USE_ROS) || defined(ROS)
  while (rclcpp::ok())
#else
  while (true)
#endif
  {
    this->spinOnce();

    // Throttle loop speed if waiting for a re-connection
    if (!this->getConnection()->isConnected())
      mySleep(5);
  }
}

bool MessageManager::add(MessageHandler * handler, bool allow_replace)
{
  int idx = -1;
  bool rtn = false;

  if (NULL != handler)
  {
    // If get handler returns -1 then a hander for the message type
    // does not exist, and a new one should be added
    idx = getHandlerIdx(handler->getMsgType());
    if (0 > idx)
    {
      if (this->getMaxNumHandlers() > this->getNumHandlers())
      {
        this->handlers_[this->getNumHandlers()] = handler;
        this->setNumHandlers(this->getNumHandlers() + 1);
        //RCLCPP_INFO(rclcpp::get_logger("message_manager"), "Added message handler for message type: %d", handler->getMsgType());
        rtn = true;
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Max number of handlers exceeded");
        rtn = false;
      }
    }
    else if (allow_replace)
    {
      this->handlers_[idx] = handler;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "Failed to add handler for: %d, handler already exists", handler->getMsgType());
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("message_manager"), "NULL handler not added");
    rtn = false;
  }
  return rtn;
}

MessageHandler* MessageManager::getHandler(int msg_type)
{
  int idx = getHandlerIdx(msg_type);

  if (idx < 0)
    return NULL;
  else
    return this->handlers_[idx];
}

int MessageManager::getHandlerIdx(int msg_type)
{
  int rtn = -1;
  MessageHandler* temp = NULL;

  for (unsigned int i = 0; i < this->getMaxNumHandlers(); i++)
  {
    temp = this->handlers_[i];
    if (NULL == temp) break;  // end of handler-list

    if (temp->getMsgType() == msg_type)
    {
        rtn = i;
        break;
    }
  }

  return rtn;
}

} // namespace message_manager
} // namespace industrial
