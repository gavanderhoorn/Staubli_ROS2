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

#include "robot_middleware/message_relay_handler/message_relay_handler.hpp"

#include <cstdio>

using namespace industrial::simple_message;
using namespace robot_middleware::connection_manager;

namespace robot_middleware
{
namespace message_relay_handler
{

MessageRelayHandler::MessageRelayHandler(const std::string& name, std::shared_ptr<rclcpp::Node> node)
  : name_(name), driver_(nullptr), in_conn_mngr_(nullptr), out_conn_mngr_(nullptr)
{
  node_ = node;
}

MessageRelayHandler::~MessageRelayHandler()
{
  stop();
}

void MessageRelayHandler::init(const std::shared_ptr<RobotDriver>& driver, const SimpleSocketManagerPtr& in,
                               const SimpleSocketManagerPtr& out)
{
  driver_ = driver;
  in_conn_mngr_ = in;
  out_conn_mngr_ = out;
}

const char* MessageRelayHandler::getName()
{
  return name_.c_str();
}

void MessageRelayHandler::onReceiveTimeout()
{
  RCLCPP_DEBUG(node_->get_logger(), "[%s] Receive timeout", getName());
}

void MessageRelayHandler::onReceiveFail()
{
  RCLCPP_ERROR(node_->get_logger(), "[%s] Receive error", getName());
}

bool MessageRelayHandler::handleMessage(SimpleMessage& msg, rclcpp::Time& timestamp)
{
  RCLCPP_ERROR(node_->get_logger(), "[%s] Received message (msgType: %d)", getName(), msg.getMessageType());
  return true;
}

void MessageRelayHandler::spin()
{
  RCLCPP_DEBUG(node_->get_logger(), "[%s] Start spinning", getName());

  while (rclcpp::ok())
  {
    // wait until _in_ connection is ready
    while (rclcpp::ok() && !in_conn_mngr_->isConnected())
      rclcpp::Rate(10).sleep();

    spinOnce();
  }
}

void MessageRelayHandler::spinOnce()
{
  // check connection
  if (!in_conn_mngr_->isConnected())
    return;

  // handle receive timeout
  if (!in_conn_mngr_->isReadyReceive(receive_timeout_))
  {
    onReceiveTimeout();
    return;
  }

  // receive message
  SimpleMessage msg;
  if (!in_conn_mngr_->receiveMsg(msg))
  {
    onReceiveFail();
    return;
  }

  // handle message
  rclcpp::Time timestamp = node_->now();
  bool rtn = handleMessage(msg, timestamp);

  // relay message if 'confirmed' by handler callback
  if (rtn && out_conn_mngr_->isConnected())
  {
    rtn = out_conn_mngr_->sendMsg(msg);
    if (rtn)
    {
      RCLCPP_DEBUG(node_->get_logger(), "[%s] Relayed message", getName());
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "[%s] Could not relay message to OUT connection (%s)", getName(),
                      out_conn_mngr_->getName());
    }
  }
  else
  {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] Not relaying message", getName());
    rtn = false;
  }

  // relay reply message from _out_ to _in_ connection if required
  if (msg.getCommType() == CommType::SERVICE_REQUEST)
  {
    // receive reply from _out_ connection
    if (rtn)
    {
      SimpleMessage reply;
      rtn = out_conn_mngr_->receiveMsg(reply);
      if (rtn)
      {
        RCLCPP_DEBUG(node_->get_logger(), "[%s] Received reply message from OUT connection (%s)", getName(),
                        out_conn_mngr_->getName());

        // send reply to _in_ connection
        if (in_conn_mngr_->sendMsg(reply))
        {
          RCLCPP_DEBUG(node_->get_logger(), "[%s] Relayed reply message to IN connection (%s)", getName(),
                          in_conn_mngr_->getName());
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "[%s] Could not relay reply message to IN connection (%s)", getName(),
                          in_conn_mngr_->getName());
        }
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Could not receive reply message from OUT connection (%s)", getName(),
                        out_conn_mngr_->getName());
      }
    }

    // send 'FAILURE' reply if receiving reply from _out_ connection failed
    if (!rtn)
    {
      RCLCPP_DEBUG(node_->get_logger(), "[%s] Sending reply message 'FAILURE' to IN connection (%s)", getName(),
                      in_conn_mngr_->getName());

      sendReply(in_conn_mngr_, msg.getMessageType(), ReplyType::FAILURE);
    }
  }
}

void MessageRelayHandler::start()
{
  spinner_task_ = std::thread(&MessageRelayHandler::spin, this);
}

void MessageRelayHandler::stop()
{
  if (spinner_task_.joinable())
  {
#ifndef NDEBUG
    printf("DEBUG: Waiting for '%s' thread to join...\n", getName());
#endif
    spinner_task_.join();
  }
}

bool MessageRelayHandler::sendReply(const SimpleSocketManagerPtr& conn, int msg_type, ReplyType reply_type)
{
  SimpleMessage reply;
  reply.init(msg_type, CommType::SERVICE_REPLY, reply_type);
  return conn->sendMsg(reply);
}

}  // namespace message_relay_handler
}  // namespace robot_middleware