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

#pragma once

#include "robot_middleware/connection_manager/simple_socket_manager.hpp"
#include "robot_middleware/robot_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "simple_message/simple_message.hpp"

#include <string>
#include <thread>

namespace robot_middleware
{
namespace message_relay_handler
{

class MessageRelayHandler
{
public:
  explicit MessageRelayHandler(const std::string& name, std::shared_ptr<rclcpp::Node> node);

  ~MessageRelayHandler();

  virtual void init(const std::shared_ptr<RobotDriver>& driver,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& in,
                    const robot_middleware::connection_manager::SimpleSocketManagerPtr& out);

  const char* getName();

  void spin();

  void spinOnce();

  void start();

  void stop();

protected:
  virtual void onReceiveTimeout();

  virtual void onReceiveFail();

  virtual bool handleMessage(industrial::simple_message::SimpleMessage& msg, rclcpp::Time& timestamp);

  bool sendReply(const robot_middleware::connection_manager::SimpleSocketManagerPtr& conn, int msg_type,
                 industrial::simple_message::ReplyType reply_type);

  const std::string LOGNAME = "message_relay_handler";
  const double DEFAULT_RECEIVE_TIMEOUT = 1.0;  // seconds

  const std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<RobotDriver> driver_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr in_conn_mngr_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr out_conn_mngr_;
  int receive_timeout_;  // ms
  std::thread spinner_task_;
};

}  // namespace message_relay_handler
}  // namespace robot_middleware