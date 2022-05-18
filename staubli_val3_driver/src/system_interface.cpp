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

#include <string>

#include "system_interface.hpp"
#include "set_drive_power_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "industrial_msgs/srv/set_drive_power.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#include "simple_message/socket/tcp_client.hpp"

using namespace std::placeholders;

SystemInterface::SystemInterface() : connection_(nullptr), Node("system_interface")
{
}

SystemInterface::~SystemInterface()
{
}

bool SystemInterface::init(const std::string& default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  this->declare_parameter<std::string>("robot_ip_address", default_ip);
  this->declare_parameter<int>("~port", default_port);
  this->get_parameter("robot_ip_address", ip);
  this->get_parameter("~port", port);

  // check for valid parameter values
  if (ip.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP address found. Please set the 'robot_ip_address' parameter");
    return false;
  }
  if (port <= 0 || port > 65535)
  {
    RCLCPP_ERROR(this->get_logger(), "No valid robot IP port found. Please set the '~port' parameter");
    return false;
  }

  // connection.init() requires "char*", not "const char*"
  char* ip_addr = strdup(ip.c_str());

  // create and connect client connection
  auto client = std::make_shared<industrial::tcp_client::TcpClient>();
  bool rtn = client->init(ip_addr, port);
  free(ip_addr);

  if (!rtn)
    return false;

  this->connection_ = client;
  RCLCPP_INFO(this->get_logger(), "system_interface: Connecting (%s:%d)", ip_addr, port);

  return this->connection_->makeConnect();
}

void SystemInterface::run()
{
  service = this->create_service<industrial_msgs::srv::SetDrivePower>("system_interface/set_drive_power", std::bind(&SystemInterface::setDrivePowerCb, this, _1, _2));
  RCLCPP_INFO_STREAM(this->get_logger(), "Service " << service->get_service_name() << " is ready and running");
}

void SystemInterface::setDrivePowerCb(const std::shared_ptr<industrial_msgs::srv::SetDrivePower::Request> req,
                                      std::shared_ptr<industrial_msgs::srv::SetDrivePower::Response> res)
{
  if (req->drive_power)
    RCLCPP_INFO(this->get_logger(), "Setting drive power ON");
  else
    RCLCPP_INFO(this->get_logger(), "Setting drive power OFF");

  if (setDrivePower(req->drive_power))
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::FAILURE;
  }
}

bool SystemInterface::setDrivePower(bool value)
{
  SetDrivePowerMessage msg;
  industrial::simple_message::SimpleMessage send, reply;
  msg.init(value);
  msg.toRequest(send);
  connection_->sendAndReceiveMsg(send, reply);
  return reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS;
}