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

#include "staubli_val3_driver/io_interface.hpp"
#include "staubli_val3_driver/write_single_io_message.hpp"

#include "rclcpp/rclcpp.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/socket/tcp_client.hpp"
#include "industrial_msgs/msg/service_return_code.hpp"

#include <string>

using namespace std::placeholders;

// Initialize module names map
const std::map<int, std::string> IOInterface::MODULE_NAMES = {
  { staubli_msgs::msg::IOModule::USER_IN, "UserIn" },
  { staubli_msgs::msg::IOModule::BASIC_IN, "BasicIn" },
  { staubli_msgs::msg::IOModule::BASIC_OUT, "BasicOut" },
  { staubli_msgs::msg::IOModule::VALVE_OUT, "ValveOut" },
  { staubli_msgs::msg::IOModule::BASIC_IN_2, "BasicIn-2" },
  { staubli_msgs::msg::IOModule::BASIC_OUT_2, "BasicOut-2" }
};

IOInterface::IOInterface() : Node("io_interface"), connection_(nullptr)
{
}

IOInterface::~IOInterface()
{
}

bool IOInterface::init(const std::string& default_ip, int default_port)
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
  RCLCPP_INFO(this->get_logger(), "io_interface: Connecting (%s:%i)", ip_addr, port);


  return this->connection_->makeConnect();
}

void IOInterface::run()
{
  service = this->create_service<staubli_msgs::srv::WriteSingleIO>("io_interface/write_single_io", std::bind(&IOInterface::writeSingleIOCb, this, _1, _2));
  RCLCPP_INFO_STREAM(this->get_logger(), "Service " << service->get_service_name() << " is ready and running");
}

void IOInterface::writeSingleIOCb(const std::shared_ptr<staubli_msgs::srv::WriteSingleIO::Request> req, 
                                std::shared_ptr<staubli_msgs::srv::WriteSingleIO::Response> res)
{
  std::string module_name;

  // lookup module name for requested module id
  if (MODULE_NAMES.find(req->module.id) != MODULE_NAMES.end())
  {
    module_name = MODULE_NAMES.at(req->module.id);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown module id given in WriteSingleIO service request!");
  }

  if (req->state)
  {
    RCLCPP_INFO(this->get_logger(), "Trying to set pin %d of module '%s'", req->pin, module_name.c_str());
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Trying to clear pin %d of module '%s'", req->pin, module_name.c_str());
  }

  if (writeSingleIO(IOModule(req->module.id), req->pin, req->state))
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res->code.val = industrial_msgs::msg::ServiceReturnCode::FAILURE;
  }
}

bool IOInterface::writeSingleIO(IOModule moduleId, int pin, bool state)
{
  WriteSingleIO write_io;
  write_io.init(moduleId, pin, state);
  WriteSingleIOMessage msg;
  msg.init(write_io);
  industrial::simple_message::SimpleMessage send, reply;
  msg.toRequest(send);
  connection_->sendAndReceiveMsg(send, reply);
  return (reply.getReplyCode() == industrial::simple_message::ReplyTypes::SUCCESS);
}