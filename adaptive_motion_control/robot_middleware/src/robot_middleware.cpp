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

#include "robot_middleware/robot_middleware.hpp"

using namespace std::chrono_literals;

namespace robot_middleware
{

RobotMiddleware::RobotMiddleware() : Node("robot_middleware")
{
}

RobotMiddleware::~RobotMiddleware()
{
}

bool RobotMiddleware::init()
{
  jog_interface_ = std::make_shared<JogInterface>(this->shared_from_this());
  pose_tracking_controller_ = std::make_shared<PoseTrackingController>(this->shared_from_this());
  server_proxy_ = std::make_shared<RobotServerProxy>(this->shared_from_this());
  state_relay_handler_ = std::make_shared<robot_middleware::message_relay_handler::StateRelayHandler>(this->shared_from_this());
  motion_relay_handler_ = std::make_shared<robot_middleware::message_relay_handler::MotionRelayHandler>(this->shared_from_this());

  //We need to get the robot description as a parameter of this node to initialise the robot_model_loader properly.
  std::vector <rclcpp::Parameter> xml_strings = {};
  rclcpp::Parameter xml_string;
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  xml_strings = parameters_client->get_parameters({"robot_description"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("~robot_description", xml_string.as_string());

  //Do the same for the semantic description
  xml_strings = {};
  parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  xml_strings = parameters_client->get_parameters({"robot_description_semantic"});
  xml_string = xml_strings[0];
  this->declare_parameter<std::string>("~robot_description_semantic", xml_string.as_string());
  
  //Init the robot_model_loader
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this(), "~robot_description", false);

  // get the robot IP from private parameter
  std::string robot_ip;

  this->declare_parameter<std::string>("robot_ip", "");
  this->get_parameter<std::string>("robot_ip", robot_ip);

  if (robot_ip == "")
  {
    RCLCPP_ERROR(this->get_logger(), "robot_ip parameter found empty!");
    return false;
  }

  // check if driver type is given as a parameter
  std::string driver_type = "staubli";
  driver_ = std::make_shared<staubli::StaubliDriver>(robot_ip, robot_model_loader_->getModel(), this->shared_from_this());

  // init the driver
  if (!driver_->init())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize the robot driver");
    return false;
  }

  // init the robot server proxy
  if (!server_proxy_->init(driver_->getDefaultMotionPort(), driver_->getDefaultStatePort()))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize the robot server proxy");
    return false;
  }

  // init the relay handlers
  state_relay_handler_->init(/* driver:  */ driver_,
                            /* in:      */ driver_->getStateClientManager(),
                            /* out:     */ server_proxy_->getStateServerManager());
  motion_relay_handler_->init(/* driver:  */ driver_,
                             /* in:      */ server_proxy_->getMotionServerManager(),
                             /* out:     */ driver_->getMotionClientManager());

  // init the controllers
  jog_interface_->init(driver_);
  pose_tracking_controller_->init(driver_);

  RCLCPP_INFO(this->get_logger(), "Initialized the middleware");

  return true;
}

void RobotMiddleware::run()
{

  // start message relay handlers
  motion_relay_handler_->start();
  state_relay_handler_->start();

  // start controllers
  jog_interface_->start();
  pose_tracking_controller_->start();

  RCLCPP_INFO(this->get_logger(), "All interfaces are ready and running. Starting the connection tasks.");

  // start async connection tasks
  driver_->connect();
  server_proxy_->connect();
}

}  // namespace robot_middleware
