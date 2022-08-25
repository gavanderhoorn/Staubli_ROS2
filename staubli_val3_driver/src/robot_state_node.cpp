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

#include "staubli_val3_driver/joint_feedback_relay_handler.hpp"
#include "industrial_robot_client/robot_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  // initialize node
  rclcpp::init(argc, argv);

  // launch the default Robot State Interface connection/handlers
  std::shared_ptr<industrial_robot_client::robot_state_interface::RobotStateInterface> rsi = std::make_shared<industrial_robot_client::robot_state_interface::RobotStateInterface>();
  
  rsi->init();

  // add the JointFeedback handler
  JointFeedbackRelayHandler joint_fbk_handler;
  std::vector<std::string> joint_names = rsi->get_joint_names();
  joint_fbk_handler.init(rsi->get_connection(), joint_names);
  rsi->add_handler(&joint_fbk_handler);
  // run the node
  rsi->run();

  rclcpp::spin(rsi);
  rclcpp::shutdown();
  return 0;
}