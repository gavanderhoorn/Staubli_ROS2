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

#include "rclcpp/rclcpp.hpp"

using namespace robot_middleware;

int main(int argc, char** argv)
{
#ifndef NDEBUG
  RCLCPP_WARN(rclcpp::get_logger("robot_middleware_node"), "Running in DEBUG mode. The output might contain many debug messages!");
#endif

  rclcpp::init(argc, argv);
  std::shared_ptr<RobotMiddleware> middleware = std::make_shared<RobotMiddleware>();

  if (!middleware->init())
  {
    RCLCPP_ERROR(rclcpp::get_logger("robot_middleware_node"), "Could not initialize the middleware");
    return 1;
  }
  middleware->run();
  rclcpp::spin(middleware);
  rclcpp::shutdown();

  return 0;
}
