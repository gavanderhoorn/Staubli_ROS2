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

#include "robot_middleware/connection_manager/tcp_server_manager.hpp"

namespace robot_middleware
{

class RobotServerProxy
{
public:
  RobotServerProxy(std::shared_ptr<rclcpp::Node> node);

  ~RobotServerProxy();

  bool init(int default_motion_port, int default_state_port);

  const robot_middleware::connection_manager::SimpleSocketManagerPtr& getMotionServerManager()
  {
    return motion_server_manager_;
  }

  const robot_middleware::connection_manager::SimpleSocketManagerPtr& getStateServerManager()
  {
    return state_server_manager_;
  }

  void connect();

private:
  std::shared_ptr<rclcpp::Node> node_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr motion_server_manager_;
  robot_middleware::connection_manager::SimpleSocketManagerPtr state_server_manager_;
};

}  // namespace robot_middleware