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

#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/socket/tcp_client.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
namespace robot_middleware
{
namespace connection_manager
{

class TcpClientManager : public SimpleSocketManager
{
public:
  TcpClientManager(const std::string& name, const std::string& ip_address, int port);

  ~TcpClientManager() override;

  bool init() override;

  bool connect() override;

private:
  // sleep time before retrying to connect in seconds
  const int CONNECT_RETRY_DELAY = 3;

  char* ip_address_;
};

using TcpClientManagerPtr = std::shared_ptr<TcpClientManager>;

}  // namespace connection_manager
}  // namespace robot_middleware