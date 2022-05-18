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

#include "simple_message/simple_message.hpp"
#include "simple_message/socket/simple_socket.hpp"

#include <condition_variable>
#include <memory>
#include <mutex>
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

class SimpleSocketManager
{
public:
  const char* getName();

  void setFriendConnection(const std::shared_ptr<SimpleSocketManager>& conn);

  bool isConnected();

  bool isReadyReceive(int timeout);

  virtual bool init() = 0;

  virtual bool connect() = 0;

  virtual void disconnect();

  void startConnectionTask();

  bool sendMsg(industrial::simple_message::SimpleMessage& msg);

  bool receiveMsg(industrial::simple_message::SimpleMessage& msg);

  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage& send,
                         industrial::simple_message::SimpleMessage& recv);

protected:
  SimpleSocketManager(const std::string& name, int port);

  virtual ~SimpleSocketManager();

  const std::string LOGNAME = "connection_manager";
  const std::string name_;
  int port_;
  bool connected_once_;
  std::unique_ptr<industrial::simple_socket::SimpleSocket> conn_;

private:
  void connectionTask();

  std::mutex connection_mtx_;
  std::condition_variable connection_lost_cv_;
  std::shared_ptr<SimpleSocketManager> friend_connection_;
};

using SimpleSocketManagerPtr = std::shared_ptr<SimpleSocketManager>;

}  // namespace connection_manager
}  // namespace robot_middleware