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

#ifndef FLATHEADERS
#include "simple_message/socket/tcp_socket.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "tcp_socket.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace tcp_socket
{

TcpSocket::TcpSocket()
{
}

TcpSocket::~TcpSocket()
// Closes socket
{
  //RCLCPP_DEBUG(rclcpp::get_logger("tcp_socket"), "Destructing TCPSocket");
  CLOSE(this->getSockHandle());
}

int TcpSocket::rawSendBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;

  rc = SEND(this->getSockHandle(), buffer, num_bytes, 0);
  
  return rc;
}

int TcpSocket::rawReceiveBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  
  rc = RECV(this->getSockHandle(), buffer, num_bytes, 0);
  
  return rc;
}

bool TcpSocket::rawPoll(int timeout, bool & ready, bool & error)
{
  timeval time;
  fd_set read, write, except;
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  ready = false;
  error = false;

  // The select function uses the timeval data structure
  time.tv_sec = timeout / 1000;
  time.tv_usec = (timeout % 1000) * 1000;

  FD_ZERO(&read);
  FD_ZERO(&write);
  FD_ZERO(&except);

  FD_SET(this->getSockHandle(), &read);
  FD_SET(this->getSockHandle(), &except);

  rc = SELECT(this->getSockHandle() + 1, &read, &write, &except, &time);

  if (this->SOCKET_FAIL != rc) {
    if (0 == rc)
      rtn = false;
    else {
      if (FD_ISSET(this->getSockHandle(), &read)) {
        ready = true;
        rtn = true;
      }
      else if(FD_ISSET(this->getSockHandle(), &except)) {
        error = true;
        rtn = true;
      }
      else {
        //RCLCPP_WARN(rclcpp::get_logger("tcp_socket"), "Select returned, but no flags are set");
        rtn = false;
      }
    }
  } else {
    this->logSocketError("Socket select function failed", rc, errno);
    rtn = false;
  }
  return rtn;
}

} //tcp_socket
} //industrial

