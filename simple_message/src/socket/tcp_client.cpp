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
#include "simple_message/socket/tcp_client.hpp"
#else
#include "tcp_client.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include <netdb.h>

namespace industrial
{
namespace tcp_client
{

TcpClient::TcpClient()
{

}

TcpClient::~TcpClient()
{
  //RCLCPP_DEBUG(rclcpp::get_logger("tcp_client"), "Destructing TCPClient");
}

bool TcpClient::init(char *buff, int port_num)
{
  addrinfo *result;
  addrinfo hints = {};

  if (!connectSocketHandle())
  {
    return false;
  }

  // Initialize address data structure
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
  this->sockaddr_.sin_family = AF_INET;


  // Check for 'buff' as hostname, and use that, otherwise assume IP address
  hints.ai_family = AF_INET;  // Allow IPv4
  hints.ai_socktype = SOCK_STREAM;  // TCP socket
  hints.ai_flags = 0;  // No flags
  hints.ai_protocol = 0;  // Any protocol
  hints.ai_canonname = NULL;
  hints.ai_addr = NULL;
  hints.ai_next = NULL;
  if (0 == GETADDRINFO(buff, NULL, &hints, &result))
  {
    this->sockaddr_ = *((sockaddr_in *)result->ai_addr);
  }
  else
  {
    this->sockaddr_.sin_addr.s_addr = INET_ADDR(buff);
  }
  this->sockaddr_.sin_port = HTONS(port_num);

  return true;
}

bool TcpClient::makeConnect()
{
  if (isConnected())
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_client"), "Tried to connect when socket already in connected state");
    return false;
  }

  if (!connectSocketHandle())
  {
    // Logging handled by connectSocketHandle()
    return false;
  }

  int rc = CONNECT(this->getSockHandle(), (sockaddr *)&sockaddr_, sizeof(sockaddr_));
  if (SOCKET_FAIL == rc)
  {
    logSocketError("Failed to connect to server", rc, errno);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("tcp_client"), "Connected to server");
  setConnected(true);

  return true;
}

bool TcpClient::connectSocketHandle()
{
  if (isConnected())
  {
    // Already connected, nothing to do
    return true;
  }

  int sock_handle = getSockHandle();

  if (sock_handle != SOCKET_FAIL)
  {
    // Handle is stale close old handle
    CLOSE(sock_handle);
  }

  sock_handle = SOCKET(AF_INET, SOCK_STREAM, 0);
  setSockHandle(sock_handle);
  if (SOCKET_FAIL == sock_handle)
  {
    //RCLCPP_ERROR(rclcpp::get_logger("tcp_client"), "Failed to create socket");
    return false;
  }

  int disableNodeDelay = 1;
  // The set no delay disables the NAGEL algorithm
  if (SOCKET_FAIL == SET_NO_DELAY(sock_handle, disableNodeDelay))
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_client"), "Failed to set no socket delay, sending data can be delayed by up to 250ms");
  }
  return true;
}
} //tcp_client
} //industrial

