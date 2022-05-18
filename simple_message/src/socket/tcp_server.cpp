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
#include "simple_message/socket/tcp_server.hpp"
#else
#include "tcp_server.h"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace tcp_server
{

TcpServer::TcpServer()
{
  this->setSockHandle(this->SOCKET_FAIL);
  this->setSrvrHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
}

TcpServer::~TcpServer()
{
  CLOSE(this->getSockHandle());
  CLOSE(this->getSrvrHandle());
}

bool TcpServer::init(int port_num)
{
  int rc;
  bool rtn;
  const int reuse_addr = 1;
  //int err;
  SOCKLEN_T addrSize = 0;

  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSrvrHandle(rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket created, rc: %d", rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket handle: %d", this->getSrvrHandle());

    
    SET_REUSE_ADDR(this->getSrvrHandle(), reuse_addr);

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = HTONS(port_num);

    addrSize = sizeof(this->sockaddr_);
    rc = BIND(this->getSrvrHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      RCLCPP_INFO(rclcpp::get_logger("tcp_server"), "Server socket successfully initialized");

      rc = LISTEN(this->getSrvrHandle(), 1);

      if (this->SOCKET_FAIL != rc)
      {
       RCLCPP_DEBUG(rclcpp::get_logger("tcp_server"), "Socket in listen mode");
        rtn = true;
      }
      else
      {
        //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to set socket to listen");
        rtn = false;
      }
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to bind socket, rc: %d", rc);
      CLOSE(this->getSrvrHandle());
      rtn = false;
    }

  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to create socket, rc: %d", rc);
    rtn = false;
  }

  return rtn;
}

bool TcpServer::makeConnect()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  //int socket = this->SOCKET_FAIL;
  int disableNodeDelay = 1;
  int err = 0;

  if (!this->isConnected())
  {
    this->setConnected(false);
    if (this->SOCKET_FAIL != this->getSockHandle())
    {
      CLOSE(this->getSockHandle());
      this->setSockHandle(this->SOCKET_FAIL);
    }

    rc = ACCEPT(this->getSrvrHandle(), NULL, NULL);

    if (this->SOCKET_FAIL != rc)
    {
      this->setSockHandle(rc);
      //RCLCPP_INFO(rclcpp::get_logger("tcp_server"), "Client socket accepted");

      // The set no delay disables the NAGEL algorithm
      rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
      err = errno;
      if (this->SOCKET_FAIL == rc)
      {
        //RCLCPP_WARN(rclcpp::get_logger("tcp_server"), "Failed to set no socket delay, errno: %d, sending data can be delayed by up to 250ms", err);
      }
      this->setConnected(true);
      rtn = true;
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("tcp_server"), "Failed to accept for client connection");
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("tcp_server"), "Tried to connect when socket already in connected state");
  }

  return rtn;

}

} //tcp_socket
} //industrial

