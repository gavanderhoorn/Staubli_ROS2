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
#include "simple_message/socket/udp_server.hpp"
#else
#include "udp_server.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::byte_array;

namespace industrial
{
namespace udp_server
{

UdpServer::UdpServer()
{
  this->setConnected(false);
}

UdpServer::~UdpServer()
{
}



bool UdpServer::init(int port_num)
{
  int rc = this->SOCKET_FAIL;
  bool rtn;
  SOCKLEN_T addrSize = 0;

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("udp_server"), "Socket created, rc: %d", rc);
    //RCLCPP_DEBUG(rclcpp::get_logger("udp_server"), "Socket handle: %d", this->getSockHandle());

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;
    this->sockaddr_.sin_addr.s_addr = INADDR_ANY;
    this->sockaddr_.sin_port = HTONS(port_num);

    // This set the socket to be non-blocking (NOT SURE I WANT THIS) - sme
    //fcntl(sock_handle, F_SETFL, O_NONBLOCK);

    addrSize = sizeof(this->sockaddr_);
    rc = BIND(this->getSockHandle(), (sockaddr *)&(this->sockaddr_), addrSize);

    if (this->SOCKET_FAIL != rc)
    {
      rtn = true;
      RCLCPP_INFO(rclcpp::get_logger("udp_server"), "Server socket successfully initialized");
    }
    else
    {
      //RCLCPP_ERROR(rclcpp::get_logger("udp_server"), "Failed to bind socket, rc: %d", rc);
      CLOSE(this->getSockHandle());
      rtn = false;
    }
  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("udp_server"), "Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}


bool UdpServer::makeConnect()
{
  ByteArray send;
  char sendHS = this->CONNECT_HANDSHAKE;
  char recvHS = 0;
  int bytesRcvd = 0;
  const int timeout = 1000;  // Time (ms) between handshake sends
  bool rtn = false;
  
  send.load((void*)&sendHS, sizeof(sendHS));
    
  if (!this->isConnected())
  {
    this->setConnected(false);
    
    // Listen for handshake.  Once received, break
    // listen loop.
    do
    {
      ByteArray recv;
      recvHS = 0;
      if (this->isReadyReceive(timeout))
      {
        bytesRcvd = this->rawReceiveBytes(this->buffer_, 0);
        
        if (bytesRcvd > 0)
        {
          //RCLCPP_DEBUG(rclcpp::get_logger("udp_server"), "UDP server received %d bytes while waiting for handshake", bytesRcvd);
          recv.init(&this->buffer_[0], bytesRcvd);
          recv.unload((void*)&recvHS, sizeof(recvHS));
        }
      }
      
    }
    while(recvHS != sendHS);
    
    // copy to local array, since ByteArray no longer supports
    // direct pointer-access to data values
    const int sendLen = send.getBufferSize();
    std::vector<char> localBuffer(sendLen);
    send.unload(localBuffer.data(), sendLen);

    // Send a reply handshake
    this->rawSendBytes(localBuffer.data(), sendLen);
    this->setConnected(true);
    rtn = true;
    
  }
  else
  {
    //RCLCPP_WARN(rclcpp::get_logger("udp_server"), "Tried to connect when socket already in connected state");
    rtn = true;
  }

  return rtn;
}


} //udp_server
} //industrial

