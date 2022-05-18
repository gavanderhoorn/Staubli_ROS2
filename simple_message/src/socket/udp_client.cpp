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
#include "simple_message/socket/udp_client.hpp"
#else
#include "udp_client.hpp"
#endif
#include "rclcpp/rclcpp.hpp"
#include <netdb.h>

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace udp_client
{

UdpClient::UdpClient()
{
}

UdpClient::~UdpClient()
{
}

bool UdpClient::init(char *buff, int port_num)
{

  int rc;
  bool rtn;
  addrinfo *result;
  addrinfo hints = {};

  /* Create a socket using:
   * AF_INET - IPv4 internet protocol
   * SOCK_DGRAM - UDP type
   * protocol (0) - System chooses
   */
  rc = SOCKET(AF_INET, SOCK_DGRAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;

    // Check for 'buff' as hostname, and use that, otherwise assume IP address
    hints.ai_family = AF_INET;  // Allow IPv4
    hints.ai_socktype = SOCK_DGRAM;  // UDP socket
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

    rtn = true;

  }
  else
  {
    //RCLCPP_ERROR(rclcpp::get_logger("udp_client"), "Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}


bool UdpClient::makeConnect()
{
  ByteArray send;
  char sendHS = this->CONNECT_HANDSHAKE;
  char recvHS = 0;
  bool rtn = false;
  const int timeout = 1000;  // Time (ms) between handshake sends
  int bytesRcvd = 0;
  
  if (!this->isConnected())
  {
    this->setConnected(false);
    send.load((void*)&sendHS, sizeof(sendHS));
  
    // copy to local array, since ByteArray no longer supports
    // direct pointer-access to data values
    const int sendLen = send.getBufferSize();
    std::vector<char> localBuffer(sendLen);
    send.unload(localBuffer.data(), sendLen);

    do
    {
      ByteArray recv;
      recvHS = 0;
      //RCLCPP_DEBUG(rclcpp::get_logger("udp_client"), "UDP client sending handshake");
      this->rawSendBytes(localBuffer.data(), sendLen);
      if (this->isReadyReceive(timeout))
      {
        bytesRcvd = this->rawReceiveBytes(this->buffer_, 0);
 	      //RCLCPP_DEBUG(rclcpp::get_logger("udp_client"), "UDP client received possible handshake");	
        recv.init(&this->buffer_[0], bytesRcvd);
        recv.unload((void*)&recvHS, sizeof(recvHS));
      }
    }
    while(recvHS != sendHS);
    //RCLCPP_INFO(rclcpp::get_logger("udp_client"), "UDP client connected");
    rtn = true;
    this->setConnected(true);
    
  }
  else
  {
    rtn = true;
    //RCLCPP_WARN(rclcpp::get_logger("udp_client"), "Tried to connect when socket already in connected state");
  }

  return rtn;
}
} //udp_client
} //industrial

