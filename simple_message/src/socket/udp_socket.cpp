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
#include "simple_message/socket/udp_socket.hpp"
#include "simple_message/simple_message.hpp"
#else
#include "udp_socket.h"
#include "simple_message.h"
#endif
#include "rclcpp/rclcpp.hpp"


using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace udp_socket
{

UdpSocket::UdpSocket()
// Constructor for UDP socket object
{
  memset(&this->udp_read_buffer_, 0, sizeof(this->udp_read_buffer_));
  udp_read_head_ = this->udp_read_buffer_;
  udp_read_len_ = 0;
}

UdpSocket::~UdpSocket()
// Destructor for UDP socket object
// Closes socket
{
  CLOSE(this->getSockHandle());
}

int UdpSocket::rawSendBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;

  rc = SEND_TO(this->getSockHandle(), buffer,
        num_bytes, 0, (sockaddr *)&this->sockaddr_,
        sizeof(this->sockaddr_));
  
  return rc;
}

int UdpSocket::rawReceiveBytes(char *buffer, shared_int num_bytes)
{
  int rc, len_cpy;
  SOCKLEN_T addrSize;

  if(udp_read_len_ == 0) {
    // there is currently no data in the temporary buffer, do a socket read
    addrSize = sizeof(this->sockaddr_);

    rc = RECV_FROM(this->getSockHandle(), &this->udp_read_buffer_[0], this->MAX_BUFFER_SIZE,
        0, (sockaddr *)&this->sockaddr_, &addrSize);
    if(rc <= 0)
      return 0; // either we had an error or read no data, don't update the buffer
    udp_read_head_ = this->udp_read_buffer_;
    udp_read_len_ = rc;
  }
  if(num_bytes == 0 || num_bytes >= udp_read_len_) // read all data available
    len_cpy = udp_read_len_;
  else
    len_cpy = num_bytes;
  memcpy(buffer, udp_read_head_, len_cpy);
  udp_read_head_ += len_cpy; // shift pointer in buffer
  udp_read_len_ -= len_cpy;
  return len_cpy;
}

bool UdpSocket::rawPoll(int timeout, bool & ready, bool & error)
{
  if(udp_read_len_ > 0) {
    // we still have data in the buffer, we can read without socket calls
    ready = true;
    error = false;
    return true;
  }

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
        //RCLCPP_WARN(rclcpp::get_logger("udp_socket"), "Select returned, but no flags are set");
        rtn = false;
      }
    }
  } else {
    this->logSocketError("Socket select function failed", rc, errno);
    rtn = false;
  }
  return rtn;
}

} //udp_socket
} //industrial

