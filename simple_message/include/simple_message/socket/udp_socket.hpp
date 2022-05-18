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

#ifndef FLATHEADERS
#include "simple_message/socket/simple_socket.hpp"
#include "simple_message/shared_types.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#else
#include "simple_socket.hpp"
#include "shared_types.hpp"
#include "smpl_msg_connection.hpp"
#endif

// remove LINUXSOCKETS after Melodic (bw compat for #262)
#if defined(SIMPLE_MESSAGE_LINUX) || defined(LINUXSOCKETS)
#ifndef WIN32
#include "sys/socket.h"
#include "netdb.h"
#include "arpa/inet.h"
#include "unistd.h"
#else
#include <ws2tcpip.h>
#endif
#include "string.h"
#endif

#ifdef SIMPLE_MESSAGE_MOTOPLUS
#include "motoPlus.h"
#endif

namespace industrial
{
namespace udp_socket
{

class UdpSocket : public industrial::simple_socket::SimpleSocket
{
public:

  UdpSocket();
  ~UdpSocket();

protected:

  /**
   * \brief udp socket connect handshake value
   */
  static const char CONNECT_HANDSHAKE = 142;

  char udp_read_buffer_[MAX_BUFFER_SIZE + 1];
  char* udp_read_head_;
  size_t udp_read_len_;

  // Virtual
  int rawSendBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  int rawReceiveBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);
  bool rawPoll(int timeout, bool & ready, bool & error);

};

} //udp_socket
} //industrial

