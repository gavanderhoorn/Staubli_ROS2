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
#include "simple_message/socket/tcp_socket.hpp"
#else
#include "tcp_socket.hpp"
#endif

namespace industrial
{
namespace tcp_server
{

/**
 * \brief Defines TCP server functions.
 */
class TcpServer : public industrial::tcp_socket::TcpSocket
{
public:

  /**
   * \brief Constructor
   */
  TcpServer();

  /**
   * \brief Destructor
   */
  ~TcpServer();

  /**
   * \brief initializes TCP server socket.  The connect method must be called
   * following initialization in order to communicate with the remote host.
   *
   * \param port_num port number (server & client port number must match)
   *
   * \return true on success, false otherwise (socket is invalid)
   */
  bool init(int port_num);

  // Overrides
  bool makeConnect();

protected:
  /**
   * \brief server handle.  Every time a connection is made, the class generates
   * a new handle for sending/receiving.  The server handle is saved off to a
   * separate variable so that recoving a lost connection is possible.
   */
  int srvr_handle_;

  int getSrvrHandle() const
  {
    return srvr_handle_;
  }

  void setSrvrHandle(int srvr_handle_)
  {
    this->srvr_handle_ = srvr_handle_;
  }
};

} //simple_socket
} //industrial