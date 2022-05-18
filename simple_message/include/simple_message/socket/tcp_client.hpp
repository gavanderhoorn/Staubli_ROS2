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
namespace tcp_client
{


/**
 * \brief Defines TCP client functions.
 */
class TcpClient : public industrial::tcp_socket::TcpSocket
{
public:

  /**
   * \brief Constructor
   */
  TcpClient();

  /**
   * \brief Destructor
   */
  ~TcpClient();

  /**
     * \brief initializes TCP client socket.  Object can either be a client OR
     * a server, NOT BOTH.
     *
     * \param buff server address (in string form) xxx.xxx.xxx.xxx
     * \param port_num port number (server & client port number must match)
     *
     * \return true on success, false otherwise (socket is invalid)
     */
    bool init(char *buff, int port_num);

    // Overrides
    bool makeConnect();

private:
    bool connectSocketHandle();


};

} //tcp_client
} //industrial

