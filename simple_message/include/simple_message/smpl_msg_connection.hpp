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
#include "simple_message/byte_array.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"
#else
#include "byte_array.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#endif


namespace industrial
{
namespace smpl_msg_connection
{

/**
 * \brief Defines an interface and common methods for sending simple messages 
 * (see simple_message).  This interface makes a bare minimum of assumptions:
 *
 * 1. The connection is capable of sending raw bytes (encapsulated within a simple message)
 *
 * 2. The data connection has an explicit connect that establishes the connection (and an 
 *    associated disconnect method).  NOTE: For data connections that are connectionless,
 *    such as UDP, the connection method can be a NULL operation.
 */
class SmplMsgConnection

{
public:

  // Message

  /**
   * \brief Sends a message using the data connection
   *
   * \param message to send
   *
   * \return true if successful
   */
  virtual bool sendMsg(industrial::simple_message::SimpleMessage & message);

  /**
   * \brief Receives a message using the data connection
   *
   * \param populated with received message
   *
   * \return true if successful
   */
  virtual bool receiveMsg(industrial::simple_message::SimpleMessage & message);

  /**
   * \brief Receives a message using the data connection with a timeout.
   *
   * \param [out] message Populated with received message
   * \param [in] timeout_ms The timeout for receiving a message, in milliseconds
   *
   * \return true if successful
   */
  virtual bool receiveMsg(industrial::simple_message::SimpleMessage & message,
                          industrial::shared_types::shared_int timeout_ms);

  /**
   * \brief Performs a complete send and receive.  This is helpful when sending
   * a message that requires and explicit reply
   *
   * \param message to send
   * \param populated with received message
   * \param verbosity level of low level logging
   *
   * \return true if successful
   */
  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage & send,
                         industrial::simple_message::SimpleMessage & recv,
                         bool verbose = false);

  /**
   * \brief Performs a complete send and receive with a timeout.
   * This is helpful when sending a message that requires and explicit reply.
   *
   * \param [in] send The message to send
   * \param [out] recv Populated with received message
   * \param [in] timeout_ms The timeout for receiving a message, in milliseconds
   * \param [in] verbose Turn on low level logging
   *
   * \return true if successful
   */
  bool sendAndReceiveMsg(industrial::simple_message::SimpleMessage & send,
                         industrial::simple_message::SimpleMessage & recv,
                         industrial::shared_types::shared_int timeout_ms,
                         bool verbose = false);

  /**
   * \brief return connection status
   *
   * \return true if connected
   */
  virtual bool isConnected()=0;

  /**
   * \brief connects to the remote host
   *
   * \return true on success, false otherwise
   */
  virtual bool makeConnect()=0;

private:

  // Overrides
  /**
   * \brief Method used by send message interface method.  This should be overridden 
   * for the specific connection type
   *
   * \param data to send.
   *
   * \return true if successful
   */
  virtual bool sendBytes(industrial::byte_array::ByteArray & buffer) =0;

  /**
   * \brief Method used by receive message interface method.  This should be overridden
   * for the specific connection type.
   *
   * \param data to receive.
   * \param size (in bytes) of data to receive
   * \param timeout_ms Timeout to receive a message (in milliseconds). A negative timeout
   * means that this function should wait indefinitely.
   *
   * \return true if successful
   */
  virtual bool receiveBytes(industrial::byte_array::ByteArray & buffer,
                            industrial::shared_types::shared_int num_bytes,
                            industrial::shared_types::shared_int timeout_ms) = 0;
};

} //namespace message_connection
} //namespace industrial
