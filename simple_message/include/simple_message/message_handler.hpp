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
#include "simple_message/simple_message.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#else
#include "simple_message.hpp"
#include "smpl_msg_connection.hpp"
#endif

namespace industrial
{
namespace message_handler
{

/**
 * \brief Interface definition for message handlers.  The interface defines the
 * callback function that should execute when a message is received.
 */
//* MessageHandler
/**
 * Defines the interface used for function callbacks when a message is received.
 * When used in conjunction with a message_manager (link) generic message handling
 * can be achieved.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class MessageHandler

{
public:

  /**
   * \brief Constructor
   */
  MessageHandler();

  /**
   * \brief Destructor
   */
  ~MessageHandler();

  /**
   * \brief Callback function that should be executed when a message arrives
   * DO NOT OVERRIDE THIS FUNCTION.  It performs message validation before the
   * internal callback (which should be overridden) is called.  If one is required
   * the callback sends a message reply
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool callback(industrial::simple_message::SimpleMessage & in);

  /**
   * \brief Gets message type that callback expects
   *
   * \return message type
   */
  int getMsgType()
  {
    return this->msg_type_;
  }
  ;

protected:
  /**
   * \brief Gets connectoin for message replies
   *
   * \return connection reference
   */
  industrial::smpl_msg_connection::SmplMsgConnection*getConnection()
  {
    return this->connection_;
  }
  ;

  /**
   * \brief Class initializer
   *
   * \param msg_type type of message expected
   * \param connection simple message connection that will be used to send replies.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection);

private:

  /**
   * \brief Reference to reply connection (called if incoming message requires a reply)
   */
  industrial::smpl_msg_connection::SmplMsgConnection* connection_;

  /**
   * \brief Message type expected by callback
   */
  int msg_type_;

  /**
   * \brief Virtual callback function
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  virtual bool internalCB(industrial::simple_message::SimpleMessage & in)=0;

  /**
   * \brief Validates incoming message for processing by internal callback
   *
   * \param in incoming message
   *
   * \return true on if valid, false otherwise
   */
  bool validateMsg(industrial::simple_message::SimpleMessage & in);

  /**
   * \brief Sets connection for message replies
   *
   * \param connection connection reference
   */
  void setConnection(industrial::smpl_msg_connection::SmplMsgConnection* connection)
  {
    this->connection_ = connection;
  }
  ;

  /**
   * \brief Sets message type that callback expects
   *
   * \param msg_type message type
   */
  void setMsgType(int msg_type)
  {
    this->msg_type_ = msg_type;
  }
  ;

};

} //namespace message_handler
} //namespace industrial