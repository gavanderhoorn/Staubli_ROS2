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
#include "simple_message/message_handler.hpp"
#else
#include "message_handler.hpp"
#endif


namespace industrial
{
namespace ping_handler
{

/**
 * \brief Message handler that handles ping messages.
 */
//* MessageHandler
/**
 * Responds to ping message types.  A ping is a simple message that is meant to
 * test communications channels.  A ping simply responds with a copy of the data
 * it was sent.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class PingHandler : public industrial::message_handler::MessageHandler
{

public:
  /**
* \brief Class initializer
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);

  /**
* \brief Class initializer (Direct call to base class with the same name)
* I couldn't get the "using" form to work/
*
* \param connection simple message connection that will be used to send replies.
*
* \return true on success, false otherwise (an invalid message type)
*/
bool init(int msg_type, industrial::smpl_msg_connection::SmplMsgConnection* connection)
{ return MessageHandler::init(msg_type, connection);};


private:



 /**
  * \brief Callback executed upon receiving a ping message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage & in);
};

}//ping_handler
}//industrial

