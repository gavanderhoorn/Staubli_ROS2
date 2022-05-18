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
#include "simple_message/byte_array.hpp"
#else
#include "simple_message.hpp"
#include "byte_array.hpp"
#endif


namespace industrial
{
namespace typed_message
{

/**
 * \brief Message interface for typed messages built from SimpleMessage.
 *
 * This is an interface for a helper class that when implemented is used
 * to create simple messages of the various types (i.e. as defined by the
 * message type enumeration).  It also has constructors and initializers 
 * that can be used to create a typed message from a simple message.
 *
 * If the typed message does not support a particular simple message type
 * the "to" method should be overridden to return false.  For exmaple, a
 * ping message cannot be a topic, it is always expected to be a request/
 * reply.  A joint trajectory point on the other hand may either be a topic
 * (i.e. asynchronously sent) or a request/reply (i.e. syncrounously sent)
 *
 * Classes that implement this interface shall include data members for
 * the data payload of the typed message.
 *
 * \deprecated The base function implementations in the class will be removed
 * in a later release.  This will force classes that inherit from this
 * class to implement them.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class TypedMessage : public industrial::simple_serialize::SimpleSerialize
{

public:
  /**
   * \brief Initializes message from a simple message
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool init(industrial::simple_message::SimpleMessage & msg)=0;

  /**
   * \brief Initializes a new empty message
   *
   */
  virtual void init()=0;

  /**
   * \brief creates a simple_message request
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toRequest(industrial::simple_message::SimpleMessage & msg)
  {
	  industrial::byte_array::ByteArray data;
	  data.load(*this);
	  return msg.init(this->getMessageType(),
			  industrial::simple_message::CommTypes::SERVICE_REQUEST,
			  industrial::simple_message::ReplyTypes::INVALID, data);
  }

  /**
   * \brief creates a simple_message reply
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toReply(industrial::simple_message::SimpleMessage & msg,
		  industrial::simple_message::ReplyType reply)
  {
	  industrial::byte_array::ByteArray data;
	data.load(*this);
	return msg.init(this->getMessageType(),
			industrial::simple_message::CommTypes::SERVICE_REPLY,
			reply, data);
  }
  /**
   * \brief creates a simple_message topic
   *
   * \return true if message successfully initialized, otherwise false
   */
  virtual bool toTopic(industrial::simple_message::SimpleMessage & msg)
  {
	  industrial::byte_array::ByteArray data;
    data.load(*this);
    return msg.init(this->getMessageType(),
    		industrial::simple_message::CommTypes::TOPIC,
    		industrial::simple_message::ReplyTypes::INVALID, data);
  }
  /**
   * \brief gets message type (enumeration)
   *
   * \return message type
   */
  int getMessageType() const
  {
    return message_type_;
  }
  
  /**
   * \brief Gets the communication type of the message
   * 
   * \return the value of the comm_type parameter (refer to simple_message::CommTypes::CommType)
   */
  int getCommType() const
  {
    return comm_type_;
  }

protected:

  /**
   * \brief sets message type
   *
   * \return message type
   */
  void setMessageType(int message_type = industrial::simple_message::StandardMsgTypes::INVALID)
  {
    this->message_type_ = message_type;
  }
  
  /**
   * \brief Sets the communication type of the message
   *
   * \param comm_type: value of the comm_type parameter (refer to simple_message::CommTypes::CommType)
   */
  void setCommType(int comm_type = industrial::simple_message::CommTypes::INVALID)
  {
    this->comm_type_ = comm_type;
  }

private:

  /**
   * \brief Message type expected by callback
   */

  int message_type_;
    
  /**
   * \brief Communications type (see simple_message::CommTypes::CommType)
   */
  int comm_type_;

};

}
}

