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
#include "simple_message/comms_fault_handler.hpp"
#include "simple_message/smpl_msg_connection.hpp"
#else
#include "comms_fault_handler.hpp"
#include "smpl_msg_connection.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

namespace industrial
{
namespace simple_comms_fault_handler
{

/**
 * \brief Default implementation of comms fault handler.  This class attempts
 * to reconnect if the connection is lost.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class SimpleCommsFaultHandler : public industrial::comms_fault_handler::CommsFaultHandler

{
public:

  /**
      * \brief Default constructor
      *
      */
  SimpleCommsFaultHandler();

  /**
       * \brief Destructor
       *
       */
  ~SimpleCommsFaultHandler();
  /**
    * \brief Initializes default communications fault handler
    *
    * \param message connection to use for reconnecting
    *
    * \return true on success, false otherwise
    */
   bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection);


   /**
      * \brief Send failure callback method: Nothing is performed
      *
      */
     void sendFailCB() {RCLCPP_WARN(rclcpp::get_logger("simple_comms_fault_handler"), "Send failure, no callback support");};

     /**
      * \brief Receive failure callback method: Nothing is performed
      *
      */
     void receiveFailCB() {RCLCPP_WARN(rclcpp::get_logger("simple_comms_fault_handler"), "Receive failure, no callback support");};

     /**
      * \brief Connection failure callback method: On a connection failure
      * a blocking reconnection is attempted.
      *
      */
     void connectionFailCB();

private:


/**
 * \brief Reference to reply connection (called if incoming message requires a reply)
 */
industrial::smpl_msg_connection::SmplMsgConnection* connection_;

/**
   * \brief Sets connection manager
   *
   * \param connection connection reference
   */
  void setConnection(industrial::smpl_msg_connection::SmplMsgConnection* connection)
  {
    this->connection_ = connection;
  }
  ;

  /**
   * \brief Gets connection for manager
   *
   * \return connection reference
   */
  industrial::smpl_msg_connection::SmplMsgConnection* getConnection()
  {
    return this->connection_;
  }
};

} //namespace default_comms_fault_handler
} //namespace industrial
