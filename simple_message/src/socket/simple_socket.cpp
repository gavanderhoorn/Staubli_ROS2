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
#include "simple_message/socket/simple_socket.hpp"
#else
#include "simple_socket.hpp"
#endif
#include "rclcpp/rclcpp.hpp"

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
  namespace simple_socket
  {

    bool SimpleSocket::sendBytes(ByteArray & buffer)
    {
      int rc = this->SOCKET_FAIL;
      bool rtn = false;

      if (this->isConnected())
      {
        // Nothing restricts the ByteArray from being larger than the what the socket
        // can handle.
        if (this->MAX_BUFFER_SIZE > (int)buffer.getBufferSize())
        {

          // copy to local array, since ByteArray no longer supports
          // direct pointer-access to data values
          std::vector<char> localBuffer;
          buffer.copyTo(localBuffer);
          rc = rawSendBytes(&localBuffer[0], localBuffer.size());
          if (this->SOCKET_FAIL != rc)
          {
            rtn = true;
          }
          else
          {
            rtn = false;
            logSocketError("Socket sendBytes failed", rc, errno);
          }

        }
        else
        {
          //RCLCPP_ERROR(rclcpp::get_logger("simple_socket"), "Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
          rtn = false;
        }

      }
      else
      {
        rtn = false;
        //RCLCPP_WARN(rclcpp::get_logger("simple_socket"), "Not connected, bytes not sent");
      }

      if (!rtn)
      {
        this->setConnected(false);
      }

      return rtn;

    }

    bool SimpleSocket::receiveBytes(ByteArray & buffer, shared_int num_bytes, shared_int timeout_ms)
    {
      int rc = this->SOCKET_FAIL;
      bool rtn = false;
      shared_int remainBytes = num_bytes;
      shared_int remainTimeMs = timeout_ms;
      bool ready, error;

      // Reset the buffer (this is not required since the buffer length should
      // ensure that we don't read any of the garbage that may be left over from
      // a previous read), but it is good practice.

      memset(&this->buffer_, 0, sizeof(this->buffer_));

      // Doing a sanity check to determine if the byte array buffer is smaller than
      // what can be received by the socket.
      if (this->MAX_BUFFER_SIZE > buffer.getMaxBufferSize())
      {
        //RCLCPP_WARN(rclcpp::get_logger("simple_socket"), "Socket buffer max size: %u, is larger than byte array buffer: %u", this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
      }
      if (this->isConnected())
      {
        buffer.init();
        while (remainBytes > 0 && (timeout_ms < 0 || remainTimeMs > 0))
        {
          // Polling the socket results in an "interruptable" socket read.  This
          // allows Control-C to break out of a socket read.  Without polling,
          // a sig-term is required to kill a program in a socket read function.
          if (this->rawPoll(this->SOCKET_POLL_TO, ready, error))
          {
            if(ready)
            {
              rc = rawReceiveBytes(this->buffer_, remainBytes);
              if (this->SOCKET_FAIL == rc)
              {
                this->logSocketError("Socket received failed", rc, errno);
                remainBytes = 0;
                rtn = false;
                break;
              }
              else if (0 == rc)
              {
                //RCLCPP_WARN(rclcpp::get_logger("simple_socket"), "Recieved zero bytes: %u", rc);
                remainBytes = 0;
                rtn = false;
                break;
              }
              else
              {
                remainBytes = remainBytes - rc;
                remainTimeMs = timeout_ms;  // Reset the timeout on successful read
                //RCLCPP_INFO(rclcpp::get_logger("simple_socket"), "Byte array receive, bytes read: %u, bytes reqd: %u, bytes left: %u", rc, num_bytes, remainBytes);
                buffer.load(&this->buffer_, rc);
                if (remainBytes <= 0) {
                  rtn = true;
                }
              }
            }
            else if(error)
            {
              //RCLCPP_ERROR(rclcpp::get_logger("simple_socket"), "Socket poll returned an error");
              rtn = false;
              break;
            }
            else
            {
              //RCLCPP_ERROR(rclcpp::get_logger("simple_socket"), "Uknown error from socket poll");
              rtn = false;
              break;
            }
          }
          else
          {
            remainTimeMs = remainTimeMs - this->SOCKET_POLL_TO;
            //RCLCPP_INFO(rclcpp::get_logger("simple_socket"), "Socket poll timeout, trying again");
          }
        }
      }
      else
      { 
        //RCLCPP_WARN(rclcpp::get_logger("simple_socket"), "Not connected, bytes not received");
        rtn = false;
      }

      // Close the socket on all failures except timeouts
      if (!rtn && (timeout_ms < 0 || remainTimeMs > 0))
      {
        this->setConnected(false);
      }
      return rtn;
    }

  }  //simple_socket
}  //industrial
