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

namespace industrial
{
namespace comms_fault_handler
{

/**
 * \brief Interface definition for communications fault handler.  Defines the type
 * of communcations faults that can be handled and the function callbacks that should
 * be executed under the specific fault conditions.
 *
 */
class CommsFaultHandler

{
public:

  /**
   * \brief Send failure callback method.  This method will be executed in the event
   * that a comms send fails.
   *
   */
  virtual void sendFailCB()=0;

  /**
   * \brief Receive failure callback method.  This method will be executed in the event
   * that a comms receive fails.
   */
  virtual void receiveFailCB()=0;

  /**
   * \brief Connection failure callback method.  This method will be exectured in the 
   * event that a comms connection is lost.
   *
   */
  virtual void connectionFailCB()=0;

};

} //namespace comms_fault_handler
} //namespace industrial