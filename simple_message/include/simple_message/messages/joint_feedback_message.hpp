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
#include "simple_message/typed_message.hpp"
#include "simple_message/simple_message.hpp"
#include "simple_message/shared_types.hpp"
#include "simple_message/joint_feedback.hpp"
#else
#include "typed_message.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#include "joint_feedback.hpp"
#endif

namespace industrial
{
namespace joint_feedback_message
{


/**
 * \brief Class encapsulated joint feedback message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::joint_feedback::JointFeedback data type.
 * The data portion of this typed message matches JointFeedback.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointFeedbackMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointFeedbackMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointFeedbackMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint feedback structure
   *
   * \param joint feedback data structure
   *
   */
  void init(industrial::joint_feedback::JointFeedback & data);

  /**
   * \brief Initializes a new message
   *
   */
  void init();

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return this->data_.byteLength();
  }

  industrial::shared_types::shared_int getRobotID()
  {
    return this->data_.getRobotID();
  }

  bool getTime(industrial::shared_types::shared_real & time)
  {
    return this->data_.getTime(time);
  }

  bool getPositions(industrial::joint_data::JointData &dest)
  {
    return this->data_.getPositions(dest);
  }

  bool getVelocities(industrial::joint_data::JointData &dest)
  {
    return this->data_.getVelocities(dest);
  }

  bool getAccelerations(industrial::joint_data::JointData &dest)
  {
    return this->data_.getAccelerations(dest);
  }

private:

  industrial::joint_feedback::JointFeedback data_;

};

}
}