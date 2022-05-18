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
#include "simple_message/joint_data.hpp"
#else
#include "typed_message.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#include "joint_data.hpp"
#endif

namespace industrial
{
namespace joint_message
{

/**
 * \brief Enumeration of special sequence values that signal the end of trajectory
 * or an immediate stop.
 */
namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  END_TRAJECTORY = -1, STOP_TRAJECTORY = -2
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

/**
 * \brief Class encapsulated joint message generation methods (either to or
 * from a SimpleMessage type.  This message represents the joint position data.
 * NOTE: In earlier versions this was simply referred to as  JOINT message.  This
 * caused confusion as there are many types of joint messages (position, velocity,
 * feedback).  To remove confusion, this message was changed to JOINT_POSITION.
 * Other types of messages will have to be created for velocity and other feedback.
 *
 * The byte representation of a joint message is as follow (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   joints              (industrial::joint_data)                  40 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointMessage : public industrial::typed_message::TypedMessage
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint structure
   *
   * \param sequence number
   * \param joints
   *
   */
  void init(industrial::shared_types::shared_int seq, industrial::joint_data::JointData & joints);

  /**
   * \brief Initializes a new joint message
   *
   */
  void init();

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence);

  /**
   * \brief returns the maximum message sequence number
   *
   * \return message sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return sequence_;
  }

  /**
   * \brief returns reference to underlying joint class
   *
   * \return reference to joint class
   */
  industrial::joint_data::JointData& getJoints()
  {
    return this->joints_;
  }

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);

  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + this->joints_.byteLength();
  }

private:
  /**
   * \brief sequence number (for those joints messages that require it)
   */
  industrial::shared_types::shared_int sequence_;
  /**
   * \brief maximum number of joints positions that can be held in the message.
   */
  industrial::joint_data::JointData joints_;

};

}
}
