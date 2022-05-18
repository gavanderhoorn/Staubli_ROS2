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
#include "simple_message/joint_traj_pt.hpp"
#else
#include "typed_message.hpp"
#include "simple_message.hpp"
#include "shared_types.hpp"
#include "joint_traj_pt.hpp"
#endif

namespace industrial
{
namespace joint_traj_pt_message
{


/**
 * \brief Class encapsulated joint trajectory point message generation methods
 * (either to or from a industrial::simple_message::SimpleMessage type.
 *
 * This message simply wraps the industrial::joint_traj_pt::JointTrajPt data type.
 * The data portion of this typed message matches JointTrajPt.
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtMessage : public industrial::typed_message::TypedMessage

{
public:
  /**
   * \brief Default constructor
   *
   * This method creates an empty message.
   *
   */
  JointTrajPtMessage(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtMessage(void);
  /**
   * \brief Initializes message from a simple message
   *
   * \param simple message to construct from
   *
   * \return true if message successfully initialized, otherwise false
   */
  bool init(industrial::simple_message::SimpleMessage & msg);

  /**
   * \brief Initializes message from a joint trajectory point structure
   *
   * \param joint trajectory point data structure
   *
   */
  void init(industrial::joint_traj_pt::JointTrajPt & point);

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
    return this->point_.byteLength();
  }

  /**
   * \brief Sets message sequence number
   *
   * \param message sequence number
   */
  void setSequence(industrial::shared_types::shared_int sequence) { point_.setSequence(sequence); }

  industrial::joint_traj_pt::JointTrajPt point_;

private:


};

}
}