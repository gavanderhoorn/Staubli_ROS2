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
#include "simple_message/simple_serialize.hpp"
#include "simple_message/shared_types.hpp"
#include "simple_message/joint_traj_pt.hpp"
#else
#include "simple_message.hpp"
#include "simple_serialize.hpp"
#include "shared_types.hpp"
#include "joint_traj_pt.hpp"
#endif



namespace industrial
{
namespace joint_traj
{

/**
 * \brief Class encapsulated joint trajectory.  A joint trajectory includes
 * an array of JointTrajPt data types.  The intention for this class is to
 * be loaded into a single message for communication over a simple connection.
 *
 * For simplicity and cross platform compliance, this is implemented as a
 * fixed size array.  The size of the trajectory cannot exceed the max size
 * of the array.
 */
//* JointTraj
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTraj : public industrial::simple_serialize::SimpleSerialize
{
public:
	/**
	 * \brief Default constructor
	 *
	 * This method creates empty data.
	 *
	 */
	JointTraj(void);
	/**
	 * \brief Destructor
	 *
	 */
	~JointTraj(void);

	/**
	 * \brief Initializes a empty joint data
	 *
	 */
	void init();

	/**
	 * \brief Adds a point value to the end of the buffer
	 *
	 * \param point value
	 *
	 * \return true if value set, otherwise false (buffer is full)
	 */
	bool addPoint(industrial::joint_traj_pt::JointTrajPt & point);

	/**
	 * \brief Gets a point value within the buffer
	 *
	 * \param point index
	 * \param point value
	 *
	 * \return true if value set, otherwise false (index greater than size)
	 */
	bool getPoint(industrial::shared_types::shared_int index,
			industrial::joint_traj_pt::JointTrajPt & point);
	/**
	 * \brief Gets a size of trajectory
	 *
	 * \return trajectory size
	 */
	industrial::shared_types::shared_int size()
	{
		return this->size_;
	}

	/**
	 * \brief returns True if buffer is full
	 *
	 * \return true if buffer is full
	 */
	bool isFull()
	{
		return this->size_ >= this->getMaxNumPoints();
	}

	/**
	 * \brief returns the maximum number of points the message holds
	 *
	 * \return max number of points
	 */
	int getMaxNumPoints() const
	{
		return MAX_NUM_POINTS;
	}

	/**
	 * \brief Copies the passed in value
	 *
	 * \param src (value to copy)
	 */
	void copyFrom(JointTraj &src);

	/**
	 * \brief == operator implementation
	 *
	 * \return true if equal
	 */
	bool operator==(JointTraj &rhs);

	// Overrides - SimpleSerialize
	bool load(industrial::byte_array::ByteArray *buffer);
	bool unload(industrial::byte_array::ByteArray *buffer);
	unsigned int byteLength()
	{
		industrial::joint_traj_pt::JointTrajPt pt;
		return this->size() * pt.byteLength();
	}

private:

	/**
	 * \brief maximum number of joints positions that can be held in the message.
	 */
	static const industrial::shared_types::shared_int MAX_NUM_POINTS = 200;
	/**
	 * \brief internal data buffer
	 */
	industrial::joint_traj_pt::JointTrajPt points_[MAX_NUM_POINTS];
	/**
	 * \brief size of trajectory
	 */
	industrial::shared_types::shared_int size_;

};

}
}
