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

#include "rclcpp/rclcpp.hpp"
#include "motion_control_msgs/srv/get_motion_plan.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "std_srvs/srv/trigger.hpp"

class MoveItInterface : public rclcpp::Node
{
public:
    explicit MoveItInterface();

    ~MoveItInterface();
    void init();
    void run();

private:

    void getMotionPlan(const std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Request> req,
                       std::shared_ptr<motion_control_msgs::srv::GetMotionPlan::Response> res);

    void executeMotionPlan(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    const int DEFAULT_PLANNING_TIME = 5; // in seconds
    const double DEFAULT_VELOCITY_SCALING = 0.1;
    const double DEFAULT_ACCELERATION_SCALING = 0.1;
    rclcpp::Service<motion_control_msgs::srv::GetMotionPlan>::SharedPtr plan_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;
};