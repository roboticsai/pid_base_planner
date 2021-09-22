/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <tf2/utils.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/footprint_helper.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>

#include "PIDLocalPlanner.h"


PLUGINLIB_EXPORT_CLASS(pid_local_planner::PIDLocalPlanner, nav_core::BaseLocalPlanner)


namespace pid_local_planner {

PIDLocalPlanner::PIDLocalPlanner()
{
	pid.initPid(0.1, 0.01, 0.1, 0.3, -0.3);
}

PIDLocalPlanner::~PIDLocalPlanner()
{
}

void PIDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
}
	
bool PIDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	ROS_INFO("Setting global plan!");
	m_global_plan = plan;
	return true;
}

bool PIDLocalPlanner::isGoalReached()
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);
	ROS_INFO("Checking if goal is reached!");
	return false;
}

bool PIDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	boost::mutex::scoped_lock lock(m_odometry_mutex);
	ROS_INFO("Computing cmd_vel!");
	// double pow = pid.computeCommand(pow1_error,elapsed_time);
	return true;
}

} // neo_local_planner
