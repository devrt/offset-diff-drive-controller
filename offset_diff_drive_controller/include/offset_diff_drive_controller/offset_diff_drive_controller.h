/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, MID Academic Promotions, Inc.
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
 *   * Neither the name of the MID Academic Promotions nor the names of its
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

/*
 * Author: Yosuke Matsusaka
 */

#pragma once

#include <string>
#include <vector>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/multi_interface_controller.h>
#include <offset_diff_drive_controller/dynamics.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/tfMessage.h>

namespace offset_diff_drive_controller
{

    class OffsetDiffDriveController
        : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::VelocityJointInterface>
    {
    public:
        OffsetDiffDriveController();

        bool init(hardware_interface::RobotHW *hw,
                  ros::NodeHandle &root_nh,
                  ros::NodeHandle &controller_nh);

        void update(const ros::Time &time, const ros::Duration &period);

        void starting(const ros::Time &time);

        void stopping(const ros::Time &time);

    private:
        std::string name_;
        hardware_interface::RobotHW *robot_hw_;

        // Parameters of the vehicle:
        double wheel_separation_;
        double wheel_radius_;
        double wheel_offset_;
        double vel_limit_steer_;
        double vel_limit_wheel_;
        state state_;

        // Joint handles:
        std::string wheel_left_name_;
        std::string wheel_right_name_;
        std::string steer_name_;
        hardware_interface::JointHandle left_wheel_joint_;
        hardware_interface::JointHandle right_wheel_joint_;
        hardware_interface::JointHandle steer_joint_;
        hardware_interface::JointHandle odom_x_loopback_;
        hardware_interface::JointHandle odom_y_loopback_;
        hardware_interface::JointHandle odom_r_loopback_;

        // Odometry publishing:
        ros::Duration publish_period_;
        ros::Time last_state_publish_time_;
        bool enable_odom_tf_;
        bool enable_odom_loopback_;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage>> tf_odom_pub_;
        struct Odometry
        {
            double x;
            double y;
            double ang;

            Odometry() : x(0.0), y(0.0), ang(0.0) {}
        };
        Odometry odometry_;
        std::string base_frame_id_;
        std::string odom_frame_id_;

        // Velocity command subscribing:
        struct Command
        {
            double lin_x;
            double lin_y;
            double ang;
            ros::Time stamp;

            Command() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
        };
        realtime_tools::RealtimeBuffer<Command> command_;
        Command command_struct_;
        ros::Subscriber sub_command_;

        double cmd_vel_timeout_;

    protected:
        void cmdVelCallback(const geometry_msgs::Twist &command);

        joint_param joint_param_;
        cartesian_param cartesian_param_;

        double desired_abs_dot_x_;
        double desired_abs_dot_y_;
        double desired_dot_r_;
        double desired_steer_pos_;
    };

    PLUGINLIB_EXPORT_CLASS(offset_diff_drive_controller::OffsetDiffDriveController, controller_interface::ControllerBase);
} // namespace offset_diff_drive_controller