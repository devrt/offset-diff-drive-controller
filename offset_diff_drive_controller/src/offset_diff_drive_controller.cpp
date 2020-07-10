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

#include <cmath>
#include <offset_diff_drive_controller/offset_diff_drive_controller.h>
#include <tf/transform_datatypes.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

namespace offset_diff_drive_controller
{

    OffsetDiffDriveController::OffsetDiffDriveController()
        : command_struct_(), wheel_separation_(0.0), wheel_radius_(0.0), wheel_offset_(0.0), desired_steer_pos_(0.0), base_frame_id_("base_link"), odom_frame_id_("odom"), enable_odom_tf_(true), wheel_left_name_(""), wheel_right_name_(""), steer_name_(""), vel_limit_steer_(8.0), vel_limit_wheel_(8.0)
    {
    }

    bool OffsetDiffDriveController::init(hardware_interface::RobotHW *hw,
                                         ros::NodeHandle &root_nh,
                                         ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        // Read parameters
        double publish_rate;
        controller_nh.param("publish_rate", publish_rate, 50.0);
        ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at " << publish_rate << "Hz.");
        publish_period_ = ros::Duration(1.0 / publish_rate);

        controller_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

        controller_nh.param("odom_frame_id", odom_frame_id_, odom_frame_id_);
        ROS_INFO_STREAM_NAMED(name_, "Odometry frame_id set to " << odom_frame_id_);

        controller_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
        ROS_INFO_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_ ? "enabled" : "disabled"));

        controller_nh.param("wheel_separation", wheel_separation_, wheel_separation_);
        ROS_INFO_STREAM_NAMED(name_, "Wheel separation set to " << wheel_separation_ << "[m]");

        controller_nh.param("wheel_radius", wheel_radius_, wheel_radius_);
        ROS_INFO_STREAM_NAMED(name_, "Wheel radius set to " << wheel_radius_ << "[m]");

        controller_nh.param("wheel_offset", wheel_offset_, wheel_offset_);
        ROS_INFO_STREAM_NAMED(name_, "Wheel offset set to " << wheel_offset_ << "[m]");

        controller_nh.param("left_wheel", wheel_left_name_, wheel_left_name_);
        ROS_INFO_STREAM_NAMED(name_, "Left wheel joint name set to " << wheel_left_name_);

        controller_nh.param("right_wheel", wheel_right_name_, wheel_right_name_);
        ROS_INFO_STREAM_NAMED(name_, "Right wheel joint name set to " << wheel_right_name_);

        controller_nh.param("steer", steer_name_, steer_name_);
        ROS_INFO_STREAM_NAMED(name_, "Steering joint name set to " << steer_name_);

        controller_nh.param("vel_limit_wheel", vel_limit_wheel_, vel_limit_wheel_);
        ROS_INFO_STREAM_NAMED(name_, "Wheel velocity limit set to " << vel_limit_wheel_);

        controller_nh.param("vel_limit_steer", vel_limit_steer_, vel_limit_steer_);
        ROS_INFO_STREAM_NAMED(name_, "Steering velocity limit set to " << vel_limit_steer_);

        state_.steer_angle = 0.0;
        state_.wheel_offset = wheel_offset_;
        state_.wheel_radius = wheel_radius_;
        state_.wheel_separation = wheel_separation_;

        // Get joint interface handlers
        left_wheel_joint_ = hw->get<hardware_interface::VelocityJointInterface>()->getHandle(wheel_left_name_);
        right_wheel_joint_ = hw->get<hardware_interface::VelocityJointInterface>()->getHandle(wheel_right_name_);
        steer_joint_ = hw->get<hardware_interface::PositionJointInterface>()->getHandle(steer_name_);

        // Read and check settings for odometry covariances
        XmlRpc::XmlRpcValue pose_cov_list;
        controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
        ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(pose_cov_list.size() == 6);
        for (int i = 0; i < pose_cov_list.size(); ++i)
            ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        XmlRpc::XmlRpcValue twist_cov_list;
        controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
        ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(twist_cov_list.size() == 6);
        for (int i = 0; i < twist_cov_list.size(); ++i)
            ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

        // Setup odometry publisher
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
        odom_pub_->msg_.header.frame_id = odom_frame_id_;
        odom_pub_->msg_.child_frame_id = base_frame_id_;
        odom_pub_->msg_.pose.pose.position.z = 0;
        odom_pub_->msg_.pose.covariance = {
            static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5])};
        odom_pub_->msg_.twist.twist.linear.y = 0;
        odom_pub_->msg_.twist.twist.linear.z = 0;
        odom_pub_->msg_.twist.twist.angular.x = 0;
        odom_pub_->msg_.twist.twist.angular.y = 0;
        odom_pub_->msg_.twist.covariance = {
            static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
            0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
            0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
            0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
            0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
            0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5])};
        tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
        tf_odom_pub_->msg_.transforms.resize(1);
        tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
        tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
        tf_odom_pub_->msg_.transforms[0].header.frame_id = odom_frame_id_;

        // Subscribe to cmd_vel topic
        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &OffsetDiffDriveController::cmdVelCallback, this);

        return true;
    }

    void OffsetDiffDriveController::update(const ros::Time &time, const ros::Duration &period)
    {
        double left_pos = left_wheel_joint_.getPosition();
        double right_pos = right_wheel_joint_.getPosition();
        double steer_pos = steer_joint_.getPosition();
        double left_vel = left_wheel_joint_.getVelocity();
        double right_vel = right_wheel_joint_.getVelocity();
        double steer_vel = steer_joint_.getVelocity();
        double dt = period.toSec();

        state_.steer_angle = steer_pos;
        joint_param_.vel_steer = steer_vel;
        joint_param_.vel_wheel_l = left_vel;
        joint_param_.vel_wheel_r = right_vel;

        // Calculate cartesian space velocities by using forward dynamics equations
        forward_dynamics(joint_param_, state_, cartesian_param_);

        // Integrate velocities to update wheel odometry
        double x = cartesian_param_.dot_x * cos(odometry_.ang) - cartesian_param_.dot_y * sin(odometry_.ang);
        double y = cartesian_param_.dot_x * sin(odometry_.ang) + cartesian_param_.dot_y * cos(odometry_.ang);
        odometry_.x += x * dt;
        odometry_.y += y * dt;
        odometry_.ang += cartesian_param_.dot_r * dt;

        // Publish odometry message
        if (last_state_publish_time_ + publish_period_ < time)
        {
            last_state_publish_time_ += publish_period_;

            const geometry_msgs::Quaternion orientation(
                tf::createQuaternionMsgFromYaw(odometry_.ang));

            // Publish odometry message (from realtime to non-realtime)
            if (odom_pub_->trylock())
            {
                odom_pub_->msg_.header.stamp = time;
                odom_pub_->msg_.pose.pose.position.x = odometry_.x;
                odom_pub_->msg_.pose.pose.position.y = odometry_.y;
                odom_pub_->msg_.pose.pose.orientation = orientation;
                odom_pub_->msg_.twist.twist.linear.x = x;
                odom_pub_->msg_.twist.twist.linear.y = y;
                odom_pub_->msg_.twist.twist.angular.z = cartesian_param_.dot_r;
                odom_pub_->unlockAndPublish();
            }

            // Publish tf /odom frame (from realtime to non-realtime)
            if (enable_odom_tf_ && tf_odom_pub_->trylock())
            {
                geometry_msgs::TransformStamped &odom_frame = tf_odom_pub_->msg_.transforms[0];
                odom_frame.header.stamp = time;
                odom_frame.transform.translation.x = odometry_.x;
                odom_frame.transform.translation.y = odometry_.y;
                odom_frame.transform.rotation = orientation;
                tf_odom_pub_->unlockAndPublish();
            }
        }

        // Read velocity command from non-realtime process
        Command curr_cmd = *(command_.readFromRT());

        // Compute wheel velocities
        cartesian_param_.dot_x = curr_cmd.lin_x;
        cartesian_param_.dot_y = curr_cmd.lin_y;
        cartesian_param_.dot_r = curr_cmd.ang;
        inverse_dynamics(cartesian_param_, state_, joint_param_);

        // Apply speed limit
        double ratio = std::fabs(joint_param_.vel_steer) / vel_limit_steer_;
        ratio = std::max(ratio, std::fabs(joint_param_.vel_wheel_l) / vel_limit_wheel_);
        ratio = std::max(ratio, std::fabs(joint_param_.vel_wheel_r) / vel_limit_wheel_);
        if (ratio > 1.0)
        {
            joint_param_.vel_steer /= ratio;
            joint_param_.vel_wheel_l /= ratio;
            joint_param_.vel_wheel_r /= ratio;
        }

        // Set wheel velocities
        left_wheel_joint_.setCommand(joint_param_.vel_wheel_l);
        right_wheel_joint_.setCommand(joint_param_.vel_wheel_r);

        // Set steer position
        desired_steer_pos_ += joint_param_.vel_steer * dt;
        steer_joint_.setCommand(desired_steer_pos_);
    }

    void OffsetDiffDriveController::starting(const ros::Time &time)
    {
        left_wheel_joint_.setCommand(0.0);
        right_wheel_joint_.setCommand(0.0);

        last_state_publish_time_ = time;
        state_.steer_angle = steer_joint_.getPosition();
        desired_steer_pos_ = steer_joint_.getPosition();
    }

    void OffsetDiffDriveController::stopping(const ros::Time &time)
    {
        left_wheel_joint_.setCommand(0.0);
        right_wheel_joint_.setCommand(0.0);
    }

    void OffsetDiffDriveController::cmdVelCallback(const geometry_msgs::Twist &command)
    {
        if (isRunning())
        {
            // Send command from non-realtime callback process to realtime control process
            command_struct_.ang = command.angular.z;
            command_struct_.lin_x = command.linear.x;
            command_struct_.lin_y = command.linear.y;
            command_struct_.stamp = ros::Time::now();
            command_.writeFromNonRT(command_struct_);
        }
    }
} // namespace offset_diff_drive_controller