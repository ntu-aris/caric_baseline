/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "include/joy_position.h"

#include <mav_msgs/default_topics.h>

Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  pnh.param("axis_roll_", axes_.roll, 0);
  pnh.param("axis_pitch_", axes_.pitch, 1);
  pnh.param("axis_thrust_", axes_.thrust, 2);

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, -1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 10.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 45.0 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 30.0);  // [N]

  pnh.param("v_yaw_step", v_yaw_step_, 0.05);  // [rad/s]

  pnh.param("is_fixed_wing", is_fixed_wing_, false);

  pnh.param("button_yaw_left_", buttons_.yaw_left, 3);
  pnh.param("button_yaw_right_", buttons_.yaw_right, 4);
  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable, 5);
  pnh.param("button_ctrl_mode_", buttons_.ctrl_mode, 10);
  pnh.param("button_takeoff_", buttons_.takeoff, 7);
  pnh.param("button_land_", buttons_.land, 8);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
  odom_sub_ = nh_.subscribe("/ground_truth/odometry", 10, &Joy::OdomCallback, this);
  cmd_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory> (
  "/firefly/command/trajectory", 10);
  timer = nh_.createTimer(ros::Duration(1.0 / 10.0), &Joy::TimerCallback, this);
  gim_cmd_pub_ = nh_.advertise<geometry_msgs::Twist> ("/firefly/command/gimbal", 10);
}

void Joy::TimerCallback(const ros::TimerEvent &)
{
  if (!got_joy_) return;

  if ((ros::Time::now()-time_joy).toSec()>0.5)
  {
    yaw_cmd = tf_uav.yaw();
    xyz_cmd_world = Eigen::Vector3d(0.0, 0.0, 0.0);
    vertical_speed = 0.0;
  }

  // Eigen::Quaterniond q = Util::YPR2Quat(yaw_cmd, 0.0, 0.0);
  trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
  trajset_msg.header.stamp = ros::Time::now();
  trajset_msg.header.frame_id = "world";
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
  geometry_msgs::Transform transform_msg;
  geometry_msgs::Twist accel_msg, vel_msg;
  transform_msg.translation.x = tf_uav.pos(0);
  transform_msg.translation.y = tf_uav.pos(1);
  transform_msg.translation.z = tf_uav.pos(2);
  transform_msg.rotation.x = 0.0;
  transform_msg.rotation.y = 0.0;
  transform_msg.rotation.z = sinf(yaw_cmd/180.0*M_PI*0.5);
  transform_msg.rotation.w = cosf(yaw_cmd/180.0*M_PI*0.5);
  trajpt_msg.transforms.push_back(transform_msg);
  vel_msg.linear.x = xyz_cmd_world(0);
  vel_msg.linear.y = xyz_cmd_world(1);
  vel_msg.linear.z = vertical_speed;
  accel_msg.linear.x = 0.0;
  accel_msg.linear.x = 0.0;
  accel_msg.linear.x = 0.0;
  trajpt_msg.velocities.push_back(vel_msg);
  trajpt_msg.accelerations.push_back(accel_msg);
  trajset_msg.points.push_back(trajpt_msg);
  cmd_pub_.publish(trajset_msg);
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Joy::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_ = *msg;
}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  time_joy = ros::Time::now();
  myTf<double>tf_new(odom_);
  if (!got_joy_) tf_uav = tf_new;

  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;

  current_yaw_vel_ = 0;
  if (msg->axes[3]>0.02) {
    current_yaw_vel_ = max_.rate_yaw;
    tf_uav.rot = Util::YPR2Rot(tf_new.yaw(), 0.0, 0.0);
  }
  else if (msg->axes[3]<-0.02) {
    current_yaw_vel_ = -max_.rate_yaw;
    tf_uav.rot = Util::YPR2Rot(tf_new.yaw(), 0.0, 0.0);
  }
  else {
    current_yaw_vel_ = 0;
  }
  control_msg_.yaw_rate = current_yaw_vel_;
  yaw_cmd = tf_uav.yaw()/180.0*M_PI + current_yaw_vel_*0.3;
  yaw_cmd = Util::wrapTo180(yaw_cmd/M_PI*180.0);

  if (fabs(control_msg_.pitch)>0.01 || fabs(control_msg_.roll)>0.01)
  {
    tf_uav.pos(0) = tf_new.pos(0);
    tf_uav.pos(1) = tf_new.pos(1);
  }
     
  Eigen::Vector3d xyz_cmd_body(0.0, 0.0, 0.0);
  xyz_cmd_body(0) = control_msg_.pitch *5.0;
  xyz_cmd_body(1) = -control_msg_.roll * 5.0;

  control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  vertical_speed = -3.0*msg->axes[axes_.thrust]* axes_.thrust_direction;

  if (fabs(vertical_speed)>0.1) tf_uav.pos(2) = tf_new.pos(2);

  // Eigen::Quaterniond q = YPR2Quat(yaw_cmd, 0.0, 0.0);
  xyz_cmd_world = Util::YPR2Rot(tf_uav.yaw(), 0.0, 0.0)*xyz_cmd_body;
  got_joy_ = true;

  geometry_msgs::Twist gimmsg;
  gimmsg.linear.x = -1.0;
  gimmsg.linear.y = 0.0;
  gimmsg.linear.z = 0.0;
  gimmsg.angular.x = 0.0;
  gimmsg.angular.y = msg->axes[4];
  gimmsg.angular.z = msg->axes[5]; 
  gim_cmd_pub_.publish(gimmsg);
}

void Joy::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_interface");
  Joy joy;

  ros::spin();

  return 0;
}
