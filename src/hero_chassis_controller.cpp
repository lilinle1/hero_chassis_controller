// Copyright 2026 lilinle1
// BSD 3-Clause License

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Read geometric and frame parameters from config/controllers.yaml
  controller_nh.param("wheel_track", wheel_track_, 0.4);
  controller_nh.param("wheel_base", wheel_base_, 0.4);
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");

  std::string lf_name, rf_name, lb_name, rb_name;
  controller_nh.param<std::string>("left_front_joint", lf_name, "left_front_wheel_joint");
  controller_nh.param<std::string>("right_front_joint", rf_name, "right_front_wheel_joint");
  controller_nh.param<std::string>("left_back_joint", lb_name, "left_back_wheel_joint");
  controller_nh.param<std::string>("right_back_joint", rb_name, "right_back_wheel_joint");

  // Get joint handles
  front_left_joint_ = effort_joint_interface->getHandle(lf_name);
  front_right_joint_ = effort_joint_interface->getHandle(rf_name);
  back_left_joint_ = effort_joint_interface->getHandle(lb_name);
  back_right_joint_ = effort_joint_interface->getHandle(rb_name);

  // Initialize 4 PID controllers from parameter server (enables rqt_reconfigure)
  if (!pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")))
  {
    ROS_ERROR("Failed to initialize wheel PID controllers.");
    return false;
  }

  // Subscribe to /cmd_vel and advertise /odom
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 10);
  debug_vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("wheel_vel_debug", 10);

  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  pid_lf_.reset();
  pid_rf_.reset();
  pid_lb_.reset();
  pid_rb_.reset();
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  last_cmd_time_ = time;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_ = *msg;
  last_cmd_time_ = ros::Time::now();
}

void HeroChassisController::computeInverseKinematics(double vx, double vy, double wz)
{
  double half_sum = (wheel_track_ + wheel_base_) / 2.0;
  desired_vel_lf_ = (vx - vy - half_sum * wz) / wheel_radius_;
  desired_vel_rf_ = (vx + vy + half_sum * wz) / wheel_radius_;
  desired_vel_lb_ = (vx + vy - half_sum * wz) / wheel_radius_;
  desired_vel_rb_ = (vx - vy + half_sum * wz) / wheel_radius_;
}

void HeroChassisController::updateOdometry(const ros::Time& time, const ros::Duration& period)
{
  double w_lf = front_left_joint_.getVelocity();
  double w_rf = front_right_joint_.getVelocity();
  double w_lb = back_left_joint_.getVelocity();
  double w_rb = back_right_joint_.getVelocity();

  // Forward Kinematics: 4 wheel actual velocities -> chassis actual twist in base_link
  double half_sum = (wheel_track_ + wheel_base_) / 2.0;
  double vx_actual = (w_lf + w_rf + w_lb + w_rb) * wheel_radius_ / 4.0;
  double vy_actual = (-w_lf + w_rf + w_lb - w_rb) * wheel_radius_ / 4.0;
  double wz_actual = (-w_lf + w_rf - w_lb + w_rb) * wheel_radius_ / (4.0 * half_sum);

  // Integrate velocity over time (dt) in the global "odom" frame
  double dt = period.toSec();
  double delta_x = (vx_actual * std::cos(odom_yaw_) - vy_actual * std::sin(odom_yaw_)) * dt;
  double delta_y = (vx_actual * std::sin(odom_yaw_) + vy_actual * std::cos(odom_yaw_)) * dt;
  double delta_yaw = wz_actual * dt;

  odom_x_ += delta_x;
  odom_y_ += delta_y;
  odom_yaw_ += delta_yaw;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);
  geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

  // 1. Broadcast TF transform from "odom" to "base_link"
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp = time;
  odom_tf.header.frame_id = odom_frame_;
  odom_tf.child_frame_id = base_frame_;
  odom_tf.transform.translation.x = odom_x_;
  odom_tf.transform.translation.y = odom_y_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  tf_broadcaster_.sendTransform(odom_tf);

  // 2. Publish nav_msgs::Odometry message on "/odom"
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;
  odom_msg.pose.pose.position.x = odom_x_;
  odom_msg.pose.pose.position.y = odom_y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.twist.twist.linear.x = vx_actual;
  odom_msg.twist.twist.linear.y = vy_actual;
  odom_msg.twist.twist.angular.z = wz_actual;
  odom_pub_.publish(odom_msg);
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // Update forward kinematics odometry and broadcast TF
  updateOdometry(time, period);

  double vx = cmd_vel_.linear.x;
  double vy = cmd_vel_.linear.y;
  double wz = cmd_vel_.angular.z;

  // Safety timeout: stop chassis if no /cmd_vel received recently
  if ((time - last_cmd_time_).toSec() > cmd_vel_timeout_)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  // Inverse Kinematics: chassis twist -> 4 wheel target velocities
  computeInverseKinematics(vx, vy, wz);

  // Closed-loop PID velocity control for each wheel
  double actual_lf = front_left_joint_.getVelocity();
  double actual_rf = front_right_joint_.getVelocity();
  double actual_lb = back_left_joint_.getVelocity();
  double actual_rb = back_right_joint_.getVelocity();

  double cmd_lf = pid_lf_.computeCommand(desired_vel_lf_ - actual_lf, period);
  double cmd_rf = pid_rf_.computeCommand(desired_vel_rf_ - actual_rf, period);
  double cmd_lb = pid_lb_.computeCommand(desired_vel_lb_ - actual_lb, period);
  double cmd_rb = pid_rb_.computeCommand(desired_vel_rb_ - actual_rb, period);

  front_left_joint_.setCommand(cmd_lf);
  front_right_joint_.setCommand(cmd_rf);
  back_left_joint_.setCommand(cmd_lb);
  back_right_joint_.setCommand(cmd_rb);

  // Publish target [0..3] and actual [4..7] wheel velocities for rqt_plot
  std_msgs::Float64MultiArray debug_msg;
  debug_msg.data = {desired_vel_lf_, desired_vel_rf_, desired_vel_lb_, desired_vel_rb_,
                    actual_lf, actual_rf, actual_lb, actual_rb};
  debug_vel_pub_.publish(debug_msg);
}

void HeroChassisController::stopping(const ros::Time& /*time*/)
{
  front_left_joint_.setCommand(0.0);
  front_right_joint_.setCommand(0.0);
  back_left_joint_.setCommand(0.0);
  back_right_joint_.setCommand(0.0);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
