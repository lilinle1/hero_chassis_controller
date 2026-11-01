// Copyright 2026 lilinle1
// BSD 3-Clause License

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  controller_nh.param("wheel_track", wheel_track_, 0.4);
  controller_nh.param("wheel_base", wheel_base_, 0.4);
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  controller_nh.param<std::string>("odom_frame", odom_frame_, "odom");
  controller_nh.param<std::string>("base_frame", base_frame_, "base_link");
  controller_nh.param<std::string>("velocity_mode", velocity_mode_, "base");
  controller_nh.param("enable_acc_limit", enable_acc_limit_, true);
  controller_nh.param("max_linear_acc", max_linear_acc_, 4.0);
  controller_nh.param("max_angular_acc", max_angular_acc_, 6.0);

  std::string lf_name, rf_name, lb_name, rb_name;
  controller_nh.param<std::string>("left_front_joint", lf_name, "left_front_wheel_joint");
  controller_nh.param<std::string>("right_front_joint", rf_name, "right_front_wheel_joint");
  controller_nh.param<std::string>("left_back_joint", lb_name, "left_back_wheel_joint");
  controller_nh.param<std::string>("right_back_joint", rb_name, "right_back_wheel_joint");

  front_left_joint_ = effort_joint_interface->getHandle(lf_name);
  front_right_joint_ = effort_joint_interface->getHandle(rf_name);
  back_left_joint_ = effort_joint_interface->getHandle(lb_name);
  back_right_joint_ = effort_joint_interface->getHandle(rb_name);

  if (!pid_lf_.init(ros::NodeHandle(controller_nh, "pid_lf")) ||
      !pid_rf_.init(ros::NodeHandle(controller_nh, "pid_rf")) ||
      !pid_lb_.init(ros::NodeHandle(controller_nh, "pid_lb")) ||
      !pid_rb_.init(ros::NodeHandle(controller_nh, "pid_rb")))
  {
    ROS_ERROR("Failed to initialize wheel PID controllers.");
    return false;
  }

  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("/odom", 10);
  target_vel_pub_ = controller_nh.advertise<std_msgs::Float64>("lf_target_vel", 10);
  actual_vel_pub_ = controller_nh.advertise<std_msgs::Float64>("lf_actual_vel", 10);

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
  ramped_vx_ = 0.0;
  ramped_vy_ = 0.0;
  ramped_wz_ = 0.0;
  last_cmd_time_ = time;
  new_cmd_received_ = false;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_ = *msg;
  new_cmd_received_ = true;
}

double HeroChassisController::applyAccLimit(double target, double current, double max_acc, double dt)
{
  double max_delta = max_acc * dt;
  double diff = target - current;
  if (std::abs(diff) > max_delta)
  {
    return current + std::copysign(max_delta, diff);
  }
  return target;
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

  double half_sum = (wheel_track_ + wheel_base_) / 2.0;
  double vx_actual = (w_lf + w_rf + w_lb + w_rb) * wheel_radius_ / 4.0;
  double vy_actual = (-w_lf + w_rf + w_lb - w_rb) * wheel_radius_ / 4.0;
  double wz_actual = (-w_lf + w_rf - w_lb + w_rb) * wheel_radius_ / (4.0 * half_sum);

  double dt = period.toSec();
  if (dt > 0.0 && dt < 0.5)
  {
    odom_x_ += (vx_actual * std::cos(odom_yaw_) - vy_actual * std::sin(odom_yaw_)) * dt;
    odom_y_ += (vx_actual * std::sin(odom_yaw_) + vy_actual * std::cos(odom_yaw_)) * dt;
    odom_yaw_ += wz_actual * dt;
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);
  geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header.stamp = time;
  odom_tf.header.frame_id = odom_frame_;
  odom_tf.child_frame_id = base_frame_;
  odom_tf.transform.translation.x = odom_x_;
  odom_tf.transform.translation.y = odom_y_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  tf_broadcaster_.sendTransform(odom_tf);

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
  if (new_cmd_received_)
  {
    last_cmd_time_ = time;
    new_cmd_received_ = false;
  }

  updateOdometry(time, period);

  double target_vx = cmd_vel_.linear.x;
  double target_vy = cmd_vel_.linear.y;
  double target_wz = cmd_vel_.angular.z;

  if (std::abs((time - last_cmd_time_).toSec()) > cmd_vel_timeout_)
  {
    target_vx = 0.0;
    target_vy = 0.0;
    target_wz = 0.0;
  }

  if (velocity_mode_ == "global")
  {
    geometry_msgs::Vector3Stamped global_vec;
    geometry_msgs::Vector3Stamped base_vec;
    global_vec.header.frame_id = odom_frame_;
    global_vec.header.stamp = ros::Time(0);
    global_vec.vector.x = target_vx;
    global_vec.vector.y = target_vy;
    global_vec.vector.z = 0.0;
    try
    {
      tf_listener_.transformVector(base_frame_, global_vec, base_vec);
      target_vx = base_vec.vector.x;
      target_vy = base_vec.vector.y;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "TF transformVector exception: %s", ex.what());
    }
  }

  double dt = period.toSec();
  if (enable_acc_limit_ && dt > 0.0 && dt < 0.1)
  {
    ramped_vx_ = applyAccLimit(target_vx, ramped_vx_, max_linear_acc_, dt);
    ramped_vy_ = applyAccLimit(target_vy, ramped_vy_, max_linear_acc_, dt);
    ramped_wz_ = applyAccLimit(target_wz, ramped_wz_, max_angular_acc_, dt);
  }
  else
  {
    ramped_vx_ = target_vx;
    ramped_vy_ = target_vy;
    ramped_wz_ = target_wz;
  }

  computeInverseKinematics(ramped_vx_, ramped_vy_, ramped_wz_);

  double actual_lf = front_left_joint_.getVelocity();
  double actual_rf = front_right_joint_.getVelocity();
  double actual_lb = back_left_joint_.getVelocity();
  double actual_rb = back_right_joint_.getVelocity();

  front_left_joint_.setCommand(pid_lf_.computeCommand(desired_vel_lf_ - actual_lf, period));
  front_right_joint_.setCommand(pid_rf_.computeCommand(desired_vel_rf_ - actual_rf, period));
  back_left_joint_.setCommand(pid_lb_.computeCommand(desired_vel_lb_ - actual_lb, period));
  back_right_joint_.setCommand(pid_rb_.computeCommand(desired_vel_rb_ - actual_rb, period));

  // Publish at 50Hz (every 20th loop of 1000Hz) so rqt_plot never lags!
  if (++publish_div_ >= 20)
  {
    publish_div_ = 0;
    std_msgs::Float64 target_msg;
    std_msgs::Float64 actual_msg;
    target_msg.data = desired_vel_lf_;
    actual_msg.data = actual_lf;
    target_vel_pub_.publish(target_msg);
    actual_vel_pub_.publish(actual_msg);
  }
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
