// Copyright 2026 lilinle1
// BSD 3-Clause License

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller
{

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // Read geometric and timeout parameters from config/controllers.yaml
  controller_nh.param("wheel_track", wheel_track_, 0.4);
  controller_nh.param("wheel_base", wheel_base_, 0.4);
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);

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

  // Subscribe to /cmd_vel topic
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  // Publish target vs actual velocity for easy plotting in rqt_plot
  debug_vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("wheel_vel_debug", 10);

  return true;
}

void HeroChassisController::starting(const ros::Time& time)
{
  pid_lf_.reset();
  pid_rf_.reset();
  pid_lb_.reset();
  pid_rb_.reset();
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

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
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

  // 1. Inverse Kinematics: chassis twist -> 4 wheel target velocities
  computeInverseKinematics(vx, vy, wz);

  // 2. Closed-loop PID velocity control for each wheel
  double error_lf = desired_vel_lf_ - front_left_joint_.getVelocity();
  double error_rf = desired_vel_rf_ - front_right_joint_.getVelocity();
  double error_lb = desired_vel_lb_ - back_left_joint_.getVelocity();
  double error_rb = desired_vel_rb_ - back_right_joint_.getVelocity();

  double cmd_lf = pid_lf_.computeCommand(error_lf, period);
  double cmd_rf = pid_rf_.computeCommand(error_rf, period);
  double cmd_lb = pid_lb_.computeCommand(error_lb, period);
  double cmd_rb = pid_rb_.computeCommand(error_rb, period);

  front_left_joint_.setCommand(cmd_lf);
  front_right_joint_.setCommand(cmd_rf);
  back_left_joint_.setCommand(cmd_lb);
  back_right_joint_.setCommand(cmd_rb);
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
