// Copyright 2026 lilinle1
// BSD 3-Clause License

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <string>
#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

namespace hero_chassis_controller
{

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void computeInverseKinematics(double vx, double vy, double wz);

  // Joint handles for 4 Mecanum wheels
  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;

  // 4 independent PID controllers using control_toolbox::Pid
  control_toolbox::Pid pid_lf_;
  control_toolbox::Pid pid_rf_;
  control_toolbox::Pid pid_lb_;
  control_toolbox::Pid pid_rb_;

  // Chassis geometric parameters
  double wheel_track_{0.4};
  double wheel_base_{0.4};
  double wheel_radius_{0.07625};

  // Desired chassis velocity and timeout
  geometry_msgs::Twist cmd_vel_;
  ros::Time last_cmd_time_;
  double cmd_vel_timeout_{0.5};

  // Desired wheel angular velocities (rad/s)
  double desired_vel_lf_{0.0};
  double desired_vel_rf_{0.0};
  double desired_vel_lb_{0.0};
  double desired_vel_rb_{0.0};

  // ROS subscriber and debug publisher for rqt_plot
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher debug_vel_pub_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
