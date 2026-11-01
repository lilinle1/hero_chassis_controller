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
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

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
  void updateOdometry(const ros::Time& time, const ros::Duration& period);
  double applyAccLimit(double target, double current, double max_acc, double dt);

  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;

  control_toolbox::Pid pid_lf_;
  control_toolbox::Pid pid_rf_;
  control_toolbox::Pid pid_lb_;
  control_toolbox::Pid pid_rb_;

  double wheel_track_{0.4};
  double wheel_base_{0.4};
  double wheel_radius_{0.07625};

  geometry_msgs::Twist cmd_vel_;
  ros::Time last_cmd_time_;
  bool new_cmd_received_{false};
  double cmd_vel_timeout_{0.5};
  std::string velocity_mode_{"base"};
  bool enable_acc_limit_{true};
  double max_linear_acc_{4.0};
  double max_angular_acc_{6.0};
  double ramped_vx_{0.0};
  double ramped_vy_{0.0};
  double ramped_wz_{0.0};

  double desired_vel_lf_{0.0};
  double desired_vel_rf_{0.0};
  double desired_vel_lb_{0.0};
  double desired_vel_rb_{0.0};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher target_vel_pub_;
  ros::Publisher actual_vel_pub_;
  int publish_div_{0};
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
