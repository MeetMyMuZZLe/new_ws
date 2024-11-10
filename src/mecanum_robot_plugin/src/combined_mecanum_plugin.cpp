// File: src/combined_mecanum_plugin.cpp
#include "mecanum_robot_plugin/combined_mecanum_plugin.hpp"

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{

CombinedMecanumPlugin::CombinedMecanumPlugin()
  : cmd_vx_(0.0)
  , cmd_vy_(0.0)
  , cmd_omega_(0.0)
  , cmd_piston_pos_(0.0)
{
}

CombinedMecanumPlugin::~CombinedMecanumPlugin()
{
  rclcpp::shutdown();
}

void CombinedMecanumPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  body_ = model_->GetLink("base_link");
  
  // Get wheel joints
  fl_wheel_ = model_->GetJoint("front_left_wheel_joint");
  fr_wheel_ = model_->GetJoint("front_right_wheel_joint");
  rl_wheel_ = model_->GetJoint("rear_left_wheel_joint");
  rr_wheel_ = model_->GetJoint("rear_right_wheel_joint");
  
  // Get piston joint
  piston_joint_ = model_->GetJoint("piston_joint");
  
  // Load parameters from SDF
  wheel_radius_ = sdf->Get<double>("wheel_radius", 0.1).first;
  wheel_separation_x_ = sdf->Get<double>("wheel_separation_x", 0.4).first;
  wheel_separation_y_ = sdf->Get<double>("wheel_separation_y", 0.4).first;
  max_wheel_speed_ = sdf->Get<double>("max_wheel_speed", 10.0).first;
  max_piston_force_ = sdf->Get<double>("max_piston_force", 100.0).first;
  
  // Initialize ROS node
  rclcpp::init(0, nullptr);
  ros_node_ = std::make_shared<rclcpp::Node>("combined_mecanum_plugin");
  
  // Create subscribers
  twist_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&CombinedMecanumPlugin::OnTwistMsg, this, std::placeholders::_1));
    
  piston_sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
    "piston_cmd", 10,
    std::bind(&CombinedMecanumPlugin::OnPistonCmd, this, std::placeholders::_1));
    
  // Connect to update event
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&CombinedMecanumPlugin::OnUpdate, this));
}

void CombinedMecanumPlugin::OnTwistMsg(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_omega_ = msg->angular.z;
}

void CombinedMecanumPlugin::OnPistonCmd(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cmd_piston_pos_ = msg->data;
}

void CombinedMecanumPlugin::UpdateWheelVelocities(double vx, double vy, double omega)
{
  // Calculate wheel velocities for Mecanum drive
  // Using standard Mecanum wheel equations for X pattern
  double fl = (vx - vy - omega * (wheel_separation_x_ + wheel_separation_y_)) / wheel_radius_;
  double fr = (vx + vy + omega * (wheel_separation_x_ + wheel_separation_y_)) / wheel_radius_;
  double rl = (vx + vy - omega * (wheel_separation_x_ + wheel_separation_y_)) / wheel_radius_;
  double rr = (vx - vy + omega * (wheel_separation_x_ + wheel_separation_y_)) / wheel_radius_;
  
  // Apply velocity limits
  fl = std::clamp(fl, -max_wheel_speed_, max_wheel_speed_);
  fr = std::clamp(fr, -max_wheel_speed_, max_wheel_speed_);
  rl = std::clamp(rl, -max_wheel_speed_, max_wheel_speed_);
  rr = std::clamp(rr, -max_wheel_speed_, max_wheel_speed_);
  
  // Set wheel velocities
  fl_wheel_->SetVelocity(0, fl);
  fr_wheel_->SetVelocity(0, fr);
  rl_wheel_->SetVelocity(0, rl);
  rr_wheel_->SetVelocity(0, rr);
}

void CombinedMecanumPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Update wheel velocities based on current commands
  UpdateWheelVelocities(cmd_vx_, cmd_vy_, cmd_omega_);
  
  // Apply body forces/velocities for movement
  ignition::math::Vector3d linear_vel(cmd_vx_, cmd_vy_, 0);
  ignition::math::Vector3d angular_vel(0, 0, cmd_omega_);
  body_->SetLinearVel(linear_vel);
  body_->SetAngularVel(angular_vel);
  
  // Update piston position
  piston_joint_->SetForce(0, max_piston_force_ * (cmd_piston_pos_ - piston_joint_->Position()));
}

}  // namespace gazebo