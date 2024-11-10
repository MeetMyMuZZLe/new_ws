// File: include/mecanum_robot_plugin/combined_mecanum_plugin.hpp
#ifndef MECANUM_ROBOT_PLUGIN_COMBINED_MECANUM_PLUGIN_HPP_
#define MECANUM_ROBOT_PLUGIN_COMBINED_MECANUM_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>

namespace gazebo
{
class CombinedMecanumPlugin : public ModelPlugin
{
public:
  CombinedMecanumPlugin();
  virtual ~CombinedMecanumPlugin();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

private:
  void OnTwistMsg(const geometry_msgs::msg::Twist::SharedPtr msg);
  void OnPistonCmd(const std_msgs::msg::Float64::SharedPtr msg);
  void UpdateWheelVelocities(double vx, double vy, double omega);
  void OnUpdate();

  physics::ModelPtr model_;
  physics::LinkPtr body_;
  physics::JointPtr fl_wheel_, fr_wheel_, rl_wheel_, rr_wheel_;
  physics::JointPtr piston_joint_;
  
  event::ConnectionPtr update_connection_;
  
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr piston_sub_;
  
  // Robot parameters
  double wheel_radius_;
  double wheel_separation_x_;  // Distance between front and rear wheels
  double wheel_separation_y_;  // Distance between left and right wheels
  double max_wheel_speed_;
  double max_piston_force_;
  
  // Current commands
  double cmd_vx_, cmd_vy_, cmd_omega_;
  double cmd_piston_pos_;
  
  std::mutex mutex_;
};

GZ_REGISTER_MODEL_PLUGIN(CombinedMecanumPlugin)
}  // namespace gazebo

#endif  // MECANUM_ROBOT_PLUGIN_COMBINED_MECANUM_PLUGIN_HPP_