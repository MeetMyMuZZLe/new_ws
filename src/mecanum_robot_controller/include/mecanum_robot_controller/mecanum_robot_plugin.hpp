// File: include/mecanum_robot_controller/mecanum_robot_plugin.hpp

#ifndef MECANUM_ROBOT_PLUGIN_HPP_
#define MECANUM_ROBOT_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo_plugins
{
class MecanumRobotPlugin : public gazebo::ModelPlugin
{
public:
  MecanumRobotPlugin();
  virtual ~MecanumRobotPlugin();

protected:
  // Gazebo callbacks
  virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate();

private:
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void UpdateWheelVelocities(double vx, double vy, double omega);
  void ControlPiston(bool extend, double force);

  // Gazebo objects
  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr front_left_joint_;
  gazebo::physics::JointPtr front_right_joint_;
  gazebo::physics::JointPtr rear_left_joint_;
  gazebo::physics::JointPtr rear_right_joint_;
  gazebo::physics::JointPtr racket_joint_;
  gazebo::event::ConnectionPtr update_connection_;

  // ROS objects
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Parameters
  double wheel_radius_;
  double wheel_separation_width_;
  double wheel_separation_length_;
  double max_wheel_speed_;
  double max_piston_force_;
  
  // Control states
  bool piston_extended_;
  double last_update_time_;
};

} // namespace gazebo_plugins

#endif // MECANUM_ROBOT_PLUGIN_HPP_
