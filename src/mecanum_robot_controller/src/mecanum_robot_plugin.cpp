// File: src/mecanum_robot_plugin.cpp

#include "mecanum_robot_controller/mecanum_robot_plugin.hpp"

namespace gazebo_plugins
{

MecanumRobotPlugin::MecanumRobotPlugin()
: wheel_radius_(0.1),
  wheel_separation_width_(0.4),
  wheel_separation_length_(0.4),
  max_wheel_speed_(10.0),
  max_piston_force_(1000.0),
  piston_extended_(false),
  last_update_time_(0.0)
{
}

MecanumRobotPlugin::~MecanumRobotPlugin()
{
  this->update_connection_.reset();
}

void MecanumRobotPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;

  // Initialize ROS node
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Get joints
  front_left_joint_ = model_->GetJoint("front_left_wheel_joint");
  front_right_joint_ = model_->GetJoint("front_right_wheel_joint");
  rear_left_joint_ = model_->GetJoint("rear_left_wheel_joint");
  rear_right_joint_ = model_->GetJoint("rear_right_wheel_joint");
  racket_joint_ = model_->GetJoint("racket_joint");

  // Load parameters from SDF
  if (sdf->HasElement("wheel_radius"))
    wheel_radius_ = sdf->GetElement("wheel_radius")->Get<double>();
  if (sdf->HasElement("wheel_separation_width"))
    wheel_separation_width_ = sdf->GetElement("wheel_separation_width")->Get<double>();
  if (sdf->HasElement("wheel_separation_length"))
    wheel_separation_length_ = sdf->GetElement("wheel_separation_length")->Get<double>();
  if (sdf->HasElement("max_wheel_speed"))
    max_wheel_speed_ = sdf->GetElement("max_wheel_speed")->Get<double>();
  if (sdf->HasElement("max_piston_force"))
    max_piston_force_ = sdf->GetElement("max_piston_force")->Get<double>();

  // Subscribe to joy and cmd_vel topics
  joy_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&MecanumRobotPlugin::JoyCallback, this, std::placeholders::_1));
  
  cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MecanumRobotPlugin::CmdVelCallback, this, std::placeholders::_1));

  // Connect to the world update event
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&MecanumRobotPlugin::OnUpdate, this));

  RCLCPP_INFO(ros_node_->get_logger(), "Mecanum robot plugin loaded successfully");
}

void MecanumRobotPlugin::OnUpdate()
{
  // Add any continuous update logic here if needed
}

void MecanumRobotPlugin::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Map joystick axes to robot movement
  double vx = msg->axes[1] * max_wheel_speed_; // Forward/backward
  double vy = msg->axes[0] * max_wheel_speed_; // Left/right
  double omega = msg->axes[3] * max_wheel_speed_; // Rotation

  UpdateWheelVelocities(vx, vy, omega);

  // Map button to piston control (assuming button 0 controls the piston)
  if (msg->buttons[0] && !piston_extended_) {
    ControlPiston(true, max_piston_force_);
  } else if (!msg->buttons[0] && piston_extended_) {
    ControlPiston(false, 0.0);
  }
}

void MecanumRobotPlugin::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  UpdateWheelVelocities(msg->linear.x, msg->linear.y, msg->angular.z);
}

void MecanumRobotPlugin::UpdateWheelVelocities(double vx, double vy, double omega)
{
  // Mecanum wheel kinematics
  double front_left_speed = (vx - vy - omega * (wheel_separation_width_ + wheel_separation_length_) / 2.0) / wheel_radius_;
  double front_right_speed = (vx + vy + omega * (wheel_separation_width_ + wheel_separation_length_) / 2.0) / wheel_radius_;
  double rear_left_speed = (vx + vy - omega * (wheel_separation_width_ + wheel_separation_length_) / 2.0) / wheel_radius_;
  double rear_right_speed = (vx - vy + omega * (wheel_separation_width_ + wheel_separation_length_) / 2.0) / wheel_radius_;

  // Apply velocities to joints
  front_left_joint_->SetVelocity(0, front_left_speed);
  front_right_joint_->SetVelocity(0, front_right_speed);
  rear_left_joint_->SetVelocity(0, rear_left_speed);
  rear_right_joint_->SetVelocity(0, rear_right_speed);
}

void MecanumRobotPlugin::ControlPiston(bool extend, double force)
{
  if (extend) {
    racket_joint_->SetForce(0, force);
    piston_extended_ = true;
  } else {
    racket_joint_->SetForce(0, -force * 0.5); // Use less force for retraction
    piston_extended_ = false;
  }
}

GZ_REGISTER_MODEL_PLUGIN(MecanumRobotPlugin)
} // namespace gazebo_plugins
