#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

namespace gazebo
{
class MecanumDrivePlugin : public ModelPlugin
{
public:
  MecanumDrivePlugin() : wheel_separation_w_(0.5),
                         wheel_separation_l_(0.5),
                         wheel_radius_(0.1),
                         max_wheel_torque_(20.0),
                         max_wheel_velocity_(10.0),
                         update_rate_(100.0) {}

  ~MecanumDrivePlugin()
  {
    if (update_connection_) {
      update_connection_.reset();
    }
    if (node_) {
      rclcpp::shutdown();
    }
  }

private:
  // Model and world pointers
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  
  // Joints
  physics::JointPtr fl_joint_;
  physics::JointPtr fr_joint_;
  physics::JointPtr rl_joint_;
  physics::JointPtr rr_joint_;
  physics::JointPtr racket_joint_;
  std::vector<physics::JointPtr> roller_joints_;

  // Links
  physics::LinkPtr base_link_;
  
  // ROS 2 node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Update event connection
  event::ConnectionPtr update_connection_;
  
  // Parameters
  double wheel_separation_w_;  // Width between wheels
  double wheel_separation_l_;  // Length between wheels
  double wheel_radius_;
  double max_wheel_torque_;
  double max_wheel_velocity_;
  double update_rate_;
  
  // State variables
  common::Time last_update_time_;
  double x_ = 0;
  double y_ = 0;
  double theta_ = 0;
  double vx_ = 0;
  double vy_ = 0;
  double vtheta_ = 0;

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    model_ = _model;
    world_ = model_->GetWorld();
    
    // Initialize ROS 2
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("mecanum_drive_plugin");
    
    // Load parameters from SDF
    LoadParameters(_sdf);
    
    // Get joints
    GetJoints();
    
    // Set up ROS 2 communication
    SetupROS2();
    
    // Configure joints
    ConfigureJoints();
    
    // Connect update
    last_update_time_ = world_->SimTime();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumDrivePlugin::OnUpdate, this));
      
    RCLCPP_INFO(node_->get_logger(), "Mecanum drive plugin loaded successfully");
  }

private:
  void LoadParameters(sdf::ElementPtr _sdf)
  {
    // Load robot parameters
    wheel_separation_w_ = _sdf->Get<double>("wheel_separation_width", wheel_separation_w_).first;
    wheel_separation_l_ = _sdf->Get<double>("wheel_separation_length", wheel_separation_l_).first;
    wheel_radius_ = _sdf->Get<double>("wheel_radius", wheel_radius_).first;
    max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", max_wheel_torque_).first;
    max_wheel_velocity_ = _sdf->Get<double>("max_wheel_velocity", max_wheel_velocity_).first;
    update_rate_ = _sdf->Get<double>("update_rate", update_rate_).first;
  }

  void GetJoints()
  {
    // Get wheel joints
    fl_joint_ = model_->GetJoint("front_left_wheel_joint");
    fr_joint_ = model_->GetJoint("front_right_wheel_joint");
    rl_joint_ = model_->GetJoint("rear_left_wheel_joint");
    rr_joint_ = model_->GetJoint("rear_right_wheel_joint");
    
    // Get racket joint
    racket_joint_ = model_->GetJoint("racket_joint");
    
    // Get roller joints
    for (int i = 1; i <= 32; i++) {
      std::string joint_name = "roller" + std::to_string(i) + "_joint";
      auto joint = model_->GetJoint(joint_name);
      if (joint) {
        roller_joints_.push_back(joint);
      }
    }
    
    // Get base link
    base_link_ = model_->GetLink("base_link");
    
    // Verify joints
    if (!fl_joint_ || !fr_joint_ || !rl_joint_ || !rr_joint_) {
      RCLCPP_ERROR(node_->get_logger(), "Could not find all wheel joints!");
      return;
    }
  }

  void SetupROS2()
  {
    // Create publisher for odometry
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Create subscriber for velocity commands
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&MecanumDrivePlugin::OnCmdVel, this, std::placeholders::_1));
      
    // Set up transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  void ConfigureJoints()
  {
    // Configure wheel joints
    std::vector<physics::JointPtr> wheel_joints = {fl_joint_, fr_joint_, rl_joint_, rr_joint_};
    for (auto& joint : wheel_joints) {
      joint->SetParam("fmax", 0, max_wheel_torque_);
      joint->SetParam("max_velocity", 0, max_wheel_velocity_);
      // Set axis to rotate around Y
      joint->SetAxis(0, ignition::math::Vector3d(0, 1, 0));
    }
    
    // Configure roller joints
    for (auto& joint : roller_joints_) {
      joint->SetParam("fmax", 0, 0.1);  // Low friction for free spinning
      joint->SetAxis(0, ignition::math::Vector3d(0, 1, 0));
    }
    
    // Configure racket joint
    if (racket_joint_) {
      racket_joint_->SetParam("fmax", 0, 1.0);
      racket_joint_->SetUpperLimit(0, 0.5);  // Limit extension
      racket_joint_->SetLowerLimit(0, 0.0);
    }
  }

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vtheta_ = msg->angular.z;
    
    // Calculate wheel velocities using mecanum drive kinematics
    double fl_vel = (vx_ - vy_ - (wheel_separation_l_ + wheel_separation_w_) * vtheta_) / wheel_radius_;
    double fr_vel = (vx_ + vy_ + (wheel_separation_l_ + wheel_separation_w_) * vtheta_) / wheel_radius_;
    double rl_vel = (vx_ + vy_ - (wheel_separation_l_ + wheel_separation_w_) * vtheta_) / wheel_radius_;
    double rr_vel = (vx_ - vy_ + (wheel_separation_l_ + wheel_separation_w_) * vtheta_) / wheel_radius_;
    
    // Apply velocity limits
    fl_vel = std::clamp(fl_vel, -max_wheel_velocity_, max_wheel_velocity_);
    fr_vel = std::clamp(fr_vel, -max_wheel_velocity_, max_wheel_velocity_);
    rl_vel = std::clamp(rl_vel, -max_wheel_velocity_, max_wheel_velocity_);
    rr_vel = std::clamp(rr_vel, -max_wheel_velocity_, max_wheel_velocity_);
    
    // Set wheel velocities
    fl_joint_->SetParam("vel", 0, fl_vel);
    fr_joint_->SetParam("vel", 0, fr_vel);
    rl_joint_->SetParam("vel", 0, rl_vel);
    rr_joint_->SetParam("vel", 0, rr_vel);
  }

  void OnUpdate()
  {
    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();
    
    if (dt < 1.0 / update_rate_) {
      return;
    }
    
    // Get base link pose and velocity
    auto pose = base_link_->WorldPose();
    auto lin_vel = base_link_->WorldLinearVel();
    auto ang_vel = base_link_->WorldAngularVel();
    
    // Update odometry
    x_ = pose.Pos().X();
    y_ = pose.Pos().Y();
    theta_ = pose.Rot().Yaw();
    
    // Publish odometry
    PublishOdometry(pose, lin_vel, ang_vel);
    
    // Update roller dynamics (simplified)
    UpdateRollers();
    
    last_update_time_ = current_time;
  }

  void PublishOdometry(const ignition::math::Pose3d& pose,
                      const ignition::math::Vector3d& lin_vel,
                      const ignition::math::Vector3d& ang_vel)
  {
    // Create and publish transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = pose.Pos().X();
    transform.transform.translation.y = pose.Pos().Y();
    transform.transform.translation.z = pose.Pos().Z();
    
    tf2::Quaternion q;
    q.setRPY(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform);
    
    // Create and publish odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = node_->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation = transform.transform.rotation;
    
    odom.twist.twist.linear.x = lin_vel.X();
    odom.twist.twist.linear.y = lin_vel.Y();
    odom.twist.twist.linear.z = lin_vel.Z();
    odom.twist.twist.angular.x = ang_vel.X();
    odom.twist.twist.angular.y = ang_vel.Y();
    odom.twist.twist.angular.z = ang_vel.Z();
    
    odom_pub_->publish(odom);
  }

    void UpdateRollers()
    {
        // Update roller positions based on wheel rotations
        for (auto& roller : roller_joints_) {
            // Set low friction for free spinning
            roller->SetParam("vel", 0, 0.0);
            roller->SetParam("fmax", 0, 0.1);
        }
    }
};

GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
}