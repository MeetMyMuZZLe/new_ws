# new_ws
**BASHRC LINES:**

gedit ~/.bashrc

export TURTLEBOT3_MODEL=waffle

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /usr/share/gazebo/setup.sh

source /opt/ros/humble/setup.bash

source ~/ros2_ws/install/setup.bash


**INSTALLATION COMMANDS:**

sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update


sudo apt install python3-colcon-common-extensions

mkdir -p ~/ros2_ws/src

sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt-get install ros-humble-joint-state-publisher*

sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-robot-state-publisher*

sudo apt-get install ros-humble-teleop-twist-keyboard
sudo apt-get install ros-humble-joint-state-publisher*
sudo apt-get install ros-humble-joint-trajectory-controller
sudo apt-get install ros-humble-controller-manager
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-gazebo-msgs
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control-demos
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-ros2controlcli
sudo apt install ros-humble-xacro
sudo apt install ros-humble-gazebo-dev
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-gazebo-ros2-control ros-humble-gazebo-ros2-control-demos

sudo apt install libopencv-dev python3-opencv

sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
sudo apt install git
sudo apt install terminator
sudo snap install code --classic

sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-slam-toolbox
pip install transforms3d
**Lab5 manipulator gazebo fk:**

ros2 run urdf_tutorial control --ros-args -p joint_angles:=[0.5,0.7,-0.3]

**SLAM YAML CHANGES:**

robot_model_type: "nav2_amcl::DifferentialMotionModel"

**CLONE TB3:**

git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git 

