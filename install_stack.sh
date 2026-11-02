set -e
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-description ros-jazzy-turtlebot3-msgs ros-jazzy-xacro ros-jazzy-tf2-tools ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-numpy python3-pandas python3-matplotlib python3-scipy
echo INSTALL_DONE
