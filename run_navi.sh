source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
source ./install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False


