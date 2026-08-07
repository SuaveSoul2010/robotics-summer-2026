# Sourcing
source opt/ros/foxy/setup.bash
source /jackal_ws/install/setup.bash

# Launch gazebo
ros2 launch jackal_gazebo jackal_world.launch.py

Launch RViz
ros2 launch jackal_viz view_model.launch.py

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Run camera viewer
ros2 run rqt_image_view rqt_image_view

# Creating package

ros2 pkg create (name) \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs nav_msgs geometry_msgs # and more