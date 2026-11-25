roslaunch terrahca terrahca_core.launch
rostopic pub /robot/autonomy std_msgs/Bool "data: true"

# color image
rosrun image_view image_view image:=/camera/color/image_raw

# compressed image
rosrun image_view image_view image:=/camera/color/image_raw _image_transport:=compressed

# teleop
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/robot/cmd_vel

# publish vel
rostopic pub /robot/cmd_vel geometry_msgs/Twist "linear:
 x: 0.0
 y: 0.0
 z: 0.0
angular:
 x: 0.0
 y: 0.0
 z: 1.0"
