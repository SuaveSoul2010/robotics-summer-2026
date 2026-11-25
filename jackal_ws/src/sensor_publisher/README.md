# sensor_publisher

This package has a single node that subscribes to the Jackal's main sensor topics and logs what it receives. This is a good way to get familiar with the data the simulated robot produces, and a quick way to check that Gazebo is running and the sensors are publishing.

| | Topic | Type |
| --- | --- | --- |
| Subscribes | `/imu/data` | `sensor_msgs/Imu` |
| Subscribes | `/odom` | `nav_msgs/Odometry` |
| Subscribes | `/joint_states` | `sensor_msgs/JointState` |
| Subscribes | `/front/scan` | `sensor_msgs/LaserScan` |

```bash
ros2 run sensor_publisher sensor_node
```

`/front/scan` only exists when the simulation is launched with `JACKAL_LASER=1`.
