# strolling_turtlebot :: A ros_gazebo package

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

Turtlebot is a robot made for research and educational purposes. It has a stereo camera mounted on it and can be used to try and test different planning and perception algorithms. One such simple demo has been provided here but for simulated environment. 

## Dependencies

These are the dependencies required to use this package

* Ubuntu 16.04 LTS
* ROS Kinetic
* Gazebo 7.0

```
rosbag info results/stroller.bag
```

```
path:         stroller.bag
version:      2.0
duration:     28.5s
start:        Dec 31 1969 19:03:58.38 (238.38)
end:          Dec 31 1969 19:04:26.91 (266.91)
size:         15.8 MB
messages:     200865
compression:  bz2 [128/128 chunks; 13.98%]
uncompressed: 95.9 MB @   3.4 MB/s
compressed:   13.4 MB @ 480.9 KB/s (13.98%)
types:        bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
              dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
              dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
              gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
              gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
              geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
              nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
              rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
              rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
              sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
              sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
              sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
              std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
              tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:       /clock                                            28548 msgs    : rosgraph_msgs/Clock                  
              /cmd_vel_mux/active                                   1 msg     : std_msgs/String                      
              /cmd_vel_mux/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
              /cmd_vel_mux/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
              /depthimage_to_laserscan/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
              /depthimage_to_laserscan/parameter_updates            1 msg     : dynamic_reconfigure/Config           
              /gazebo/link_states                               28149 msgs    : gazebo_msgs/LinkStates               
              /gazebo/model_states                              28143 msgs    : gazebo_msgs/ModelStates              
              /gazebo/parameter_descriptions                        1 msg     : dynamic_reconfigure/ConfigDescription
              /gazebo/parameter_updates                             1 msg     : dynamic_reconfigure/Config           
              /gazebo_gui/parameter_descriptions                    1 msg     : dynamic_reconfigure/ConfigDescription
              /gazebo_gui/parameter_updates                         1 msg     : dynamic_reconfigure/Config           
              /joint_states                                     28161 msgs    : sensor_msgs/JointState               
              /laserscan_nodelet_manager/bond                      58 msgs    : bond/Status                          
              /mobile_base/commands/velocity                      249 msgs    : geometry_msgs/Twist                  
              /mobile_base/sensors/imu_data                     28292 msgs    : sensor_msgs/Imu                      
              /mobile_base_nodelet_manager/bond                   116 msgs    : bond/Status                          
              /odom                                             28279 msgs    : nav_msgs/Odometry                    
              /rosout                                             662 msgs    : rosgraph_msgs/Log                    
              /rosout_agg                                         646 msgs    : rosgraph_msgs/Log                    
              /scan                                               248 msgs    : sensor_msgs/LaserScan                
              /tf                                               29304 msgs    : tf2_msgs/TFMessage                   
              /tf_static                                            1 msg     : tf2_msgs/TFMessage
```