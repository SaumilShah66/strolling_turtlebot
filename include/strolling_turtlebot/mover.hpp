/**
 * MIT License

 * Copyright (c) 2019 Saumil Shah

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/

/*
 * @file mover.hpp
 * @brief class declaration for sensing and moving turtlebot
 * @author Saumil Shah
 *
 * Copyright [2019] Saumil Shah
 * All rights reserved.
 */

#ifndef INCLUDE_STROLLING_TURTLEBOT_MOVER_HPP_
#define INCLUDE_STROLLING_TURTLEBOT_MOVER_HPP_
#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Mover {
 public:
  /*
   * @brief Declaring a node handle variable
   */
  ros::NodeHandle n;
  /*
   * @brief Declaring a subscriber to read laser scan data from gazebo
   */
  ros::Subscriber laserReader;
  /*
   * @brief Declaring publisher to publish twist messages to move bot
   */
  ros::Publisher botMover;
  /*
   * @brief Container to save linear and angular velocities
   */
  geometry_msgs::Twist moveMessage;
  /*
   * @brief Class method to publish message which will stop turtlebot
   * @param none
   * @return Message of type geometry_msgs::Twist
   */
  geometry_msgs::Twist stopMessage();
  /*
   * @brief Variable to decide if proximity is at threshold or not
   */
  double thresholdIntensity = 0.5;
  /*
   * @brief Varible to define angular velocity while turning
   */
  double rotateAngle = 0.75;
  /*
   * @brief Flag to mark proximity
   */
  bool thresholdCroseed = false;
  /*
   * @brief Class method to check proximity from nearby objects
   * @param intensities of type sensor_msgs::LaserScan::ConstPtr
   * @return none
   */
  void checkProximity(const sensor_msgs::LaserScan::ConstPtr& intensities);
  /*
   * @brief Class method to publish message which rotate turtlebot
   * @param none
   * @return Message of type geometry_msgs::Twist
   */
  geometry_msgs::Twist rotateBot();
  /*
   * @brief Class method to publish message which will move turtlebot
   * 		 in straighr line
   * @param none
   * @return Message of type geometry_msgs::Twist
   */
  geometry_msgs::Twist moveAhead();
  /*
   * @brief Class constructor to initialize node
   * @param none
   * @return none
   */
  Mover();
  /*
   * @brief Class destructor
   * @param none
   * @return none
   */
  ~Mover();
};

#endif  // INCLUDE_STROLLING_TURTLEBOT_MOVER_HPP_
