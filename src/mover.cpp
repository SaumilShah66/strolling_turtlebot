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
 * @file mover.cpp
 * @brief class methods for sensing and moving turtlebot
 * @author Saumil Shah
 *
 * Copyright [2019] Saumil Shah
 * All rights reserved.
 */

#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "strolling_turtlebot/mover.hpp"

/// This method reads the laserscan data and checks if they are too much closer to an object
void Mover::checkProximity(const sensor_msgs::LaserScan::ConstPtr& intensities){
	ROS_INFO_STREAM("Checking time....");
	for(auto intensity:intensities->ranges){
		if(intensity <= thresholdIntensity){
			thresholdCroseed = true;  /// changes flag if threshold crossed
			ROS_INFO_STREAM("Danger");
			break;
		}
		else{
			thresholdCroseed = false;
		}
	}
	ROS_INFO_STREAM(thresholdCroseed);
	if(!thresholdCroseed){
		ROS_INFO_STREAM("Bot is safe...Bot can stroll...Enjoy");
		botMover.publish(moveAhead());  /// Pass message to move ahead
	}
	else{
		ROS_INFO_STREAM("Ooops... Danger... Time to turn");
		botMover.publish(stopMessage());  /// Pass message to stop the bot
		botMover.publish(rotateBot());  /// Pass message to rotate the bot
		ros::Duration(0.5).sleep();   /// Wait for half a second
		botMover.publish(stopMessage());  /// Pass message to stop the bot 
	}
}


/// Constructor created
Mover::Mover(){
	ROS_INFO_STREAM("Initializing publisher and subscriber");
	/// Subscriber initialized
	laserReader = n.subscribe<sensor_msgs::LaserScan>("/scan", 30, &Mover::checkProximity, this);
	/// Publisher initialized
	botMover = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",30);
	moveMessage.linear.x = 0;   /// Linear velocity in X direction 0
	moveMessage.linear.y = 0;   /// Linear velocity in Y direction 0
	moveMessage.linear.z = 0;   /// Linear velocity in Z direction 0
	moveMessage.angular.x = 0;  /// Angular velocity in X direction 0
	moveMessage.angular.y = 0;  /// Angular velocity in Y direction 0
	moveMessage.angular.z = 0;  /// Angular velocity in Z direction 0
}

Mover::~Mover(){
	ROS_INFO_STREAM("Destructed");
}

geometry_msgs::Twist Mover::moveAhead(){
	moveMessage.linear.x = 0.5;  /// Linear velocity in X to move forward
	return moveMessage;
}

geometry_msgs::Twist Mover::rotateBot(){
	moveMessage.linear.x = 0.0;   
	moveMessage.angular.z = rotateAngle;  /// Angular velocity to rotate
	return moveMessage;
}

geometry_msgs::Twist Mover::stopMessage(){
	/// Set all velocities to zero 
	moveMessage.linear.x = 0;  
	moveMessage.linear.y = 0;
	moveMessage.linear.z = 0;
	moveMessage.angular.x = 0;
	moveMessage.angular.y = 0;
	moveMessage.angular.z = 0;
	return moveMessage;
}



