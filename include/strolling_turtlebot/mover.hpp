#include <sensor_msgs/LaserScan.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Mover{
	public:
		ros::Subscriber laser_reader;
		ros::Publisher botMover;
		ros::NodeHandle n;
		geometry_msgs::Twist moveMessage;
		geometry_msgs::Twist stopMessage();
		double thresholdIntensity = 0.5;
		double rotateAngle = 0.75;
		bool thresholdCroseed = false;
		void checkProximity(const sensor_msgs::LaserScan::ConstPtr& intensities);
		geometry_msgs::Twist rotateBot();
		geometry_msgs::Twist moveAhead();
		Stroll();
		~Stroll();
}