#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

ros::Publisher vel_pub;
geometry_msgs::Twist vel;

int z_angular_position;
const double angular_vel = 0.1;

int radiansToDegrees(double radians) {
    int degrees = std::fmod(std::round(radians * (180.0 / M_PI)), 360.0);

    return degrees;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &data) {
	// Access the orientation from Odometry message
    tf2::Quaternion quaternion;
    tf2::fromMsg(data->pose.pose.orientation, quaternion);

    // Convert Quaternion to Euler angles
    tf2::Matrix3x3 rotation(quaternion);
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);

	z_angular_position = radiansToDegrees(yaw);	

    // Display Euler angles
    // ROS_INFO("Roll: %f", roll);
    // ROS_INFO("Pitch: %f", pitch);
    ROS_INFO("Yaw-Radians: %f", yaw);
    ROS_INFO("Yaw-Degrees: %d", z_angular_position);

    // vel.angular.z = -1 * angular_vel;
    // vel.angular.z = angular_vel;
    // vel_pub.publish(vel);
}

void subscriber() {
  ros::NodeHandle nh;
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber scan_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1000, odomCallback);
  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "publisher_subscriber_test_node");
  subscriber();
  return 0;
}