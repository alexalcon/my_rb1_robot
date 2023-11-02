#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>

#include <cmath>

ros::Publisher vel_pub;            // Publisher object.
ros::ServiceServer service_server; // Service server object.
geometry_msgs::Twist vel;          // Velocity object.

const double kP = 0.03; // Proportional controller gain.
int z_angular_position; // Global subscriber data from odomCallback()
                        // for rotateAngleServiceCallback.

// Radians to degrees conversion
int radiansToDegrees(double radians) {
    int degrees = std::fmod(std::round(radians * (180.0 / M_PI)), 360.0);

    return degrees;
}

// Subscriber callback function to recover nav_msgs/Odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr &data) {   
    // Access the orientation from Odometry message
    tf2::Quaternion quaternion;
    tf2::fromMsg(data->pose.pose.orientation, quaternion);

    // Convert Quaternion to Euler angles
    tf2::Matrix3x3 rotation(quaternion);
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);

	z_angular_position = radiansToDegrees(yaw); // Euler angular position 
                                                // converted to degrees.	 
}

// Callback function for service server
bool rotateAngleServiceCallback(
    my_rb1_ros::Rotate::Request &req,
    my_rb1_ros::Rotate::Response &res) {
    
    ROS_INFO("The Service /rotate_robot has been called.");

    bool flag = true;
    int initial_angle = z_angular_position;
    int target_angle = initial_angle + req.degrees; // Calculate the target angle
    ros::Rate loop_rate(10);                        // 10 [Hz] rate.

    // Degrees adjustments for target angle
    if (target_angle > 180) {
        target_angle = target_angle - 360;
    }
    else if (target_angle < -180) {
        target_angle = target_angle + 360;
    }

    ros::Time start_time = ros::Time::now(); // Get the start time before the loop iteration.

    while(flag == true && ros::ok()) {
        ros::spinOnce(); // Process incoming messages, including 
                         // updating the subscriber data.
        
        /* --- Main logic --- */
        /* -------------------------------------------------------------------- */
        vel.angular.z = kP *(target_angle - z_angular_position);
        vel_pub.publish(vel);
        // ROS_INFO("Target: %d   Current: %d", target_angle, z_angular_position);
        // ROS_INFO("Error: %d", target_angle - z_angular_position);

        if (target_angle - z_angular_position == 0) {
            flag = false;
        }
        // /* ----------------------------------------------------------------- */

        loop_rate.sleep(); // Control the loop rate.
    }
    
    ros::Time end_time = ros::Time::now();           // Get the end time after the loop iteration
    ros::Duration duration = end_time - start_time;  // Calculate the duration of the loop iteration
       
    ROS_INFO("The operation took %f seconds.", duration.toSec());

    res.result = "The service /rotate_robot has been accomplished.";
    ROS_INFO("Finished /rotate_robot service.");
    ROS_INFO("-------------------------------");

    return true;
}

void subscriber() {
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odom", 1000, odomCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    service_server = nh.advertiseService("/rotate_robot", rotateAngleServiceCallback);

    ROS_INFO("Service /rotate_robot ready.");

    ros::spin(); // Spin to process incoming messages and service requests.
}
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "rotate_service_node");
    subscriber();
    return 0;
}