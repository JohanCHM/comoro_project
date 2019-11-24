#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
void manipulateData_callback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    ROS_INFO("Received [%d]", msg->transform);
}

void manipulateDataSimulation_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::cout << msg->twist.twist.linear.x << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover_controller");
    ros::NodeHandle node_obj;

    // Simulation
    ros::Subscriber hover_subscriber = node_obj.subscribe("/bebop/odom", 10, manipulateDataSimulation_callback);
    // Drone
    // ros::Subscriber hover_subscriber = node_obj.subscribe("/vicon/bebop/bebop", 10, manipulateData_callback);

    ros::spin();
    return 0;
}