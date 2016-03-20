#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

void odomCallback(const nav_msgs::OdometryConstPtr &msg){
    
    geometry_msgs::Quaternion qt;
    
    qt = msg->pose.pose.orientation; 
    
    double yaw = tf::getYaw(qt);
    
    std::cout << "Yaw: " << yaw*180/M_PI << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2yaw");
    
    ros::NodeHandle nh;
    
    ros:: Subscriber odom_sub = nh.subscribe("/vrep/vehicle/odometry", 1, odomCallback);
        
    ros::spin();
}