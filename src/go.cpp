#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define xf 4
#define yf 1.5

geometry_msgs::Twist v;

ros::Publisher v_pub;

void odomCallback(const nav_msgs::OdometryConstPtr &msg){
    
    geometry_msgs::Quaternion qt;
    
    qt = msg->pose.pose.orientation; 
    
    double yaw = tf::getYaw(qt);
    
    std::cout << "Yaw: " << yaw*180/M_PI << std::endl;
    
    double x = 0, y = 0;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    
    double d = sqrt(((x - xf)*(x - xf)) + ((y - yf)*(y - yf)));
    
    std::cout << "d: " << d << std::endl;
    
    double t = atan((yf - y)/(xf - x));
    
    double dt = (t - yaw)*180/M_PI;
    
    if(d > 0.1){
        if(dt > 0.5){
            v.linear.x = 2;
            v.angular.z = 0.5;
        }
        else{
            if(dt < -0.5){
                v.linear.x = 2;
                v.angular.z = -0.5;
            }
            else {
                v.linear.x = 2;
                v.angular.z = 0;
            }
        }
    }
    if(d < 0.1){
        v.linear.x = 0;
        v.angular.z = 0;
    }
    v_pub.publish(v);
}
    
int main(int argc, char **argv){
    
    ros::init(argc,argv,"control");
    
    ros::NodeHandle control;
    
    v_pub = control.advertise<geometry_msgs::Twist>("Velocity", 1);
    
    ros::Subscriber odom_sub = control.subscribe("/vrep/vehicle/odometry", 1, odomCallback);
    
    ros::spin();
}