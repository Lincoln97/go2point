#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

geometry_msgs::Twist v;

ros::Publisher v_pub;

double normalizeAngle(double angle)
{
    if(angle > M_PI){
        return(angle - 2*M_PI);
    }else if(angle < -M_PI){
        return (angle + 2*M_PI);
    }else{
        return angle;
    }
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg){
    
    geometry_msgs::Quaternion qt;
    
    qt = msg->pose.pose.orientation; 
    
    double yaw = tf::getYaw(qt)*180/M_PI;
    
    std::cout << "Yaw: " << yaw << std::endl;
    
    double x = 0, y = 0, xf = -3, yf = 1.5, t;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    
    double d = sqrt(((xf - x)*(xf - x)) + ((yf - y)*(yf - y)));
    
    std::cout << "d: " << d << std::endl;
    
    t = atan2((yf - y),(xf - x))*180/M_PI;
        
    std::cout << "t: " << t << std::endl;
    
    double dt = normalizeAngle(t - yaw);
    
    std::cout << "dt: " << dt << std::endl;
    
    if(d > 0.1){
        if(dt > 0.5){
            v.linear.x = 2;
            v.angular.z = 0.1*dt;
        }
        else if(dt < -0.5){
            v.linear.x = 2;
            v.angular.z = 0.1*dt;
        }
        else {
            v.linear.x = 2;
            v.angular.z = 0;
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