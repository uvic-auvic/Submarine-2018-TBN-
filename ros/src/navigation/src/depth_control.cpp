#include <ros/ros.h>
#include "pid.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include "math.h"

#define My      (1)
#define Mx      (1)

double y1_ratio;
double x1_ratio; 

void IMU_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    double pitch = atan(msg->z / msg->x);
    double roll = atan(msg->z / msg->y);
    y1_ratio = sin(roll) * My;
    x1_ratio = sin(pitch) * Mx; 
    ROS_INFO("I heard: [%f]", msg->x);
}

 void depth_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int command_to_thrust(double thrust, int direction){
    //forward: rpm = 21.74167 - 43.47222
    //reverse: rpm = 31.84857 - 225.50476
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "depth_sensor_node");
    ros::NodeHandle nh("~");

    // Get args from launch file
    int loop_rate;
    double kp, ki, kd;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("kp", kp);
    nh.getParam("ki", ki);
    nh.getParam("kd", kd);

    double interval = 1.0/loop_rate;

    ros::Rate r(loop_rate);

    ros::Subscriber imu = nh.subscribe("imu_sensor", 20, IMU_callback);
    ros::Subscriber depth = nh.subscribe("depth", 20, depth_callback);

    PID y_pid = PID(interval, 0.7, -0.7, kp, kd, ki);
    PID x_pid = PID(interval, 0.7, -0.7, kp, kd, ki);
    PID depth_force = PID(interval, 0.7, -0.7, kp, kd, ki);

    while(ros::ok())
    {
        double y1 = y_pid.calculate(0, y1_ratio);
        double x1 = x_pid.calculate(0, x1_ratio);

        double y2 = 1.0 - y1;
        double x2 = 1.0 - x2;
        
        double m11 = y1 * x1;
        double m12 = y1 * x2;
        double m21 = y2 * x1;
        double m22 = y2 * x2;

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}