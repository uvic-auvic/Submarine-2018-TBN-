#include <ros/ros.h>
#include "pid.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/String.h"
#include "math.h"

/*
#include "geometry_msgs/Vector3.h"
#include "navigation/nav_movement.h"
#include "peripherals/imu.h"
#include "peripherals/orientation.h"
*/

#define KI      (1)
#define KP      (1)
#define KD        (0)

#define ATM_PSI                     (14.6959)
#define PSI_PER_METER               (1.4579)
#define DEPTH_PSI_TO_METERS(p)      ((p - ATM_PSI) / PSI_PER_METER)

class control_system { 

public:
    control_system(double interval);
    ~control_system();

private:
    float imu_pitch_V;
    float imu_roll_V;
    float imu_yaw_V;
    float depth_m;

    geometry_msgs::Vector3 imu_velocity;
    geometry_msgs::Vector3 desired_velocity;


    PID* x_pid;
    PID* y_pid;
    PID* depth_pid;

    void update_error();


    /*void IMU_callback(const peripherals::imu::ConstPtr& msg)
    {
        imu_velocity = msg->velocity;
        this->imu_pitch_V = msg->euler_angles->pitch;
        this->imu_roll_V = msg->euler_angles->roll;
        this->imu_yaw_V = msg->euler_angles->yaw;
        
        //ROS_INFO("I heard: [%f]", msg->x);
    }*/

    /* void depth_callback(const peripherals::powerboard::ConstPtr& msg)
    {
        this->depth_m = DEPTH_PSI_TO_METERS(msg->external_pressure)
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }*/

    /* void camera_callback()
    {
        this->camera_coordinates = geometry_msgs::Vector3;
        ROS_INFO("I heard: [%s]", msg->data.c_str());
    }*/
};

control_system :: control_system (double interval)
    :imu_pitch_V(0), imu_roll_V(0), imu_yaw_V(0),
    depth_m(0)
{
    x_pid = new PID(interval, 0.7, -0.7, KP, KD, KI);
    y_pid = new PID(interval, 0.7, -0.7, KP, KD, KI);
    depth_pid = new PID(interval, 0.7, -0.7, KP, KD, KI);
}

control_system :: ~control_system () 
{
    delete x_pid;
    delete y_pid;
    delete depth_pid;
}

void control_system::update_error()
{
    float x_error = x_pid->calculate(this->desired_velocity.x, this->imu_velocity.x);
    float y_error = y_pid->calculate(this->desired_velocity.y, this->imu_velocity.y);
    float depth_error = depth_pid->calculate(this->desired_velocity.z, this->imu_velocity.z);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "control_system_node");
    ros::NodeHandle nh("~");

    // Get args from launch file
    int loop_rate;
    double kp, ki, kd;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("kp", kp);
    nh.getParam("ki", ki);
    nh.getParam("kd", kd);

    double interval = 1.0/loop_rate;

    control_system CS(interval);

    ros::Rate r(loop_rate);

    //ros::Subscriber imu = nh.subscribe("imu_sensor", 20, IMU_callback);
    //ros::Subscriber depth = nh.subscribe("depth", 20, depth_callback);


    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}