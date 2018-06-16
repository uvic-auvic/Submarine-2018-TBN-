#include <math.h>
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "navigation/nav_movement.h"
#include "peripherals/imu.h"
#include "peripherals/orientation.h"

#define PI 3.14159265

#define EPSILON_VELOCITY 0.001
#define EPSILON_ANGULAR_RATE 0.008
#define EPSILON_ACCEL 0.01

class navigation_receiver { 
public:
    navigation_receiver();
    ~navigation_receiver() {}
    void update_velocity(geometry_msgs::Vector3 gravity_accel, geometry_msgs::Vector3 acceleration, double timestamp);
    void imu_data_callback(const peripherals::imu::ConstPtr& msg);
    geometry_msgs::Vector3 velocity_actual;
    geometry_msgs::Vector3 velocity_desired;
    geometry_msgs::Vector3 angular_rate_actual;
    geometry_msgs::Vector3 angular_rate_desired;
    peripherals::orientation euler_angles;
private:
    // Below are members used for the velocity computation
    geometry_msgs::Vector3 last_accel;
    double last_timestamp;
    bool valid_vector;
};

navigation_receiver::navigation_receiver() {   
    // Initial conditions for integration: v(x) = v(y) = v(z) = a(x) = a(y) = a(z) = 0
    velocity_actual.x = velocity_actual.y = velocity_actual.z = 0;
    last_accel.x = last_accel.y = last_accel.z = 0;
    last_timestamp = 0;
    valid_vector = false;

    // Direct from IMU vectors
    angular_rate_actual.x = angular_rate_actual.y = angular_rate_actual.z = 0;
    euler_angles.pitch = euler_angles.roll = euler_angles.yaw = 0;

    // Desired vectors
    velocity_desired.x = velocity_desired.y = velocity_desired.z = 0;
    angular_rate_desired.x = angular_rate_desired.y = angular_rate_desired.z = 0;
}

void navigation_receiver::imu_data_callback(const peripherals::imu::ConstPtr& msg) {
    angular_rate_actual = msg->compensated_angular_rate;
    double angular_rate_abs = angular_rate_actual.x * angular_rate_actual.x + angular_rate_actual.y * angular_rate_actual.y + angular_rate_actual.z *  angular_rate_actual.z;
    double accel_abs = msg->acceleration.x * msg->acceleration.x + msg->acceleration.y * msg->acceleration.y + msg->acceleration.z * msg->acceleration.z;
    double stab_accel_abs = msg->stabilised_acceleration.x * msg->stabilised_acceleration.x + msg->stabilised_acceleration.y * msg->stabilised_acceleration.y + msg->stabilised_acceleration.z * msg->stabilised_acceleration.z;
    ROS_INFO("Accel:%f, Stab Accel:%f", accel_abs, stab_accel_abs);
    euler_angles = msg->euler_angles;
    /*geometry_msgs::Vector3 gravity_vector;
    gravity_vector.x = sin(euler_angles.pitch * PI / 180.0);
    gravity_vector.y = sin(euler_angles.roll * PI / 180.0) * (-1.0);
    gravity_vector.z = (-1.0) * (1 - gravity_vector.x * gravity_vector.x - gravity_vector.y * gravity_vector.y);*/
    //ROS_INFO("Gravity: X:%f, Y:%f, Z:%f", gravity_vector.x, gravity_vector.y, gravity_vector.z);
    update_velocity(msg->stabilised_acceleration, msg->acceleration, msg->instantaneous_vectors_timestamp);

    if(angular_rate_abs < EPSILON_ANGULAR_RATE && (accel_abs - 1) < EPSILON_ACCEL)
        velocity_actual.x = velocity_actual.y = velocity_actual.z = 0.0;
    //update_velocity(gravity_vector, msg->acceleration, msg->instantaneous_vectors_timestamp);
}

void navigation_receiver::update_velocity(geometry_msgs::Vector3 gravity_accel, 
        geometry_msgs::Vector3 acceleration, double timestamp) {
    // Factor in gravity to get vehicle acceleration
    geometry_msgs::Vector3 accel;
    accel.x = acceleration.x - gravity_accel.x;
    accel.y = acceleration.y - gravity_accel.y;
    accel.z = acceleration.z - gravity_accel.z;
    accel.x = ((int)(10.0 * accel.x + copysign(0.5, accel.x))) / 10.0;
    accel.y = ((int)(10.0 * accel.y + copysign(0.5, accel.y))) / 10.0;
    accel.z = ((int)(10.0 * accel.z + copysign(0.5, accel.z))) / 10.0;

    //ROS_INFO("Compensated accel: X:%f, Y:%f, Z:%f", accel.x, accel.y, accel.z);

    // The first run will not have enough data to evaluate integral
    if(!valid_vector) { 
        last_accel = accel;
        last_timestamp = timestamp;
        valid_vector = true;
        return;
    }

    // This chunk of code is necessary for timestamp overflows
    double this_time;
    // Check for timestamp overflow
    if(timestamp < last_timestamp) {    
        this_time = timestamp + 429490.2;
    }
    else {      
        this_time = timestamp;
    }

    // Integrate acceleration to get velocity in m/s
    // Integration uses the average acceleration between the two closest points
    velocity_actual.x = velocity_actual.x + 9.8 * 0.5 * (accel.x + last_accel.x) * (this_time - last_timestamp) / 1000.0;
    velocity_actual.y = velocity_actual.y + 9.8 * 0.5 * (accel.y + last_accel.y) * (this_time - last_timestamp) / 1000.0;
    velocity_actual.z = velocity_actual.z + 9.8 * 0.5 * (accel.z + last_accel.z) * (this_time - last_timestamp) / 1000.0;

    last_timestamp = timestamp; // Set to timestamp instead of this_time due to overflow events (simplifies things)
    last_accel = accel;

    /*if(abs(velocity_actual.x) < EPSILON_VELOCITY)
        velocity_actual.x = 0.0;
    if(abs(velocity_actual.y) < EPSILON_VELOCITY)
        velocity_actual.y = 0.0;
    if(abs(velocity_actual.z) < EPSILON_VELOCITY)
        velocity_actual.z = 0.0;*/
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "nav_req_handler");
    ros::NodeHandle nh("~");
    
    // Subscribe to the IMU, and publish vehicle movement vectors for further processing
    navigation_receiver nav_state;
    ros::Subscriber sub = 
        nh.subscribe<peripherals::imu>("/imu/imu_sensor", 100, &navigation_receiver::imu_data_callback, &nav_state);
    ros::Publisher pub = nh.advertise<navigation::nav_movement>("movement", 10);

    ros::Rate r(10);
    while(ros::ok()) {  
        navigation::nav_movement msg;
        msg.velocity_desired = nav_state.velocity_desired;
        msg.velocity_actual = nav_state.velocity_actual;
        msg.angular_rate_desired = nav_state.angular_rate_desired;
        msg.angular_rate_actual = nav_state.angular_rate_actual;
        msg.euler_angles = nav_state.euler_angles;
        pub.publish(msg);

        // Check for callbacks and sleep until ready for next publish
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
