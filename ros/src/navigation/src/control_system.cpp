#include <ros/ros.h>

#include "navigation/nav.h"
#include "navigation/nav_request.h"
#include "peripherals/imu.h"
#include "controllers.hpp"
#include "geometry_msgs/Vector3.h"

class control_system
{
public:
    control_system(
        double dt, double lin_min, double lin_max, double angl_min, double angl_max, 
        double Kp_vel_x, double Ki_vel_x, double Kp_vel_y, double Ki_vel_y,
        double Kp_pos_z, double Ki_pos_z, double Kp_vel_z, double Ki_vel_z,
        double Kp_pos_p, double Ki_pos_p, double Kp_vel_p, double Ki_vel_p,
        double Kp_pos_r, double Ki_pos_r, double Kp_vel_r, double Ki_vel_r,
        double Kp_vel_yw, double Ki_vel_yw
    );
    ~control_system();
    void receive_nav_request(const navigation::nav_request::ConstPtr &msg);
    void receive_imu_data(const peripherals::imu::ConstPtr &msg);
    void compute_output_vectors(navigation::nav &msg);
private:
    // Controllers
    velocity_controller* linear_vel_x;
    velocity_controller* linear_vel_y;
    position_controller* linear_pos_z;
    position_controller* angular_pos_p;
    position_controller* angular_pos_r;
    velocity_controller* angular_vel_yw;

    // Data
    navigation::nav_request current_request;
    peripherals::imu imu_data;
    double current_depth;
};

control_system::control_system(
    double dt, double lin_min, double lin_max, double angl_min, double angl_max, 
    double Kp_vel_x, double Ki_vel_x, double Kp_vel_y, double Ki_vel_y,
    double Kp_pos_z, double Ki_pos_z, double Kp_vel_z, double Ki_vel_z,
    double Kp_pos_p, double Ki_pos_p, double Kp_vel_p, double Ki_vel_p,
    double Kp_pos_r, double Ki_pos_r, double Kp_vel_r, double Ki_vel_r,
    double Kp_vel_yw, double Ki_vel_yw)
{
    linear_vel_x = new velocity_controller(lin_min, lin_max, dt, Kp_vel_x, Ki_vel_x);
    linear_vel_y = new velocity_controller(lin_min, lin_max, dt, Kp_vel_y, Ki_vel_y);
    linear_pos_z = new position_controller(lin_min, lin_max, dt, Kp_pos_z, Ki_pos_z, Kp_vel_z, Ki_vel_z);
    angular_pos_p = new position_controller(angl_min, angl_max, dt, Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p);
    angular_pos_r = new position_controller(angl_min, angl_max, dt, Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r);
    angular_vel_yw = new velocity_controller(angl_min, angl_max, dt, Kp_vel_yw, Ki_vel_yw);

    current_request.depth = 0;
    current_request.yaw_rate = 0;
    current_request.forwards_velocity = 0;
    current_request.sideways_velocity = 0;

    imu_data.velocity.x = imu_data.velocity.y = imu_data.velocity.z = 0;
    imu_data.compensated_angular_rate.x = imu_data.compensated_angular_rate.y = imu_data.compensated_angular_rate.z = 0;
    imu_data.euler_angles.pitch = imu_data.euler_angles.roll = imu_data.euler_angles.yaw = 0;

    current_depth = 0;
}

control_system::~control_system()
{       
    delete linear_vel_x;
    delete linear_vel_y;
    delete linear_pos_z;
    delete angular_pos_p;
    delete angular_pos_r;
    delete angular_vel_yw;
}

void control_system::receive_nav_request(const navigation::nav_request::ConstPtr &msg) 
{       
    current_request = *msg;
}

void control_system::receive_imu_data(const peripherals::imu::ConstPtr &msg)
{       
    imu_data = *msg;
}
    
void control_system::compute_output_vectors(navigation::nav &msg)
{       
    msg.direction.x = linear_vel_x->calculate(current_request.forwards_velocity, imu_data.velocity.x);
    msg.direction.y = linear_vel_y->calculate(current_request.sideways_velocity, imu_data.velocity.y);
    msg.direction.z = linear_pos_z->calculate(current_request.depth, current_depth, imu_data.velocity.z);
    msg.rotation.pitch = angular_pos_p->calculate(0, imu_data.euler_angles.pitch, imu_data.compensated_angular_rate.y);
    msg.rotation.roll = angular_pos_r->calculate(0, imu_data.euler_angles.roll, imu_data.compensated_angular_rate.x);
    msg.rotation.yaw = angular_vel_yw->calculate(current_request.yaw_rate, imu_data.compensated_angular_rate.z);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "control_system");
    ros::NodeHandle nh("~");

    // General Control System Parameters
    double loop_rate, min_lin, max_lin, min_angl, max_angl;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_linear_val", min_lin);
    nh.getParam("max_linear_val", max_lin);
    nh.getParam("min_angular_val", min_angl);
    nh.getParam("max_angular_val", max_angl);
    
    // Velocity X Control System
    double Kp_vel_x, Ki_vel_x;
    nh.getParam("Kp_vel_x", Kp_vel_x);
    nh.getParam("Ki_vel_x", Ki_vel_x);

    // Velocity Y Control System
    double Kp_vel_y, Ki_vel_y;
    nh.getParam("Kp_vel_y", Kp_vel_y);
    nh.getParam("Ki_vel_y", Ki_vel_y);

    // Position Z Control System
    double Kp_pos_z, Ki_pos_z, Kp_vel_z, Ki_vel_z;
    nh.getParam("Kp_pos_z", Kp_pos_z);
    nh.getParam("Ki_pos_z", Ki_pos_z);
    nh.getParam("Kp_vel_z", Kp_vel_z);
    nh.getParam("Ki_vel_z", Ki_vel_z);

    // Position Pitch Control System
    double Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p;
    nh.getParam("Kp_pos_p", Kp_pos_p);
    nh.getParam("Ki_pos_p", Ki_pos_p);
    nh.getParam("Kp_vel_p", Kp_vel_p);
    nh.getParam("Ki_vel_p", Ki_vel_p);

    // Position Roll Control System
    double Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r;
    nh.getParam("Kp_pos_r", Kp_pos_r);
    nh.getParam("Ki_pos_r", Ki_pos_r);
    nh.getParam("Kp_vel_r", Kp_vel_r);
    nh.getParam("Ki_vel_r", Ki_vel_r);

    // Velocity Yaw Control System
    double Kp_vel_yw, Ki_vel_yw;
    nh.getParam("Kp_vel_yw", Kp_vel_yw);
    nh.getParam("Ki_vel_yw", Ki_vel_yw);

    double dt = 1.0 / loop_rate;
    control_system ctrl(dt, min_lin, max_lin, min_angl, max_angl, Kp_vel_x, Ki_vel_x, Kp_vel_y, Ki_vel_y,
            Kp_pos_z, Ki_pos_z, Kp_vel_z, Ki_vel_z, Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p, Kp_pos_r, 
            Ki_pos_r, Kp_vel_r, Ki_vel_r, Kp_vel_yw, Ki_vel_yw);

    ros::Publisher pub_vectors = nh.advertise<navigation::nav>("/nav/velocity_vectors", 1);

    ros::Subscriber sub_nav = 
        nh.subscribe<navigation::nav_request>("/nav/navigation", 1, &control_system::receive_nav_request, &ctrl);
    ros::Subscriber sub_imu = 
        nh.subscribe<peripherals::imu>("/imu/imu_sensor", 1, &control_system::receive_imu_data, &ctrl);

    ros::Rate r(loop_rate);
    while(ros::ok()) { 
        navigation::nav output_vectors;
        ctrl.compute_output_vectors(output_vectors);
        pub_vectors.publish(output_vectors);

        ros::spinOnce();
        r.sleep();
    }
}
