#include <ros/ros.h>

#include "navigation/nav.h"
#include "navigation/nav_request.h"
#include "peripherals/imu.h"
#include "peripherals/powerboard.h"
#include "controllers.hpp"
#include "geometry_msgs/Vector3.h"

class control_system
{
public:
    control_system(
        double dt, double lin_min_vel, double lin_max_vel, double lin_min_pos, double lin_max_pos, 
        double angl_min_vel, double angl_max_vel, double angl_min_pos, double angl_max_pos, 
        double Kp_vel_x, double Ki_vel_x, double Kp_vel_y, double Ki_vel_y,
        double Kp_pos_z, double Ki_pos_z, double Kp_vel_z, double Ki_vel_z,
        double Kp_pos_p, double Ki_pos_p, double Kp_vel_p, double Ki_vel_p,
        double Kp_pos_r, double Ki_pos_r, double Kp_vel_r, double Ki_vel_r,
        double Kp_vel_yw, double Ki_vel_yw
    );
    ~control_system();
    void receive_nav_request(const navigation::nav_request::ConstPtr &msg);
    void receive_imu_data(const peripherals::imu::ConstPtr &msg);
    void receive_powerboard_data(const peripherals::powerboard::ConstPtr &msg);
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
    double dt, double lin_min_vel, double lin_max_vel, double lin_min_pos, double lin_max_pos,
    double angl_min_vel, double angl_max_vel, double angl_min_pos, double angl_max_pos, 
    double Kp_vel_x, double Ki_vel_x, double Kp_vel_y, double Ki_vel_y,
    double Kp_pos_z, double Ki_pos_z, double Kp_vel_z, double Ki_vel_z,
    double Kp_pos_p, double Ki_pos_p, double Kp_vel_p, double Ki_vel_p,
    double Kp_pos_r, double Ki_pos_r, double Kp_vel_r, double Ki_vel_r,
    double Kp_vel_yw, double Ki_vel_yw)
{
    linear_vel_x = new velocity_controller(lin_min_vel, lin_max_vel, dt, Kp_vel_x, Ki_vel_x);
    linear_vel_y = new velocity_controller(lin_min_vel, lin_max_vel, dt, Kp_vel_y, Ki_vel_y);
    linear_pos_z = new position_controller(
            lin_min_vel, lin_max_vel, lin_min_pos, lin_max_pos, dt, Kp_pos_z, Ki_pos_z, Kp_vel_z, Ki_vel_z);
    angular_pos_p = new position_controller(
            angl_min_vel, angl_max_vel, angl_min_pos, angl_max_pos, dt, Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p);
    angular_pos_r = new position_controller(
            angl_min_vel, angl_max_vel, angl_min_pos, angl_max_pos, dt, Kp_pos_r, Ki_pos_r, Kp_vel_r, Ki_vel_r);
    angular_vel_yw = new velocity_controller(angl_min_vel, angl_max_vel, dt, Kp_vel_yw, Ki_vel_yw);

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

void control_system::receive_powerboard_data(const peripherals::powerboard::ConstPtr &msg)
{      
    // depth[m] = pressure[N/m^2] / (density[kg/m^3] * gravity[N/kg])
    current_depth = msg->external_pressure / (997 * 9.81);
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
    double loop_rate, min_lin_vel, max_lin_vel, min_lin_pos, max_lin_pos;
    double min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_linear_vel", min_lin_vel);
    nh.getParam("max_linear_vel", max_lin_vel);
    nh.getParam("min_linear_pos", min_lin_pos);
    nh.getParam("max_linear_pos", max_lin_pos);
    nh.getParam("min_angular_vel", min_angl_vel);
    nh.getParam("max_angular_vel", max_angl_vel);
    nh.getParam("min_angular_pos", min_angl_pos);
    nh.getParam("max_angular_pos", max_angl_pos);
    
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
    control_system ctrl(dt, min_lin_vel, max_lin_vel, min_lin_pos, max_lin_pos, 
            min_angl_vel, max_angl_vel, min_angl_pos, max_angl_pos,
            Kp_vel_x, Ki_vel_x, Kp_vel_y, Ki_vel_y, 
            Kp_pos_z,  Ki_pos_z, Kp_vel_z, Ki_vel_z, 
            Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p, Kp_pos_r, 
            Ki_pos_r, Kp_vel_r, Ki_vel_r, Kp_vel_yw, Ki_vel_yw);

    ros::Publisher pub_vectors = nh.advertise<navigation::nav>("/nav/velocity_vectors", 1);

    ros::Subscriber sub_nav = nh.subscribe<navigation::nav_request>
        ("/nav/navigation", 1, &control_system::receive_nav_request, &ctrl);

    ros::Subscriber sub_imu = nh.subscribe<peripherals::imu>
        ("/imu/imu_sensor", 1, &control_system::receive_imu_data, &ctrl);

    ros::Subscriber sub_pbd = nh.subscribe<peripherals::powerboard>
        ("/power_board/power_board_data", 1, &control_system::receive_powerboard_data, &ctrl);

    ros::Rate r(loop_rate);
    while(ros::ok()) { 
        // Get the output vectors from the control system
        navigation::nav output_vectors;
        ctrl.compute_output_vectors(output_vectors);

        // Normalize the vectors
        output_vectors.direction.x /= max_lin_vel;
        output_vectors.direction.y /= max_lin_vel;
        output_vectors.direction.z /= max_lin_vel;
        pub_vectors.publish(output_vectors);

        ros::spinOnce();
        r.sleep();
    }
}
