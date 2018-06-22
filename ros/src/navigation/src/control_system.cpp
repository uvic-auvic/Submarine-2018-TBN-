#include "controllers.hpp"

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
private:
    velocity_controller* linear_vel_x;
    velocity_controller* linear_vel_y;
    position_controller* linear_pos_z;
    position_controller* angular_pos_p;
    position_controller* angular_pos_r;
    velocity_controoler* angular_vel_yw;
};

control_system::control_system(
    double dt, double lin_min, double lin_max, double angl_min, double angl_max, 
    double Kp_vel_x, double Ki_vel_x, double Kp_vel_y, double Ki_vel_y,
    double Kp_pos_z, double Ki_pos_z, double Kp_vel_z, double Ki_vel_z,
    double Kp_pos_p, double Ki_pos_p, double Kp_vel_p, double Ki_vel_p,
    double Kp_pos_r, double Ki_pos_r, double Kp_vel_r, double Ki_vel_r,
    double Kp_vel_yw, double Ki_vel_yw);
{
    linear_vel_x = new velocity_controller(lin_min, lin_max, dt, Kp_vel_x, Ki_vel_x);
    linear_vel_y = new velocity_controller(lin_min, lin_max, dt, Kp_vel_y, Ki_vel_y);
    linear_pos_z = new position_controller(lin_min, lin_max, dt, Kp_pos_z, Ki_pos_z, Kp_vel_z, Ki_vel_z);
    angular_pos_p = new position_controller(angl_min, angl_max, dt, Kp_pos_p, Ki_pos_p, Kp_vel_p, Ki_vel_p);
}


int main(int argc, char ** argv)
{
}
