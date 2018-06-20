#include <ros/ros.h>
#include "navigation/nav.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"

#define NUMBER_OF_THRUSTERS     (8)

double E[8][6];

//We calculate this dynamically so we can use different COGs for submarine changes
void calculate_E(){
    //Origin to Thruster vector
    const double Origin_to_Thruster[NUMBER_OF_THRUSTERS][3] = 
    {
        {-3.606, -2.913, -0.3},
        {-3.606, 19.413, -0.3},
        {-30.394, -2.913, -0.3},
        {-30.394, 19.413, -0.3},
        {3.063, 8.25, -1.0},
        {-37.063, 8.25, -1.0},
        {-15.391, -2.913, 3.7},
        {-15.391, 19.413, 3.7}
    };

    //Origin to Center of Gravity
    const double Origin_to_COG[3] = {-18.305, 8.217, 1.796};

    //Thruster to Center of Gravity
    double Thruster_to_COG[8][3];

    //T is for each thruster
    for(int T = 0; T < NUMBER_OF_THRUSTERS; T++){
        //D is for each dimension
        for(int D = 0; D < 3; D++){
            Thruster_to_COG[T][D] = Origin_to_Thruster[T][D] - Origin_to_COG[D];
        }
    }


}

class thrust_controller
{
public:
    thrust_controller(std::string node_name);
    void generate_thrust_val(const navigation::nav::ConstPtr &msg);
private:
    ros::NodeHandle nh;
    ros::ServiceClient motor_forward;
    ros::ServiceClient motor_reverse;
    ros::ServiceClient motor_set_all;
    ros::ServiceClient motor_stop;
    ros::ServiceClient motors_stop;
    const float EMatrix[5][5];
};

thrust_controller::thrust_controller(std::string node_name) :
    nh(ros::NodeHandle("~")),
    motor_forward(nh.serviceClient<peripherals::motor>(node_name + "/setMotorForward")),
    motor_reverse(nh.serviceClient<peripherals::motor>(node_name + "/setMotorReverse")),
    motor_set_all(nh.serviceClient<peripherals::motor>(node_name + "/setAllMotors")),
    motor_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopMotors")),
    motors_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopAllMotors")),
    EMatrix{
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0},
        {0,0,0,0,0}
    } { }

void thrust_controller::generate_thrust_val(const navigation::nav::ConstPtr &msg)
{
    ROS_INFO("X: %.2f Y: %.2f Z: %.2f Speed: %d\n"
    , msg->direction.x, msg->direction.y, msg->direction.z, msg->speed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");
    thrust_controller tc("motor_controller");
    ros::Subscriber joy = nh.subscribe<navigation::nav>
        ("/nav/navigation", 1, &thrust_controller::generate_thrust_val, &tc);
    ros::spin();
    return 0;
}