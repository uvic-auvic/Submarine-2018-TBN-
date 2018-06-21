#include <ros/ros.h>
#include "navigation/nav.h"
#include "peripherals/motor.h"
#include "peripherals/motors.h"

#define NUMBER_OF_THRUSTERS     (8)

#define MAX_FORWARD_COMMAND     (110)
#define MIN_FORWARD_COMMAND     (30)
#define MAX_REVERSE_COMMAND     (85)
#define MIN_REVERSE_COMMAND     (30)

#define E_MATRIX_ROWS           (8)
#define E_MATRIX_COLUMNS        (6)

class thrust_controller
{
public:
    thrust_controller(std::string node_name);
    void generate_thrust_val(const navigation::nav::ConstPtr &msg);
    void do_thrust_matrix(float tau[6], float thrust_value[8]);
private:
    ros::NodeHandle nh;
    ros::ServiceClient motor_forward;
    ros::ServiceClient motor_reverse;
    ros::ServiceClient motor_set_all;
    ros::ServiceClient motor_stop;
    ros::ServiceClient motors_stop;

    int thrust_to_command(float thrust);
    
    const double E_inverse[E_MATRIX_ROWS][E_MATRIX_COLUMNS] = {
        {0.0355383007316710, 0.0626175759204514, 0.226381127185469, -0.0223954134193317, -0.0186650739136927, -1.52872506320456e-17},
        {0.0355383007316709, -0.0626175759204520, 0.224903029899793, 0.0223954134193317, -0.0186650739136927, -1.61546123700340e-17},
        {-0.0355383007316710, 0.0626175759204510, 0.275096970100207, -0.0223954134193317, 0.0186650739136927, -1.68051336735253e-17},
        {-0.0355383007316710, -0.0626175759204516, 0.273618872814531, 0.0223954134193317, 0.0186650739136927, -1.03270256929244e-17},
        {0.000627995739502264, 0.475165623028772, 3.45847021392878e-17, 7.96797484263443e-17, 3.25599464924553e-18, 0.0190301739243126},
        {-0.000627995739502344, 0.524834376971227, -4.03195175199153e-17, 9.52532044474495e-17, 2.26581313390525e-18, -0.0190301739243126},
        {0.500349415164236, -0.0138177814947827, -1.91896838471135e-16, -1.39889909101611e-17, -1.04083408558608e-16, 0.0105883383101780},
        {0.499650584835764, 0.0138177814947827, 1.17507691233554e-16, -2.83689203505640e-17, -1.01047642475649e-16, -0.0105883383101780}
    };
};

void thrust_controller::do_thrust_matrix(float tau[E_MATRIX_COLUMNS], float thrust_value[E_MATRIX_ROWS]){
    // Thrusters = (E^-1) * tau
    for(int r = 0; r < E_MATRIX_ROWS; r++){
        thrust_value[r] = 0;
        for(int c = 0; c < E_MATRIX_COLUMNS; c++){
            thrust_value[r] += E_inverse[r][c] * tau[c];
        }
    }
}

int thrust_controller::thrust_to_command(float thrust){
    //forward: rpm = 21.74167 (command) - 43.47222
    //forward: thrust = (x^2) * 0.00000389750967963493  x is rpm, thrust is in newtons

    //reverse: rpm = 31.84857 (command) - 225.50476
    //reverse: thrust = (x^2) * -0.00000396914500683942  x is rpm, thrust is in newtons

    int command = 0;
    if(thrust > 0){
        unsigned int rpm = sqrt(thrust / 0.00000389750967963493);
        command = (int)((rpm + 43.47222) / 21.74167);

        if(command > MAX_FORWARD_COMMAND){
            command = MAX_FORWARD_COMMAND;
        }else if(command < MIN_FORWARD_COMMAND){
            command = 0;
        }
    }else{
        unsigned int rpm = sqrt(thrust / 0.00000396914500683942);
        command = (int)((rpm + 225.50476) / 31.84857);
        
        if(command > MAX_REVERSE_COMMAND){
            command = MAX_REVERSE_COMMAND;
        }else if(command < MIN_REVERSE_COMMAND){
            command = 0;
        }
    }

    return command;
}

thrust_controller::thrust_controller(std::string node_name) :
    nh(ros::NodeHandle("~")),
    motor_forward(nh.serviceClient<peripherals::motor>(node_name + "/setMotorForward")),
    motor_reverse(nh.serviceClient<peripherals::motor>(node_name + "/setMotorReverse")),
    motor_set_all(nh.serviceClient<peripherals::motor>(node_name + "/setAllMotors")),
    motor_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopMotors")),
    motors_stop(nh.serviceClient<peripherals::motor>(node_name + "/stopAllMotors"))
    { }

void thrust_controller::generate_thrust_val(const navigation::nav::ConstPtr &msg)
{
    ROS_INFO("X: %.2f Y: %.2f Z: %.2f Speed: %d\n"
    , msg->direction.x, msg->direction.y, msg->direction.z, msg->speed);
    
    float tau[E_MATRIX_COLUMNS] = {
        (float)(msg->direction.x), 
        (float)(msg->direction.y), 
        (float)(msg->direction.z), 
        msg->rotation.pitch,
        msg->rotation.roll,
        msg->rotation.yaw
    };   

    float thruster_vals[NUMBER_OF_THRUSTERS] = {0.0};
    this->do_thrust_matrix(tau, thruster_vals);

    for(int i = 0; i < NUMBER_OF_THRUSTERS; i++){
        thruster_vals[i] = this->thrust_to_command(thruster_vals[i]);
    }
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