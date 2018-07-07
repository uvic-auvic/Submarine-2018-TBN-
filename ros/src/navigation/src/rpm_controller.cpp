#include <ros/ros.h>
#include "controllers.hpp"
#include "peripherals/rpms.h"
#include "peripherals/motor_enums.h"

#define NUM_MOTORS (8)

#define MAX_FORWARD_PWM (400)
#define MIN_FORWARD_PWM (100)
#define MAX_REVERSE_PWM (300)
#define MIN_REVERSE_PWM (100)

class rpm_controller
{
public:
    rpm_controller();
    void receive_desired_rpms(const peripherals::rpms::ConstPtr &msg);
    void receive_actual_rpms(const peripherals::rpms::ConstPtr &msg);
private:
    int16_t rpm_to_pwm(double rpm);

    // ROS
    ros::NodeHandle nh;

    // Controllers 
    std::vector<velocity_controller> thruster_controllers;

    // Messages
    peripherals::rpms::ConstPtr current_rpm_des;
    peripherals::rpms::ConstPtr current_rpm_act;
};

rpm_controller::rpm_controller():
    nh(ros::NodeHandle("~")),
    current_rpm_des(peripherals::rpms::ConstPtr(new peripherals::rpms())),
    current_rpm_act(peripherals::rpms::ConstPtr(new peripherals::rpms()))
{
    double loop_rate, min_rpm, max_rpm, Kp, Ki;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_rpm", min_rpm);
    nh.getParam("max_rpm", max_rpm);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);

    for(int i = 0; i < NUM_MOTORS; i++)
    {
        thruster_controllers.push_back(velocity_controller(min_rpm, max_rpm, 1.0/loop_rate, Kp, Ki));
    }
}

int16_t rpm_controller::rpm_to_pwm(double rpm)
{
    int16_t pwm = 0;
    if(rpm > 0)
    {
        pwm = (int16_t)((rpm + 43.47222) / 6.522501);
        if(pwm > MAX_FORWARD_PWM)
        {
            pwm = MAX_FORWARD_PWM;
        }
        else if(pwm < MIN_FORWARD_PWM)
        {
            pwm = 0;
        }
    }
    else
    {
        pwm = (int16_t)((-rpm + 225.50476) / 9.554571);
        if(pwm > MAX_REVERSE_PWM)
        {
            pwm = MAX_REVERSE_PWM;
        }
        else if(pwm < MIN_REVERSE_PWM)
        {
            pwm = 0;
        }
        pwm = -pwm;
    }

    return pwm;
}

void rpm_controller::receive_desired_rpms(const peripherals::rpms::ConstPtr &msg)
{
    current_rpm_des = msg;
}

void rpm_controller::receive_actual_rpms(const peripherals::rpms::ConstPtr &msg)
{
    current_rpm_act = msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rpm_controller");
    ros::NodeHandle nh("~");
    rpm_controller rpm_ctrl;
    ros::Subscriber des_rpms = nh.subscribe<peripherals::rpms>("/nav/rpms", 1, &rpm_controller::receive_desired_rpms, &rpm_ctrl);
    ros::Subscriber act_rpms = nh.subscribe<peripherals::rpms>("/motor_controller/MotorRpms", 1, &rpm_controller::receive_actual_rpms, &rpm_ctrl);
    return 0;
}
