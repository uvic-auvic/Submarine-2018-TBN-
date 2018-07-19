#include <ros/ros.h>
#include "controllers.hpp"
#include "navigation/thrusts.h"
#include "peripherals/rpms.h"
#include "peripherals/motor_enums.h"
#include "peripherals/motors.h"

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
    void compute_pwms(peripherals::motors &srv);
private:
    int16_t rpm_to_pwm(double rpm);

    // ROS
    ros::NodeHandle nh;

    // Controllers 
    std::vector<std::unique_ptr<velocity_controller>> thruster_controllers;

    // Messages
    std::vector<double> current_rpm_des;
    std::vector<double> current_rpm_act;

    // Variables
    std::vector<int16_t> pwms;
};

rpm_controller::rpm_controller():
    nh(ros::NodeHandle("~")),
    current_rpm_des(NUM_MOTORS),
    current_rpm_act(NUM_MOTORS),
    pwms(NUM_MOTORS)
{
    double loop_rate, min_rpm, max_rpm, Kp, Ki;
    nh.getParam("loop_rate", loop_rate);
    nh.getParam("min_rpm", min_rpm);
    nh.getParam("max_rpm", max_rpm);
    nh.getParam("Kp", Kp);
    nh.getParam("Ki", Ki);

    for(int i = 0; i < NUM_MOTORS; i++)
    {
        thruster_controllers.push_back(std::unique_ptr<velocity_controller>(new velocity_controller(min_rpm, max_rpm, 1.0/loop_rate, Kp, Ki)));
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

void rpm_controller::compute_pwms(peripherals::motors &srv)
{
    // Get the required PWMs to correct the RPM
    for(int i = 0; i < NUM_MOTORS; i++)
    {
        double signed_rpm_act = (this->pwms[i] < 0) ? (-this->current_rpm_act[i]) : (this->current_rpm_act[i]);
        double corrected_rpm = this->thruster_controllers[i]->calculate(this->current_rpm_des[i], signed_rpm_act);
        this->pwms[i] = this->rpm_to_pwm(corrected_rpm);
    }
    srv.request.pwms = this->pwms;
}

void rpm_controller::receive_desired_rpms(const peripherals::rpms::ConstPtr &msg)
{
    current_rpm_des = msg->rpms;
}

void rpm_controller::receive_actual_rpms(const peripherals::rpms::ConstPtr &msg)
{
    current_rpm_act = msg->rpms;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rpm_controller");
    ros::NodeHandle nh("~");
    rpm_controller rpm_ctrl;
    ros::Subscriber des_rpms = nh.subscribe<peripherals::rpms>("/nav/rpms", 1, &rpm_controller::receive_desired_rpms, &rpm_ctrl);
    ros::Subscriber act_rpms = nh.subscribe<peripherals::rpms>("/motor_controller/MotorRpms", 1, &rpm_controller::receive_actual_rpms, &rpm_ctrl);
    ros::ServiceClient mtrs_set_all = nh.serviceClient<peripherals::motors>("/motor_controller/setAllMotorsPWM");
    ros::Publisher pub_pwms = nh.advertise<navigation::thrusts>("/rpm_control/pwms", 5);

    int loop_rate;
    nh.getParam("loop_rate", loop_rate);
    ros::Rate r(loop_rate);
    while(ros::ok())
    {
        // Compute the PWMs to from the RPM control systems
        peripherals::motors srv;
        rpm_ctrl.compute_pwms(srv);

        // Get PWMs
        navigation::thrusts msg;
        msg.thruster_pwms = srv.request.pwms;
        pub_pwms.publish(msg);

        // Set the motors
        mtrs_set_all.call(srv);
       
        // ROS stuff
        ros::spinOnce();
        r.sleep();
    }   

    return 0;
}
