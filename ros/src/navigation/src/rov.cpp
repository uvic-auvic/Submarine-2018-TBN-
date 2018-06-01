#include <ros/ros.h>
#include <sstream>
#include <string>
#include "navigation/joystick.h"
#include "navigation/keyboard.h"
#include "navigation/nav.h"

class rov_mapper {
public:
    rov_mapper(); 
    void recieve_joystick(const navigation::joystick::ConstPtr& msg);
    void recieve_keyboard(const navigation::keyboard::ConstPtr& msg);
private:
    ros::NodeHandle nh;
    ros::Publisher nav_pub;
    bool W_pressed;
    bool A_pressed;
    bool S_pressed;
    bool D_pressed;

};

rov_mapper::rov_mapper() 
    :   nh(ros::NodeHandle("~")),
        nav_pub(nh.advertise<navigation::nav>("/nav/nav", 5)),
        W_pressed(false),
        A_pressed(false),
        S_pressed(false),
        D_pressed(false) {}

void rov_mapper::recieve_joystick(const navigation::joystick::ConstPtr& msg) {
    bool fire = msg->buttons[0];
    bool up = msg->buttons[1];
    bool down = msg->buttons[2];
    int16_t y_axis = msg->axes[0];
    int16_t z_axis = msg->axes[1];
    int16_t x_axis = 25;
    
    if (W_pressed) {
        ROS_INFO("Forwards X: %d Y: %d Z: %d", x_axis, y_axis, z_axis);
        return; //if both W and S are pressed, prefer W
    } 
    
    if (S_pressed) {
        ROS_INFO("Backwards X: %d Y: %d Z: %d", x_axis, y_axis, z_axis);
    }

}

void rov_mapper::recieve_keyboard(const navigation::keyboard::ConstPtr& msg) {
    W_pressed = msg->W_pressed;
    A_pressed = msg->A_pressed;
    S_pressed = msg->S_pressed;
    D_pressed = msg->D_pressed;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rov");
    ros::NodeHandle nh("~");
    
    rov_mapper controls;
    ros::Subscriber joy = nh.subscribe<navigation::joystick>("/nav/joystick", 20, &rov_mapper::recieve_joystick, &controls);
    ros::Subscriber key = nh.subscribe<navigation::keyboard>("/nav/keyboard", 20, &rov_mapper::recieve_keyboard, &controls);
    ros::spin();
    return 0;

}