#include <ros/ros.h>

int main(int argc, char** argv)
{       
    ros::init(argc, argv, "hydrophones");
    ros::NodeHandle nh("~");

    std::string device_id;
    nh.getParam("device_id", device_id);

}
