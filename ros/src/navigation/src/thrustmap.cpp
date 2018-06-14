#include <ros/ros.h>
#include "navigation/nav.h"

/*  The matrix E was precomputed in Matlab/Octave then dumped here. 
    The method, summarized below, can be found in Chris Carpenter's Report
    ..
*/

const float e_matrix[3][7] = {
        {0.35355, 0.35292, 0.35355, 0.35292, 0.00141, 0.00141, 0.02099},
        {0.35355, -0.35292, 0.35355, -0.35292, -0.00141, -0.00141, -0.02099},
        {-0.00000, 0.00199, 0.00000, 0.00199, 0.49554, 0.49554, -0.06646}};


void generate_thrust_val(const navigation::nav::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrustmap");
    ros::NodeHandle nh("~");
    ros::Subscriber joy = nh.subscribe<navigation::nav>("/nav/navigation", 1, generate_thrust_val);
    
    ros::spin();
    return 0;
}