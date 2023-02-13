
#include <ros/ros.h>
#include "hpe2udp.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hpe2udp_node");
    ros::NodeHandle nodeHandle("~"); 
    HpeToUdp hpeToUdp(nodeHandle); 

    int num_threads = 1;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    hpeToUdp.run(); 

    return 0; 

}