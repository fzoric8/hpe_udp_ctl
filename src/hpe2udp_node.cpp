
#include <ros/ros.h>
#include "hpe2udp.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hpe2udp_node");
    ros::NodeHandle nodeHandle("~"); 
    HpeToUdp hpeToUdp(nodeHandle); 

    int num_threads = 5;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start();
    //ros::spinOnce(); 
    hpeToUdp.run(); 

    return 0; 

}